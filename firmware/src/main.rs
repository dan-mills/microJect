#![no_std]
#![no_main]

// Note local copies of systick-timer crate because I could not get the dependencies to resolve

use core::sync::atomic::Ordering::Relaxed;
use core::sync::atomic::{self, AtomicBool, AtomicU16};
//use cortex_m::Peripherals;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, AdcChannel, Config as ADC_Config, SampleTime};
use embassy_stm32::gpio::{AnyPin, Input, Level, Output, Pull, Speed};
use embassy_stm32::rcc::{Pll, PllMul, PllPreDiv, PllSource, Sysclk};
use embassy_stm32::usart::{Config as USART_Config, Uart};
use embassy_stm32::{Peri, adc, bind_interrupts, dma, peripherals, usart};
use embassy_time::{Duration, Ticker};
use panic_halt as _;
use static_cell::StaticCell;

mod modbus;

const DMA_BUF_LEN: usize = 64;

const COILS: usize = 4; // Enables for the PWM Outputs.
const INPUTS: usize = 2; // Two opto isolators.
const IREGISTERS: usize = 2; // Two Analog inputs.
const HREGISTERS: usize = 8; // Four pairs of PWM duration and frequency values.
const DEFAULT_MODBUS_ADDRESS: u8 = 8;
const SERVER_ID: &[u8] = b"microJect";

// Aggravatingly Rust does not have the copy trait for atomic types so initialising statics is a pain in the arse.
static COIL_DATA: [atomic::AtomicBool; COILS] = [
    AtomicBool::new(false),
    AtomicBool::new(false),
    AtomicBool::new(false),
    AtomicBool::new(false),
];
static INPUT_DATA: [atomic::AtomicBool; INPUTS] = [AtomicBool::new(false), AtomicBool::new(false)];
static IREGISTER_DATA: [atomic::AtomicU16; IREGISTERS] = [AtomicU16::new(0), AtomicU16::new(0)];
static HREGISTER_DATA: [atomic::AtomicU16; HREGISTERS] = [
    AtomicU16::new(1),   // 1ms ON
    AtomicU16::new(100), //100ms period, a sane default I think Nothing will happen anyway until the appropriate coil is turned on.
    AtomicU16::new(1),
    AtomicU16::new(100),
    AtomicU16::new(1),
    AtomicU16::new(100),
    AtomicU16::new(1),
    AtomicU16::new(100),
];

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>; //UART Itself
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>; // ADC
    DMA1_CHANNEL2_3 => dma::InterruptHandler<peripherals::DMA1_CH2>; // UART DMA
    DMA1_CHANNEL4_5 => dma::InterruptHandler<peripherals::DMA1_CH5>; //UART DMA
});

// Periodically read the analog and opto isolated inputs, and update the state as indicated.
#[embassy_executor::task]
async fn input_read(
    adc: Peri<'static, peripherals::ADC1>,
    dma: Peri<'static, peripherals::DMA1_CH1>,
    adc_pin0: Peri<'static, peripherals::PA4>, //ADC input 1
    adc_pin1: Peri<'static, peripherals::PA5>, //ADC input 2
    opto_pin0: Peri<'static, AnyPin>,          // Opto input 1
    opto_pin1: Peri<'static, AnyPin>,          // Opto input 2
    local_control: bool, // True if local switches and pots are being used to control the output
) {
    static ADC_DMA_BUF: StaticCell<[u16; DMA_BUF_LEN]> = StaticCell::new();
    let adc_dma_buf = ADC_DMA_BUF.init([0u16; DMA_BUF_LEN]);
    let mut adc_con = ADC_Config::default();
    adc_con.resolution = Some(embassy_stm32::adc::Resolution::Bits12);
    adc_con.clock = embassy_stm32::adc::Clock::Sync(adc::SyncDiv::Div4);
    defmt::trace!("Configuring ADC");
    let adc = Adc::new_blocking(adc, adc_con);
    adc.sample_time_for_us(100);
    defmt::trace!("Configuring ADC DMA");
    let mut ring_buffered_adc = adc.into_ring_buffered(
        dma,
        adc_dma_buf,
        Irqs,
        [
            (adc_pin0.degrade_adc(), SampleTime::Cycles2395),
            (adc_pin1.degrade_adc(), SampleTime::Cycles2395),
        ]
        .into_iter(),
        None,
    );
    defmt::trace!("Configuring Opto inputs");
    let opto0 = Input::new(opto_pin0, Pull::Up);
    let opto1 = Input::new(opto_pin1, Pull::Up);
    let mut measurements = [0u16; DMA_BUF_LEN / 2];
    let mut ticker = Ticker::every(Duration::from_millis(50));
    loop {
        match ring_buffered_adc.read(&mut measurements).await {
            //This runs at 20 Hz or so, every time the DMA accumilates a ring full of samples
            Ok(_) => {
                let mut adc1_avg: u32 = 0;
                let mut adc2_avg: u32 = 0;
                for i in measurements.chunks(2) {
                    adc1_avg += i[1] as u32;
                    adc2_avg += i[0] as u32;
                }
                // put the readings into the modbus register storage
                IREGISTER_DATA[0].store(adc1_avg as u16, core::sync::atomic::Ordering::Relaxed);
                IREGISTER_DATA[1].store(adc2_avg as u16, core::sync::atomic::Ordering::Relaxed);
            }
            Err(e) => {
                defmt::error!("Error: {:?}", e);
            }
        }
        // Read the opto isolated inputs, might as well do this here, it runs sufficiently often
        INPUT_DATA[0].store(opto0.is_low(), core::sync::atomic::Ordering::Relaxed);
        INPUT_DATA[1].store(opto1.is_low(), core::sync::atomic::Ordering::Relaxed);
        //defmt::info!("Optos {} {}", opto0.is_low(), opto1.is_low());
        if local_control {
            // Switch on the PWM based on the isolated inputs
            COIL_DATA[0].store(opto0.is_low(), core::sync::atomic::Ordering::Relaxed);
            COIL_DATA[1].store(opto1.is_low(), core::sync::atomic::Ordering::Relaxed);
            // On times have defaulted to 1ms, so varying the period varies the flow rate
            // Make the pots read the expected way around, more voltage increases the flow
            let pot1 = 1000 - ((IREGISTER_DATA[0].load(Relaxed) / 65).clamp(0, 999));
            let pot2 = 1000 - ((IREGISTER_DATA[1].load(Relaxed) / 65).clamp(0, 999));
            HREGISTER_DATA[1].store(pot1, Relaxed); // CH0 period
            HREGISTER_DATA[3].store(pot2, Relaxed); // CH1 period
        }
        // This chip is stupid, you cannot get the ADC to run slowly enough to avoid bogging down everything else
        // so stop it, wait a bit and then kick it off again, bit inefficient because we wake up, kick off the adc conversion and then sleep again
        // But whatever it is good enough.
        ring_buffered_adc.stop();
        ticker.next().await;
        ring_buffered_adc.start();
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // System configuration
    let mut cnf: embassy_stm32::Config = Default::default();
    cnf.rcc.sys = Sysclk::Pll1P;
    cnf.rcc.hsi = true;
    cnf.rcc.pll = Some(Pll {
        src: PllSource::HSI,
        prediv: PllPreDiv::Div2,
        mul: PllMul::Mul12,
    });
    // UART Clock
    cnf.rcc.mux.usart1sw = embassy_stm32::rcc::mux::Usart1sw::Sys;
    let p = embassy_stm32::init(cnf);
    let sw1 = Input::new(p.PA9, Pull::Up); // SW1 selects if modbus should be overridden by the analog and opto inputs
    let sw2 = Input::new(p.PA10, Pull::Up); // Alternate modbus address
    let sw3 = Input::new(p.PA11, Pull::Up); // Baud rate select MSB
    let sw4 = Input::new(p.PA12, Pull::Up); // Baud rate select LSB

    // USART Configuration
    let mut rate = 0;
    if sw3.is_low() {
        rate += 2;
    }
    if sw4.is_low() {
        rate += 1;
    }
    let mut baud = 4800;
    if rate == 1 {
        baud = 9600;
    } else if rate == 2 {
        baud = 19200;
    } else if rate == 3 {
        baud = 115200;
    }
    defmt::info!("Baud rate: {}", baud);
    let mut usart_config = USART_Config::default();
    usart_config.baudrate = baud;
    usart_config.data_bits = embassy_stm32::usart::DataBits::DataBits8;
    usart_config.eager_reads = core::option::Option::Some(1);
    usart_config.parity = embassy_stm32::usart::Parity::ParityEven;
    usart_config.stop_bits = embassy_stm32::usart::StopBits::STOP1;
    usart_config.tx_config = embassy_stm32::usart::OutputConfig::PushPull;
    usart_config.de_config = embassy_stm32::usart::OutputConfig::PushPull;
    let usart = Uart::new_with_de(
        p.USART1,
        p.PA2,
        p.PA3,
        p.PA1,
        p.DMA1_CH2,
        p.DMA1_CH5,
        Irqs,
        usart_config,
    )
    .unwrap();
    defmt::info!("USART initialized");

    let modbus_address = sw2.is_low() as u8 + DEFAULT_MODBUS_ADDRESS; // address is 8 or 9 depending on switch position
    defmt::info!("Modbus address: {}", modbus_address);
    if sw1.is_low() {
        defmt::info!("Overriding modbus input with local inputs");
    }

    let mut led1 = Output::new(p.PB3, Level::Low, Speed::Low);
    let led2 = Output::new(p.PB4, Level::Low, Speed::Low);
    let led3 = Output::new(p.PB5, Level::Low, Speed::Low);

    let mut pwm_pins = [
        Output::new(p.PA7, Level::Low, Speed::Low),
        Output::new(p.PA6, Level::Low, Speed::Low),
        Output::new(p.PA8, Level::Low, Speed::Low),
        Output::new(p.PB0, Level::Low, Speed::Low),
    ];
    // Kick off the ADC and GPI task
    spawner.spawn(
        input_read(
            p.ADC1,
            p.DMA1_CH1,
            p.PA4,
            p.PA5,
            p.PF1.into(),
            p.PF0.into(),
            sw1.is_low(),
        )
        .unwrap(),
    );
    // Kick off the modbus interface task
    spawner
        .spawn(modbus::modbus_task(modbus_address, baud, sw1.is_low(), usart, led2, led3).unwrap());
    // spin making the PWM signals
    let mut led_tbase = 0u16;
    let mut counters: [u16; 4] = [0u16; 4];
    let mut timebase = Ticker::every(Duration::from_millis(1));
    loop {
        for i in 0..4 {
            let period = HREGISTER_DATA[2 * i + 1].load(Relaxed);
            let pulse = HREGISTER_DATA[2 * i].load(Relaxed);
            let enable = COIL_DATA[i].load(Relaxed);
            defmt::trace!(
                "Counter {} period {}, duration {}, enabled {}",
                i,
                period,
                pulse,
                enable
            );
            if !enable {
                // Ensure we get a pulse as soon as we write the enable coil even at low rates
                counters[i] = 0;
            }
            counters[i] = counters[i].wrapping_add(1);
            if counters[i] >= period {
                counters[i] = 0;
            }
            if counters[i] < pulse && enable {
                pwm_pins[i].set_high();
            } else {
                pwm_pins[i].set_low();
            }
        }
        // CPU run indicator blinky
        led_tbase = led_tbase.wrapping_add(1);
        if led_tbase == 100 {
            led1.toggle(); // CPU Running indicator
            led_tbase = 0;
        }
        timebase.next().await;
    }
}
