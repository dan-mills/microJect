#![no_std]
#![no_main]

// Note local copies of systick-timer crate because I could not get the dependencies to resolve
//mod timer;
//use timer::Timer as Coretimer;
//mod embassy_driver;
//use embassy_driver::SystickDriver;

//use async_std::task;
use core::sync::atomic::{self, AtomicBool, AtomicU16};
//use cortex_m::Peripherals;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, AdcChannel, Config as ADC_Config, SampleTime};
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pin, Pull, Speed};
use embassy_stm32::rcc::{Pll, PllMul, PllPreDiv, PllSource, Sysclk};
use embassy_stm32::time::hz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::{Channel, simple_pwm};
use embassy_stm32::usart::{Config as USART_Config, Uart};
use embassy_stm32::{Config, Peri, adc, bind_interrupts, dma, peripherals, usart};
use embassy_time::{Delay, Duration, Instant, Ticker, Timer, with_timeout};
//use embassy_time_driver::*;
use panic_halt as _;
use static_cell::StaticCell;
//use systick_timer::SystickDriver;

//use embassy_stm32::_generated::interrupt::typelevel::DMA1_CHANNEL2_3
//use embassy_stm32::_generated::peripherals::DMA1_CH2

const DMA_BUF_LEN: usize = 64;
const COILS: usize = 4; // Enables for the PWM Outputs.
const INPUTS: usize = 2; // Two opto isolators.
const IREGISTERS: usize = 2; // Two Analog inputs.
const HREGISTERS: usize = 8; // Four pairs of PWM duration and frequency values.

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
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
];

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>; // ADC
    DMA1_CHANNEL2_3 => dma::InterruptHandler<peripherals::DMA1_CH2>;
    DMA1_CHANNEL4_5 => dma::InterruptHandler<peripherals::DMA1_CH5>;
});

//embassy_time_driver::time_driver_impl!(static DRIVER: SystickDriver<4>
//    = SystickDriver::new(48_000_000, 47_999));
//#[cortex_m_rt::exception]
//fn SysTick() {
//    DRIVER.systick_interrupt();
//}

// Periodically read the analog and opto isolated inputs, and update the state as indicated.
#[embassy_executor::task]
async fn input_read(
    adc: Peri<'static, peripherals::ADC1>,
    dma: Peri<'static, peripherals::DMA1_CH1>,
    adc_pin0: Peri<'static, peripherals::PA4>,  //ADC input 1
    adc_pin1: Peri<'static, peripherals::PA5>,  //ADC input 2
    opto_pin0: Peri<'static, peripherals::PF1>, // Opto input 1
    opto_pin1: Peri<'static, peripherals::PF0>, // Opto input 2
) {
    static ADC_DMA_BUF: StaticCell<[u16; DMA_BUF_LEN]> = StaticCell::new();
    let adc_dma_buf = ADC_DMA_BUF.init([0u16; DMA_BUF_LEN]);
    let mut adc_con = ADC_Config::default();
    adc_con.resolution = Some(embassy_stm32::adc::Resolution::Bits12);
    adc_con.clock = embassy_stm32::adc::Clock::Sync((adc::SyncDiv::Div4));
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
    defmt::trace!("Starting ADC and switch input loop");
    let mut ticker = Ticker::every(Duration::from_millis(50));
    loop {
        match ring_buffered_adc.read(&mut measurements).await {
            //This runs at 20 Hz or so, every time the DMA accumilates a ring full of samples
            Ok(_) => {
                let mut adc1_avg: u32 = 0;
                let mut adc2_avg: u32 = 0;
                for i in measurements.chunks(2) {
                    adc1_avg += i[0] as u32;
                    adc2_avg += i[1] as u32;
                }
                // put the readings into the modbus register storage
                IREGISTER_DATA[0].store((adc1_avg >> 2) as u16, core::sync::atomic::Ordering::Relaxed);
                IREGISTER_DATA[1].store((adc1_avg >> 2) as u16, core::sync::atomic::Ordering::Relaxed);
                info!("samples: {} {}", adc1_avg / 655, adc2_avg / 655);
            }
            Err(e) => {
                warn!("Error: {:?}", e);
            }
        }
        // Read the opto isolated inputs, might as well do this here, it runs sufficiently often
        INPUT_DATA[0].store(opto0.is_high(), core::sync::atomic::Ordering::Relaxed);
        INPUT_DATA[1].store(opto1.is_high(), core::sync::atomic::Ordering::Relaxed);
        defmt::info!("Optos {} {}", opto0.is_high(), opto1.is_high());
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
    // Core timer as system time
    //let mut periph = Peripherals::take().unwrap();
    //DRIVER.start(&mut periph.SYST);

    // USART Configuration
    let mut usart_config = USART_Config::default();
    //config.baudrate = 9600;
    usart_config.data_bits = embassy_stm32::usart::DataBits::DataBits8;
    usart_config.eager_reads = core::option::Option::Some(1);
    usart_config.parity = embassy_stm32::usart::Parity::ParityEven;
    usart_config.stop_bits = embassy_stm32::usart::StopBits::STOP1;
    usart_config.tx_config = embassy_stm32::usart::OutputConfig::PushPull;
    usart_config.de_config = embassy_stm32::usart::OutputConfig::PushPull;
    let mut usart = Uart::new_with_de(
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
    let mut led1 = Output::new(p.PB3, Level::High, Speed::Low);
    let mut led2 = Output::new(p.PB4, Level::High, Speed::Low);
    let mut led3 = Output::new(p.PB5, Level::High, Speed::Low);
    let sw1 = Input::new(p.PA9, Pull::Up);
    let sw2 = Input::new(p.PA10, Pull::Up);
    let sw3 = Input::new(p.PA11, Pull::Up);
    let sw4 = Input::new(p.PA12, Pull::Up);

    let ch1_pin = PwmPin::new(p.PA7, OutputType::PushPull);
    let mut pwm1 = SimplePwm::new(p.TIM14, Some(ch1_pin), None, None, None, hz(20), Default::default());
    let mut ch1 = pwm1.ch1();
    ch1.enable();
    ch1.set_duty_cycle_percent(2);

    let ch2_pin = PwmPin::new(p.PA6, OutputType::PushPull);
    let mut pwm2 = SimplePwm::new(p.TIM16, Some(ch2_pin), None, None, None, hz(50), Default::default());
    let mut ch2 = pwm2.ch1();
    ch2.enable();
    ch2.set_duty_cycle_percent(10);

    let ch3_pin = PwmPin::new(p.PA8, OutputType::PushPull);
    let mut pwm3 = SimplePwm::new(p.TIM1, Some(ch3_pin), None, None, None, hz(5), Default::default());
    let mut ch3 = pwm3.ch1();
    ch3.enable();
    ch3.set_duty_cycle_percent(50);

    //let ch4_pin = PwmPin::new(p.PB0, OutputType::PushPull);
    //let mut pwm4 = SimplePwm::new(p.TIM3, None, None, Some(ch4_pin), None, Hz(10), Default::default());
    //let mut ch4 = pwm4.ch3();
    //ch4.enable();
    //pwm4.set_frequency(Hz(100));
    //  ch4.set_duty_cycle_percent(5);

    // Kick off the ADC and GPI task
    spawner.spawn(input_read(p.ADC1, p.DMA1_CH1, p.PA4, p.PA5, p.PF1, p.PF0).unwrap());
    loop {
        Timer::after_millis(150).await;
        led1.toggle();
        //op1.set_high();
        //op2.set_low();
        //op3.set_low();
        //op4.set_low();
        //Timer::after_micros(1000).await;
        //op1.set_low();

        //led3.toggle();
    }
}
/*
#[derive(Default)]
#[derive(Debug)]
enum ModbusState {
    #[default]
    WaitForIdle3_5, // 3.5 charcter times
    WaitForIdle2_0, //
    RxData,
    TxData,
}

const CRC_UPPER: [u8; 256] = [
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
    0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00,
    0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80,
    0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
    0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
];

const CRC_LOWER: [u8; 256] = [
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D,
    0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB,
    0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2,
    0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37,
    0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8,
    0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24,
    0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63,
    0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE,
    0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F,
    0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C,
    0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46,
    0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40,
];

#[derive(Debug)]
#[derive(Default)]
struct ModbusSlave {
    modbus_state : ModbusState,
    rx_buffer : [u8;256],
    tx_buffer : [u8;256],
    rx_chars : usize,
    last_rx_time : Instant,
    broadcast : bool,
    mute : bool,
    address : u8,
    gap_detect : Duration,
    idle_detect : Duration,
    bus_timeout : Duration,
    // exceptions and log
    exception_status : u16,
    event_log :[u16;64],
    event_log_count : usize,
    // statistics counters
    bus_message_count : u16,
    bus_comm_error_count : u16,
    bus_exception_count : u16,
    server_message_count : u16,
    server_no_response_count : u16,
    server_nak_count : u16,
    server_busy_count : u16,
    bus_overrun_count : u16,
}

impl modbusslave {
    // We look for a 3.5 charcter idle time, minus some time that may have already expired
    fn wait_for_idle_some_spent (data : Option<u8>, expired : Duration) {
        let delta = Instant::now() - last_rx_time + expired;
        match data {
             Some(d) =>{
                 if delta > idle_detect {
                     if (d == 0x00) || (d == address) {
                        // Got the start of a message and it is addressed to us (or a broadcast)!
                        rx_buffer[0] = d;
                        rx_chars = 1;
                        broadcast = (d == 0x00);
                        modbus_state = ModbusState::RxData;
                     } else if delta > bus_timeout {
                         //TODO Switch off output
                     }
                 } // else we have not seen sufficient gap, so we drop this
                 // In either case we have seen bus traffic so reset the timer
                 last_rx_time = Instant::now();
             }
             None =>{ // UART timerout, see if we need to shutdown outputs for no message
                 if delta > bus_timeout {
                     // TODO, switch off outputs
                 }
             }
        }
    }

    fn recv (data : Option<u8>) {
        match modbus_state {
            ModbusState::WaitForIdle3_5 => {
                wait_for_idle_some_spent (data, Duration::from_micros(0));
            }
            ModbusState::WaitForIdle2_0 => {
              wait_for_idle_some_spent (data, gap_detect);
            }
            ModbusState::RxData => {
                let delta = Instant::now() - last_rx_time;
                if delta > gap_detect {
                    // This might be the first byte of the NEXT message so
                    // process what we have and pass this to our next state if appropriate
                    if rx_chars > 3 { // Bare minimum is address, command and two bytes of CRC
                        let crc = crc16(&rx_buffer[0 .. rx_chars - 2]);
                        if (rx_buffer[rx_chars -2] == (crc & 0xff)) &&
                            (rx_buffer[rx_chars -1] == (crc >>8)){
                            // CRC pass

                        }
                    }




                }
                match (data){
                    Some(d) => {


                    }




                }

            }







            }

        }



        match data {
          Some(ch) => {




          }
        None => {




        }



        }





    }



    fn crc16(d: &[u8]) -> u16 {
        let mut hi: u8 = 0xFF;
        let mut low: u8 = 0xFF;
        for ch in d {
            let index = low ^ *ch;
            low = hi ^ CRC_UPPER[index as usize];
            hi = CRC_LOWER[index as usize];
        }
        (hi as u16) << 8 | (low as u16)
    }


}
*/

/*
#[embassy_executor::task]
async fn modbus(address: u8) {
    let mut message: [u8; 256];
    let mut length = 0;
    let mut bus_state = ModbusState::WaitForIdle;
    let mut last_rx_time = Instant::now();
    let mut broadcast = false;
    let mut mute = false;
    let mut buf = [0u8; 1];
    let gap_detect = Duration::from_micros(750);
    let idle_detect = Duration::from_micros(1750);
    let bus_off_detect = Duration::from_secs(2);
    loop {
        match bus_state {
            ModbusState::WaitForIdle => {
                match with_timeout(bus_off_detect, usart.read(&mut buf)).await {
                    Ok(..) => {
                        // got a byte from the wire
                        if (Instant::now() - last_rx_time) > idle_detect {
                            // This is the first byte of a new message
                            if ((buf[0] == 0) || (buf[0] == address)) {
                                // broadcast or addressed to us?
                                length = 0;
                                message[length] = buf[0];
                                broadcast = (buf[0] == 0);
                                length += 1;
                                bus_state = ModbusState::RxData;
                                defmt::debug!("Idle->RxData");
                            }
                        } else {
                            // This is part of a message we dont care about,
                        }
                        last_rx_time = Instant::now();
                    }
                    Err(_) => {
                        defmt::debug!("No traffic timeout");
                        // TODO No bus activity seen for a few seconds we should probably switch off the flow
                    }
                }
            }
            ModbusState::RxData => {
                match with_timeout(gap_detect, usart.read(&mut buf)).await {
                    Ok(..) => {
                        if (length < 255) {
                            message[length] = buf[0];
                            length += 1;
                            defmt::debug!("RxData {}", buf[0]);
                        } else {
                            // Message longer then maximum RTU length
                            bus_state = ModbusState::WaitForIdle;
                            defmt::debug!("Too long! RxData->Idle");
                        }
                    }
                    Err(_) => {
                        // Timeout, we (might) have a complete message
                        defmt::debug!("Packet received {} bytes", length);
                        if (length > 3) { // address, two bytes of CRC
                        } else {
                            // invalid packet
                            bus_state = ModbusState::WaitForIdle;
                        }
                    }
                }
            }
            ModbusState::TxData => {
                bus_state = ModbusState::WaitForIdle;
            }
        }
    }
}
*/
