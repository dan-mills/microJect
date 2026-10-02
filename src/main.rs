#![no_std]
#![no_main]

// Note local copies of systick-timer crate because I could not get the dependencies to resolve
//mod timer;
//use timer::Timer as Coretimer;
//mod embassy_driver;
//use embassy_driver::SystickDriver;

use core::sync::atomic::Ordering::Relaxed;
//use async_std::task;
use core::sync::atomic::{self, AtomicBool, AtomicU16};
//use cortex_m::Peripherals;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, AdcChannel, Config as ADC_Config, SampleTime};
use embassy_stm32::gpio::{AnyPin, Input, Level, Output, OutputType, Pin, Pull, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::rcc::{Pll, PllMul, PllPreDiv, PllSource, Sysclk};
use embassy_stm32::time::hz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::{Channel, simple_pwm};
use embassy_stm32::usart::{Config as USART_Config, Uart};
use embassy_stm32::{Config, Peri, adc, bind_interrupts, dma, peripherals, usart};
use embassy_time::{Delay, Duration, Instant, Ticker, Timer, WithTimeout, with_timeout};
use embedded_hal::serial;
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
const DUTY_CYCLE_MAX: u16 = 1000;
const FREQUENCY_MAX: u16 = 200;
const DEFAULT_MODBUS_ADDRESS: u8 = 8;

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
    adc_pin0: Peri<'static, peripherals::PA4>, //ADC input 1
    adc_pin1: Peri<'static, peripherals::PA5>, //ADC input 2
    opto_pin0: Peri<'static, AnyPin>,          // Opto input 1
    opto_pin1: Peri<'static, AnyPin>,          // Opto input 2
    local_control: bool, // True if logal switches and pots are being used to control the output
) {
    static ADC_DMA_BUF: StaticCell<[u16; DMA_BUF_LEN]> = StaticCell::new();
    let adc_dma_buf = ADC_DMA_BUF.init([0u16; DMA_BUF_LEN]);
    let mut adc_con = ADC_Config::default();
    adc_con.resolution = Some(embassy_stm32::adc::Resolution::Bits12);
    adc_con.clock = embassy_stm32::adc::Clock::Sync(adc::SyncDiv::Div4);
    //    defmt::trace!("Configuring ADC");
    let adc = Adc::new_blocking(adc, adc_con);
    adc.sample_time_for_us(100);
    //    defmt::trace!("Configuring ADC DMA");
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
    //    defmt::trace!("Configuring Opto inputs");
    let opto0 = Input::new(opto_pin0, Pull::Up);
    let opto1 = Input::new(opto_pin1, Pull::Up);
    let mut measurements = [0u16; DMA_BUF_LEN / 2];
    //    defmt::trace!("Starting ADC and switch input loop");
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
                //warn!("Error: {:?}", e);
            }
        }
        // Read the opto isolated inputs, might as well do this here, it runs sufficiently often
        INPUT_DATA[0].store(opto0.is_low(), core::sync::atomic::Ordering::Relaxed);
        INPUT_DATA[1].store(opto1.is_low(), core::sync::atomic::Ordering::Relaxed);
        //defmt::info!("Optos {} {}", opto0.is_low(), opto1.is_low());
        if local_control {
            // Switch on the PWM based on the isolated inputs
            //COIL_DATA[0].store(opto0.is_low(), core::sync::atomic::Ordering::Relaxed);
            //COIL_DATA[1].store(opto1.is_low(), core::sync::atomic::Ordering::Relaxed);
            HREGISTER_DATA[0].store(IREGISTER_DATA[0].load(Relaxed) / 65, Relaxed); // CH0 Duty cycle
            HREGISTER_DATA[2].store(IREGISTER_DATA[0].load(Relaxed) / 65, Relaxed); // CH1 Duty cycle
            HREGISTER_DATA[1].store(IREGISTER_DATA[1].load(Relaxed) / 650, Relaxed); // CH0 Frequency
            HREGISTER_DATA[3].store(IREGISTER_DATA[1].load(Relaxed) / 650, Relaxed); // CH1 Frequency

            HREGISTER_DATA[1].store(100, Relaxed); // CH0 Frequency
            HREGISTER_DATA[0].store(200, Relaxed); // CH0 Duty cycle
            HREGISTER_DATA[2].store(20, Relaxed); // CH1 Duty cycle
            HREGISTER_DATA[3].store(20, Relaxed); // CH1 Frequency
        }
        // This chip is stupid, you cannot get the ADC to run slowly enough to avoid bogging down everything else
        // so stop it, wait a bit and then kick it off again, bit inefficient because we wake up, kick off the adc conversion and then sleep again
        // But whatever it is good enough.
        ring_buffered_adc.stop();
        ticker.next().await;
        ring_buffered_adc.start();
    }
}

#[derive(Default, Debug)]
enum ModbusState {
    #[default]
    WaitForIdle3_5, // 3.5 charcter times
    Idle,
    RxData,
}

// reads a coil or discrete input from the appropriate data array and produces the appropriate response
fn read_boolean(message: &mut [u8; 256], data: &[AtomicBool]) -> Result<usize, u8> {
    let start_address: u16 = (message[2] as u16) << 8 | message[3] as u16;
    let num_inputs: u16 = (message[4] as u16) << 8 | message[5] as u16;
    if (num_inputs > 0x7d0) {
        return Err(3);
    }
    if (start_address + num_inputs) > data.len() as u16 {
        return Err(2);
    }
    let payload_bytes = ((num_inputs + 7) / 8) as usize;
    message[2] = payload_bytes as u8;
    let mut coil_bits = 0;
    for i in 0..payload_bytes {
        message[3 + i] = 0;
    }
    let mut address: usize = 0;
    for i in 0..num_inputs as usize {
        address = 3 + i / 8;
        let bit = 1 << (i % 8);
        if (data[i as usize].load(Relaxed)) {
            coil_bits |= bit;
        }
        if (i % 8) == 7 {
            message[address] = coil_bits as u8;
            coil_bits = 0;
        }
    }
    message[address] = coil_bits as u8;
    Ok(payload_bytes + 3)
}

fn read_register(message: &mut [u8; 256], data: &[AtomicU16]) -> Result<usize, u8> {
    let start_address: u16 = (message[2] as u16) << 8 | message[3] as u16;
    let num_regs: u16 = (message[4] as u16) << 8 | message[5] as u16;
    defmt::trace!("Start address: {}", start_address);
    defmt::trace!("Number of registers: {}", num_regs);
    if (num_regs > 0x7d) {
        defmt::debug!("Number of registers too large");
        return Err(0x03);
    }
    if (start_address >= data.len() as u16) || (start_address + num_regs > data.len() as u16) {
        defmt::debug!("Address out of range");
        return Err(0x02);
    }
    // Build the response
    let payload_bytes = num_regs as usize * 2;
    message[2] = payload_bytes as u8;
    for i in 0..num_regs {
        let value = data[start_address as usize + i as usize].load(Relaxed);
        message[3 + i as usize * 2] = (value >> 8) as u8;
        message[3 + i as usize * 2 + 1] = value as u8;
    }
    Ok(payload_bytes + 3)
}

fn write_coil(message: &mut [u8; 256], data: &[AtomicBool]) -> Result<usize, u8> {
    let coil = (message[2] as u16) << 8 | message[3] as u16;
    let value = (message[4] as u16) << 8 | message[5] as u16;
    if (value != 0xff00) && (value != 0x0000) {
        return Err(0x03);
    }
    if coil >= data.len() as u16 {
        return Err(0x02);
    }
    defmt::debug!("Write coil {} value {}", coil, value == 0xff00);
    data[coil as usize].store(value == 0xff00, Relaxed);
    Ok(6)
}

#[embassy_executor::task]
async fn modbus_task(address: u8, baudrate: u32, mut usart1: Uart<'static, Async>) {
    //async fn modbus_task(address: u8, baudrate: u32, p: embassy_stm32::Peripherals) {
    // 11 bits in a serial byte (start + 8 data + parity + stop)
    // 3.5 character times at the given baudrate
    let mut timeout3_5 = 11 * 3_500_000 / baudrate;
    let mut timeout1_5 = 11 * 1_500_000 / baudrate;
    let timeout_idle = 10_000_000;
    if (baudrate > 19200) {
        // Per current spec, 3.5/1.5 char times are fixed for high baudrates
        timeout3_5 = 1750; // microseconds
        timeout1_5 = 750;
    }
    let mut rx_buffer = [0u8; 1];
    let mut state = ModbusState::WaitForIdle3_5;
    let mut bytes_received = 0;
    let mut rx_data = [0u8; 256]; // Max length per modbus spec
    let mut serial_timeout = timeout3_5;
    let mut broadcast = false;
    let mut mute = false;
    loop {
        let status = usart1
            .read(&mut rx_buffer)
            .with_timeout(Duration::from_micros(serial_timeout as u64))
            .await;
        match status {
            Ok(_) => {
                defmt::trace!("Received byte: {:?}", rx_buffer[0]);
            }
            Err(_) => {}
        }
        match state {
            ModbusState::WaitForIdle3_5 => {
                serial_timeout = timeout3_5;
                broadcast = false;
                match status {
                    Ok(_) => {
                        defmt::debug!("Rejected character: {:?}", rx_buffer[0]);
                    }
                    Err(e) => {
                        if e == embassy_time::TimeoutError {
                            state = ModbusState::Idle;
                            bytes_received = 0;
                            serial_timeout = timeout_idle; // Deliberately long timeout to wait in idle
                            defmt::trace!("Modbus state changed to Idle");
                        } else {
                            defmt::debug!("read error: {:?}", e);
                            bytes_received = 0;
                        }
                    }
                }
                continue;
            }
            ModbusState::Idle => {
                bytes_received = 0;
                match status {
                    Ok(_) => {
                        rx_data[0] = rx_buffer[0];
                        bytes_received += 1;
                        if ((rx_buffer[0] == address) || (rx_buffer[0] == 0)) {
                            // it is addressed to us
                            broadcast = false;
                            defmt::trace!("Address match {}", rx_buffer[0]);
                            if (rx_buffer[0] == 0) {
                                // broadcast
                                broadcast = true;
                                defmt::trace!("Broadcast");
                            }
                            defmt::trace!("Modbus state changed to RxData");
                            serial_timeout = timeout1_5;
                            state = ModbusState::RxData;
                        } else {
                            defmt::trace!("Not addressed to us");
                            serial_timeout = timeout3_5;
                            state = ModbusState::WaitForIdle3_5;
                        }
                    }
                    Err(e) => {
                        if e != embassy_time::TimeoutError {
                            // Any error except a timeout puts us back into the wait for gap state
                            defmt::debug!("read error: {:?}", e);
                            serial_timeout = timeout3_5;
                            state = ModbusState::WaitForIdle3_5;
                        }
                    }
                }
            }
            ModbusState::RxData => {
                match status {
                    Ok(status) => {
                        rx_data[bytes_received] = rx_buffer[0];
                        bytes_received += 1;
                        if bytes_received == rx_data.len() {
                            defmt::debug!("Receive length overrun");
                            serial_timeout = timeout3_5;
                            bytes_received = 0;
                            state = ModbusState::WaitForIdle3_5;
                        }
                    }
                    Err(e) => {
                        if e == embassy_time::TimeoutError {
                            // We got a possibly complete frame
                            defmt::debug!("Message received '{}'", rx_data[0..bytes_received]);
                            if bytes_received > 3 {
                                // address, function code, CRC, CRC
                                // See if the CRC is good
                                let crc = crc16(&rx_data[0..bytes_received - 2]);
                                if (crc.1 == rx_data[bytes_received - 2])
                                    && (crc.0 == rx_data[bytes_received - 1])
                                {
                                    defmt::trace!("CRC OK");
                                    let opcode = rx_data[1];
                                    let mut resultcode = 0x00; // set non zero if we are going to return an exception
                                    let mut payload_bytes: usize = 0; // Payload length in bytes not including checksu
                                    match opcode {
                                        0x01 => {
                                            defmt::trace!("Read Discrete Coils");
                                            match read_boolean(&mut rx_data, &COIL_DATA) {
                                                Ok(bytes) => payload_bytes = bytes,
                                                Err(code) => resultcode = code,
                                            }
                                        }

                                        0x02 => {
                                            defmt::trace!("Read Discrete Inputs");
                                            match read_boolean(&mut rx_data, &INPUT_DATA) {
                                                Ok(bytes) => payload_bytes = bytes,
                                                Err(code) => resultcode = code,
                                            }
                                        }
                                        0x03 => {
                                            defmt::trace!("Read Holding Regs");
                                            match read_register(&mut rx_data, &HREGISTER_DATA) {
                                                Ok(bytes) => payload_bytes = bytes,
                                                Err(code) => resultcode = code,
                                            }
                                        }
                                        0x04 => {
                                            defmt::trace!("Read Input Regs");
                                            match read_register(&mut rx_data, &IREGISTER_DATA) {
                                                Ok(bytes) => payload_bytes = bytes,
                                                Err(code) => resultcode = code,
                                            }
                                        }
                                        0x05 => {
                                            defmt::trace!("Write Single coil");
                                            match write_coil(&mut rx_data, &COIL_DATA) {
                                                Ok(bytes) => payload_bytes = bytes,
                                                Err(code) => resultcode = code,
                                            }
                                        }

                                        _ => {
                                            // Unknown command
                                            resultcode = 0x01
                                        }
                                    }
                                    if (resultcode != 0) {
                                        // send the exception response
                                        defmt::trace!("Sending exception response {}", resultcode);
                                        rx_data[1] |= 0x80; // Mark this as an exception
                                        rx_data[2] = resultcode;
                                        payload_bytes = 3;
                                    }
                                    let crc = crc16(&rx_data[..payload_bytes as usize]);
                                    rx_data[payload_bytes] = crc.1;
                                    rx_data[payload_bytes + 1] = crc.0;
                                    defmt::trace!("Sending {}", rx_data[..payload_bytes + 2]);
                                    if (!broadcast) && (!mute) {
                                        usart1.write(&rx_data[..payload_bytes + 2]).await.ok();
                                    }
                                } else {
                                    defmt::debug!("CRC fail");
                                    defmt::trace!("Modbus state changed to Wait for Idle3_5");
                                }
                            }
                            bytes_received = 0;
                            serial_timeout = timeout3_5;
                            state = ModbusState::WaitForIdle3_5;
                        } else {
                            defmt::debug!("read error: {:?}", e);
                            serial_timeout = timeout3_5;
                            state = ModbusState::WaitForIdle3_5;
                            defmt::trace!("Modbus state changed to Idle3_5");
                        }
                    }
                }
            }
        }
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

    let led1 = Output::new(p.PB3, Level::High, Speed::Low);
    let led2 = Output::new(p.PB4, Level::High, Speed::Low);
    let led3 = Output::new(p.PB5, Level::High, Speed::Low);

    let ch1_pin = PwmPin::new(p.PA7, OutputType::PushPull);
    let mut pwm1 = SimplePwm::new(
        p.TIM14,
        Some(ch1_pin),
        None,
        None,
        None,
        hz(20),
        Default::default(),
    );

    let ch2_pin = PwmPin::new(p.PA6, OutputType::PushPull);
    let mut pwm2 = SimplePwm::new(
        p.TIM16,
        Some(ch2_pin),
        None,
        None,
        None,
        hz(50),
        Default::default(),
    );

    //ch2.set_duty_cycle_percent(10);

    let ch3_pin = PwmPin::new(p.PA8, OutputType::PushPull);
    let mut pwm3 = SimplePwm::new(
        p.TIM1,
        Some(ch3_pin),
        None,
        None,
        None,
        hz(5),
        Default::default(),
    );
    //let mut ch3 = pwm3.ch1();
    //ch3.disable();
    //ch3.set_duty_cycle_percent(50);

    // TODO FIXME, make the core timer use the systick and not TIM3.
    //let ch4_pin = PwmPin::new(p.PB0, OutputType::PushPull);
    //let mut pwm4 = SimplePwm::new(p.TIM3, None, None, Some(ch4_pin), None, Hz(10), Default::default());
    //let mut ch4 = pwm4.ch3();
    //ch4.enable();
    //pwm4.set_frequency(Hz(100));
    //  ch4.set_duty_cycle_percent(5);
    //
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
    spawner.spawn(modbus_task(modbus_address, baud, usart).unwrap());

    let mut old_dutycycles = [0u16; 3];
    loop {
        // this does the really annoying bit of business updating the PWM channels based on the values in the modbus registers
        // Done here because passing the timers around turns out to be a real pain.
        // Yes this is a dogs dinner TODO FIXME.
        // Set the frequency if that has changed
        let mut duty_cycle1 = HREGISTER_DATA[0].load(core::sync::atomic::Ordering::Relaxed);
        let mut frequency1 = HREGISTER_DATA[1].load(core::sync::atomic::Ordering::Relaxed);
        if (duty_cycle1 > DUTY_CYCLE_MAX) {
            duty_cycle1 = DUTY_CYCLE_MAX;
        }
        if frequency1 < 1 {
            frequency1 = 1;
        }
        if frequency1 > FREQUENCY_MAX {
            frequency1 = FREQUENCY_MAX;
        }
        if hz(frequency1 as u32) != pwm1.get_frequency() {
            pwm1.set_frequency(hz(frequency1 as u32));
            pwm1.ch1()
                .set_duty_cycle_fraction(duty_cycle1 as u32, DUTY_CYCLE_MAX as u32);
            old_dutycycles[0] = duty_cycle1;
        }
        if duty_cycle1 != old_dutycycles[0] {
            pwm1.ch1()
                .set_duty_cycle_fraction(duty_cycle1 as u32, DUTY_CYCLE_MAX as u32);
            old_dutycycles[0] = duty_cycle1
        }
        // Switch on or off based on the coil state
        if COIL_DATA[0].load(core::sync::atomic::Ordering::Relaxed) {
            if !pwm1.ch1().is_enabled() {
                pwm1.ch1().enable();
            }
        } else {
            if pwm1.ch1().is_enabled() {
                pwm1.ch1().disable();
            }
        }

        let mut duty_cycle2 = HREGISTER_DATA[2].load(core::sync::atomic::Ordering::Relaxed);
        let mut frequency2 = HREGISTER_DATA[3].load(core::sync::atomic::Ordering::Relaxed);
        if frequency2 < 1 {
            frequency2 = 1;
        }
        if frequency2 > FREQUENCY_MAX {
            frequency2 = FREQUENCY_MAX;
        }
        if (duty_cycle2 > DUTY_CYCLE_MAX) {
            duty_cycle2 = DUTY_CYCLE_MAX;
        }
        if hz(frequency2 as u32) != pwm2.get_frequency() {
            pwm2.set_frequency(hz(frequency2 as u32));
            pwm2.ch1()
                .set_duty_cycle_fraction(duty_cycle2 as u32, DUTY_CYCLE_MAX as u32);
            old_dutycycles[1] = duty_cycle2;
        }
        if duty_cycle2 != old_dutycycles[1] {
            pwm2.ch1()
                .set_duty_cycle_fraction(duty_cycle2 as u32, DUTY_CYCLE_MAX as u32);
            old_dutycycles[1] = duty_cycle2;
        }
        // Switch on or off based on the coil state
        if COIL_DATA[1].load(core::sync::atomic::Ordering::Relaxed) {
            if !pwm2.ch1().is_enabled() {
                pwm2.ch1().enable();
            }
        } else {
            if pwm2.ch1().is_enabled() {
                pwm2.ch1().disable();
            }
        }
        let mut duty_cycle3 = HREGISTER_DATA[4].load(core::sync::atomic::Ordering::Relaxed);
        let mut frequency3 = HREGISTER_DATA[5].load(core::sync::atomic::Ordering::Relaxed);
        if frequency3 < 1 {
            frequency3 = 1;
        }
        if frequency3 > FREQUENCY_MAX {
            frequency3 = FREQUENCY_MAX;
        }
        if (duty_cycle3 > DUTY_CYCLE_MAX) {
            duty_cycle3 = DUTY_CYCLE_MAX;
        }
        if hz(frequency3 as u32) != pwm3.get_frequency() {
            pwm3.set_frequency(hz(frequency3 as u32));
            pwm3.ch1()
                .set_duty_cycle_fraction(duty_cycle3 as u32, DUTY_CYCLE_MAX as u32);
            old_dutycycles[2] = duty_cycle3;
        }
        if duty_cycle3 != old_dutycycles[2] {
            pwm3.ch1()
                .set_duty_cycle_fraction(duty_cycle3 as u32, DUTY_CYCLE_MAX as u32);
            old_dutycycles[2] = duty_cycle3;
        }
        // Switch on or off based on the coil state
        if COIL_DATA[2].load(core::sync::atomic::Ordering::Relaxed) {
            if !pwm3.ch1().is_enabled() {
                pwm3.ch1().enable();
            }
        } else {
            if pwm3.ch1().is_enabled() {
                pwm3.ch1().disable();
            }
        }
        Timer::after(Duration::from_millis(50)).await;
    }
}

const CRC_UPPER: [u8; 256] = [
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
];

const CRC_LOWER: [u8; 256] = [
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
    0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
    0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
    0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
    0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
    0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
    0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40,
];

fn crc16(d: &[u8]) -> (u8, u8) {
    let mut hi: u8 = 0xFF;
    let mut low: u8 = 0xFF;
    for ch in d {
        let index = low ^ *ch;
        low = hi ^ CRC_UPPER[index as usize];
        hi = CRC_LOWER[index as usize];
    }
    (hi, low)
}
