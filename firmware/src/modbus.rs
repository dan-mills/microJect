use core::sync::atomic::Ordering::Relaxed;
use core::sync::atomic::{AtomicBool, AtomicU16};
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::Uart;
use embassy_time::{Duration, Instant, WithTimeout};

use crate::COIL_DATA;
use crate::HREGISTER_DATA;
use crate::INPUT_DATA;
use crate::IREGISTER_DATA;

use crate::SERVER_ID;

const UART_BUF_LEN: usize = 64;

#[derive(Default, Debug)]
enum ModbusState {
    #[default]
    WaitForIdle3_5, // 3.5 charcter times
    Idle,
    RxData,
}

// reads a coil or discrete input from the appropriate data array and produces the appropriate response
fn read_boolean(message: &mut [u8; 256], data: &[AtomicBool]) -> Result<usize, u8> {
    defmt::trace!("Read Discrete Input or coil");
    let start_address: u16 = (message[2] as u16) << 8 | message[3] as u16;
    let num_inputs: u16 = (message[4] as u16) << 8 | message[5] as u16;
    if num_inputs > 0x7d0 {
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
        if data[i as usize].load(Relaxed) {
            coil_bits |= bit;
        }
        if (i % 8) == 7 {
            message[address] = coil_bits as u8;
            coil_bits = 0;
        }
    }
    message[address] = coil_bits as u8;
    Ok(payload_bytes + 3)
    // Note Payload bytes is everything up to the CRC, not including the CRC itself.
    // This includes the address and opcode bytes on the start of the message.
}

fn read_register(message: &mut [u8; 256], data: &[AtomicU16]) -> Result<usize, u8> {
    defmt::trace!("Read Register");
    let start_address: u16 = (message[2] as u16) << 8 | message[3] as u16;
    let num_regs: u16 = (message[4] as u16) << 8 | message[5] as u16;
    defmt::trace!("Start address: {}", start_address);
    defmt::trace!("Number of registers: {}", num_regs);
    if num_regs > 0x7d {
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
    defmt::trace!("Write Single coil");
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

fn write_coils(message: &mut [u8; 256], data: &[AtomicBool]) -> Result<usize, u8> {
    defmt::trace!("Write Multiple coils");
    let coil = (message[2] as u16) << 8 | message[3] as u16;
    let count = (message[4] as u16) << 8 | message[5] as u16;
    let bytes = message[6];
    if (count > 0x7b0) || (bytes as u16 != (count + 7) / 8) {
        return Err(0x03);
    }
    if (coil + count) > data.len() as u16 {
        return Err(0x02);
    }
    for i in 0..count {
        data[(coil + i) as usize]
            .store(message[(7 + i / 8) as usize] & (1 << (i % 8)) != 0, Relaxed);
    }
    Ok(6)
}

fn write_register(message: &mut [u8; 256], data: &[AtomicU16]) -> Result<usize, u8> {
    defmt::trace!("Write Single register");
    let register = (message[2] as u16) << 8 | message[3] as u16;
    let value = (message[4] as u16) << 8 | message[5] as u16;
    if register >= data.len() as u16 {
        return Err(0x02);
    }
    data[register as usize].store(value, Relaxed);
    Ok(6)
}

fn write_registers(message: &mut [u8; 256], data: &[AtomicU16]) -> Result<usize, u8> {
    defmt::trace!("Write Multiple registers");
    let register = (message[2] as u16) << 8 | message[3] as u16;
    let count = (message[4] as u16) << 8 | message[5] as u16;
    let bytes = message[6];
    if (count > 0x7b) || (bytes as u16 != 2 * count) {
        return Err(0x03);
    }
    if register + count > data.len() as u16 {
        return Err(0x02);
    }
    for i in 0..count {
        data[(register + i) as usize].store(
            (message[(7 + 2 * i) as usize] as u16) << 8 | message[(8 + 2 * i) as usize] as u16,
            Relaxed,
        );
    }
    Ok(6)
}

fn report_server_id(message: &mut [u8; 256], server_id: &[u8]) -> Result<usize, u8> {
    defmt::trace!("Report server ID");
    message[2] = server_id.len() as u8 + 1;
    message[3] = 0x01; // Run
    //message[4] = server_id.len() as u8;
    for (i, &byte) in server_id.iter().enumerate() {
        message[4 + i] = byte;
    }
    Ok(server_id.len() + 4)
}

fn read_pdu(message: &mut [u8; 256]) -> Result<usize, u8> {
    defmt::trace!("Read PDU");
    let mei = message[2];
    let read_device_id = message[3];
    let mut oid = message[4];
    if mei != 0x0E {
        return Err(0x01);
    }
    if read_device_id == 0 || read_device_id > 2 {
        // We support conformity level 2 only
        return Err(0x03);
    }
    let texts: [&[u8]; 5] = [
        b"MiniMonster",
        b"MJ4",
        b"V 1.0.0",
        b"HTTPS://www.github.com",
        b"microJect",
    ];
    if oid >= texts.len() as u8 {
        // We only have 5 strings to report
        return Err(0x04);
    }

    message[5] = 0x02; // Conformance level
    message[6] = 0x00; // More follows
    message[7] = 0x00; // Next object ID
    message[8] = texts.len() as u8 - oid;
    let mut location = 9; // start of the list
    loop {
        if oid > 0x04 {
            break;
        }
        message[location] = oid;
        let text = texts[oid as usize];
        message[location + 1] = text.len() as u8;
        location += 2;
        for (i, &byte) in text.iter().enumerate() {
            message[location + i] = byte;
        }
        location += text.len();
        oid += 1;
    }
    return Ok(location);
}

#[embassy_executor::task]
pub async fn modbus_task(
    address: u8,
    baudrate: u32,
    manual_control: bool,
    uart: Uart<'static, Async>,
    mut modbus_data_led: Output<'static>,
    mut modbus_err_led: Output<'static>,
) {
    // 11 bits in a serial byte (start + 8 data + parity + stop)
    // 3.5 character times at the given baudrate
    let mut timeout3_5 = 11 * 3_500_000 / baudrate;
    let mut timeout1_5 = 11 * 1_500_000 / baudrate;
    let timeout_idle = Duration::from_secs(5);
    if baudrate > 19200 {
        // Per current spec, 3.5/1.5 char times are fixed for high baudrates
        timeout3_5 = 1750; // microseconds
        timeout1_5 = 750;
    }
    // split the UART buffer into tx and rx halves so we can have a DMA ring buffer on the rx side
    // stm32 uarts are pathetic.
    let mut dma_buffer = [0u8; UART_BUF_LEN];
    let (mut uart_tx, uart_rx) = uart.split();
    let mut rx_ringbuffered = uart_rx.into_ring_buffered(&mut dma_buffer);

    let mut rx_buffer = [0u8; 1];
    let mut state = ModbusState::WaitForIdle3_5;
    let mut bytes_received = 0;
    let mut serial_data = [0u8; 256]; // Max length per modbus spec, used for both input and output
    let mut serial_timeout = timeout3_5;
    let mut broadcast = false;
    let mut mute = false;

    let mut last_message = Instant::now();

    loop {
        let status = rx_ringbuffered
            .read(&mut rx_buffer)
            .with_timeout(Duration::from_micros(serial_timeout as u64))
            .await;
        match status {
            Ok(_) => {
                defmt::trace!("Received byte: {:?}", rx_buffer[0]);
            }
            Err(_) => {}
        }
        if Instant::now() - last_message > timeout_idle {
            modbus_data_led.set_low();
            if !manual_control {
                // Reset coil data if no message received for idle timeout thus stopping pulse generation
                COIL_DATA[0].store(false, Relaxed);
                COIL_DATA[1].store(false, Relaxed);
                COIL_DATA[2].store(false, Relaxed);
                COIL_DATA[3].store(false, Relaxed);
            }
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
                            serial_timeout = timeout_idle.as_micros() as u32; // Deliberately long timeout to wait in idle
                            defmt::trace!("Modbus state changed to Idle");
                        } else {
                            defmt::debug!("read error: {:?}", e);
                            modbus_err_led.set_high();
                            bytes_received = 0;
                        }
                    }
                }
            }
            ModbusState::Idle => {
                bytes_received = 0;
                match status {
                    Ok(_) => {
                        serial_data[0] = rx_buffer[0];
                        bytes_received += 1;
                        if (rx_buffer[0] == address) || (rx_buffer[0] == 0) {
                            // it is addressed to us
                            broadcast = false;
                            defmt::trace!("Address match {}", rx_buffer[0]);
                            if rx_buffer[0] == 0 {
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
                            modbus_err_led.set_high();
                            serial_timeout = timeout3_5;
                            state = ModbusState::WaitForIdle3_5;
                        }
                    }
                }
            }
            ModbusState::RxData => {
                match status {
                    Ok(_) => {
                        serial_data[bytes_received] = rx_buffer[0];
                        bytes_received += 1;
                        if bytes_received == serial_data.len() {
                            defmt::debug!("Receive length overrun");
                            modbus_err_led.set_high();
                            break;
                        }
                    }
                    Err(e) => {
                        if e == embassy_time::TimeoutError {
                            // We got a possibly complete frame
                            defmt::debug!("Message received '{}'", serial_data[0..bytes_received]);
                            last_message = Instant::now(); // Deal with the timeout for loss of comms
                            if bytes_received > 3 {
                                // address, function code, CRC, CRC
                                // See if the CRC is good
                                let crc = crc16(&serial_data[0..bytes_received - 2]);
                                if (crc.1 == serial_data[bytes_received - 2])
                                    && (crc.0 == serial_data[bytes_received - 1])
                                {
                                    defmt::trace!("CRC OK");
                                    modbus_data_led.toggle();
                                    let opcode = serial_data[1];
                                    let resultcode: Result<usize, u8>;
                                    match opcode {
                                        0x01 => {
                                            resultcode = read_boolean(&mut serial_data, &COIL_DATA)
                                        }

                                        0x02 => {
                                            resultcode = read_boolean(&mut serial_data, &INPUT_DATA)
                                        }

                                        0x03 => {
                                            resultcode =
                                                read_register(&mut serial_data, &HREGISTER_DATA)
                                        }

                                        0x04 => {
                                            resultcode =
                                                read_register(&mut serial_data, &IREGISTER_DATA)
                                        }

                                        0x05 => {
                                            resultcode = write_coil(&mut serial_data, &COIL_DATA)
                                        }

                                        0x06 => {
                                            resultcode =
                                                write_register(&mut serial_data, &HREGISTER_DATA)
                                        }

                                        0x0f => {
                                            resultcode = write_coils(&mut serial_data, &COIL_DATA)
                                        }

                                        0x10 => {
                                            resultcode =
                                                write_registers(&mut serial_data, &HREGISTER_DATA)
                                        }

                                        0x11 => {
                                            resultcode =
                                                report_server_id(&mut serial_data, &SERVER_ID)
                                        }
                                        0x2b => resultcode = read_pdu(&mut serial_data),

                                        _ => {
                                            // Unknown command
                                            resultcode = Err(0x01)
                                        }
                                    }
                                    let response_bytes: usize; // Response length in bytes not including checksum, but including the address and opcode
                                    match resultcode {
                                        Ok(length) => {
                                            response_bytes = length;
                                            modbus_err_led.set_low();
                                        }
                                        Err(code) => {
                                            // send the exception response
                                            defmt::trace!("Sending exception response {}", code);
                                            serial_data[1] |= 0x80; // Mark this as an exception
                                            serial_data[2] = code;
                                            response_bytes = 3;
                                            modbus_err_led.set_high();
                                        }
                                    }
                                    let crc = crc16(&serial_data[..response_bytes as usize]);
                                    serial_data[response_bytes] = crc.1;
                                    serial_data[response_bytes + 1] = crc.0;
                                    defmt::trace!("Sending {}", serial_data[..response_bytes + 2]);
                                    if (!broadcast) && (!mute) {
                                        uart_tx
                                            .write(&serial_data[..response_bytes + 2])
                                            .await
                                            .ok();
                                    }
                                } else {
                                    defmt::debug!("CRC fail");
                                    modbus_err_led.set_high();
                                    defmt::trace!("Modbus state changed to Wait for Idle3_5");
                                }
                            }
                            bytes_received = 0;
                            serial_timeout = timeout3_5;
                            state = ModbusState::WaitForIdle3_5;
                        } else {
                            defmt::debug!("read error: {:?}", e);
                            modbus_err_led.set_high();
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
