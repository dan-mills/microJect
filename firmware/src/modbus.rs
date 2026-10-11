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
const TIMEOUT_IDLE: Duration = Duration::from_secs(5);

#[derive(Default, Debug)]
enum ModbusState {
    #[default]
    WaitForIdle3_5, // 3.5 charcter times
    Idle,
    RxData,
}

#[derive(Debug)]
struct modbus_data {
    bytes_received: usize,
    serial_data: [u8; 256], // Max length per modbus spec, used for both input and output
    mute: bool,             // Are we in mute mode
    broadcast: bool,
    exception_code: u16,
    bus_message_count: u16,
    bus_comms_error_count: u16,
    bus_exception_count: u16,
    bus_server_message_count: u16,
    server_no_response_count: u16,
    event_count: u16,
    event_log: [u8; 64],
    event_log_index: u8,
}

impl Default for modbus_data {
    fn default() -> Self {
        modbus_data {
            bytes_received: 0,
            serial_data: [0u8; 256],
            mute: false,
            broadcast: false,
            exception_code: 0,
            bus_message_count: 0,
            bus_comms_error_count: 0,
            bus_exception_count: 0,
            bus_server_message_count: 0,
            server_no_response_count: 0,
            event_count: 0,
            event_log: [0u8; 64],
            event_log_index: 0,
        }
    }
}
impl modbus_data {
    // reads a coil or discrete input from the appropriate data array and produces the appropriate response
    fn read_boolean(&mut self, data: &[AtomicBool]) -> Result<usize, u8> {
        defmt::trace!("Read Discrete Input or coil");
        if self.mute {
            return Err(0); // Not really an error, but we shouldnt actually do anything
        }
        let payload = &mut self.serial_data[2..];
        let start_address: u16 = (payload[0] as u16) << 8 | payload[1] as u16;
        let num_inputs: u16 = (payload[2] as u16) << 8 | payload[3] as u16;
        if num_inputs > 0x7d0 {
            return Err(3);
        }
        if (start_address + num_inputs) > data.len() as u16 {
            return Err(2);
        }
        let payload_bytes = ((num_inputs + 7) / 8) as usize;
        payload[0] = payload_bytes as u8;
        let mut coil_bits = 0;
        for i in 0..payload_bytes {
            payload[1 + i] = 0;
        }
        let mut address: usize = 0;
        for i in 0..num_inputs as usize {
            address = 1 + i / 8;
            let bit = 1 << (i % 8);
            if data[i as usize].load(Relaxed) {
                coil_bits |= bit;
            }
            if (i % 8) == 7 {
                payload[address] = coil_bits as u8;
                coil_bits = 0;
            }
        }
        payload[address] = coil_bits as u8;
        Ok(payload_bytes + 1) // +1 for the length byte
    }

    fn read_register(&mut self, data: &[AtomicU16]) -> Result<usize, u8> {
        defmt::trace!("Read Register");
        if self.mute {
            return Err(0); // Not really an error, but we shouldnt actually do anything
        }
        let payload = &mut self.serial_data[2..];
        let start_address: u16 = (payload[0] as u16) << 8 | payload[1] as u16;
        let num_regs: u16 = (payload[2] as u16) << 8 | payload[3] as u16;
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
        payload[0] = payload_bytes as u8;
        for i in 0..num_regs {
            let value = data[start_address as usize + i as usize].load(Relaxed);
            payload[1 + i as usize * 2] = (value >> 8) as u8;
            payload[1 + i as usize * 2 + 1] = value as u8;
        }
        Ok(payload_bytes + 1) // +1 for the length byte
    }

    fn write_coil(&mut self, data: &[AtomicBool]) -> Result<usize, u8> {
        defmt::trace!("Write Single coil");
        if self.mute {
            return Err(0); // Not really an error, but we shouldnt actually do anything
        }
        let payload = &mut self.serial_data[2..];
        let coil = (payload[0] as u16) << 8 | payload[1] as u16;
        let value = (payload[2] as u16) << 8 | payload[3] as u16;
        if (value != 0xff00) && (value != 0x0000) {
            return Err(0x03);
        }
        if coil >= data.len() as u16 {
            return Err(0x02);
        }
        defmt::debug!("Write coil {} value {}", coil, value == 0xff00);
        data[coil as usize].store(value == 0xff00, Relaxed);
        Ok(4) // coil and value
    }

    fn write_coils(&mut self, data: &[AtomicBool]) -> Result<usize, u8> {
        defmt::trace!("Write Multiple coils");
        if self.mute {
            return Err(0); // Not really an error, but we shouldnt actually do anything
        }
        let payload = &mut self.serial_data[2..];
        let coil = (payload[0] as u16) << 8 | payload[1] as u16;
        let count = (payload[2] as u16) << 8 | payload[3] as u16;
        let bytes = payload[4];
        if (count > 0x7b0) || (bytes as u16 != (count + 7) / 8) {
            return Err(0x03);
        }
        if (coil + count) > data.len() as u16 {
            return Err(0x02);
        }
        for i in 0..count {
            data[(coil + i) as usize]
                .store(payload[(5 + i / 8) as usize] & (1 << (i % 8)) != 0, Relaxed);
        }
        Ok(4) // just coil and count
    }

    fn write_register(&mut self, data: &[AtomicU16]) -> Result<usize, u8> {
        defmt::trace!("Write Single register");
        if self.mute {
            return Err(0); // Not really an error, but we shouldnt actually do anything
        }
        let payload = &mut self.serial_data[2..];
        let register = (payload[0] as u16) << 8 | payload[1] as u16;
        let value = (payload[2] as u16) << 8 | payload[3] as u16;
        if register >= data.len() as u16 {
            return Err(0x02);
        }
        data[register as usize].store(value, Relaxed);
        Ok(4)
    }

    fn write_registers(&mut self, data: &[AtomicU16]) -> Result<usize, u8> {
        defmt::trace!("Write Multiple registers");
        if self.mute {
            return Err(0); // Not really an error, but we shouldnt actually do anything
        }
        let payload = &mut self.serial_data[2..];
        let register = (payload[0] as u16) << 8 | payload[1] as u16;
        let count = (payload[2] as u16) << 8 | payload[3] as u16;
        let bytes = payload[4];
        if (count > 0x7b) || (bytes as u16 != 2 * count) {
            return Err(0x03);
        }
        if register + count > data.len() as u16 {
            return Err(0x02);
        }
        for i in 0..count {
            let value =
                (payload[(5 + 2 * i) as usize] as u16) << 8 | payload[(6 + 2 * i) as usize] as u16;
            data[(register + i) as usize].store(value, Relaxed);
        }
        Ok(4)
    }

    fn report_server_id(&mut self, server_id: &[u8]) -> Result<usize, u8> {
        defmt::trace!("Report server ID");
        if self.mute {
            return Err(0);
        }
        let payload = &mut self.serial_data[2..];
        payload[0] = server_id.len() as u8 + 1;
        for (i, &byte) in server_id.iter().enumerate() {
            payload[1 + i] = byte;
        }
        payload[server_id.len() + 2] = 0xFF; // Run
        Ok(server_id.len() + 3)
    }

    fn read_exception_register(&mut self) -> Result<usize, u8> {
        // We dont really have exceptions in this thing, so we just report coils and inputs.
        defmt::trace!("Read exception register");
        if self.mute {
            return Err(0);
        }
        let payload = &mut self.serial_data[2..];
        let mut data = 0u8;
        if COIL_DATA[0].load(Relaxed) {
            data |= 0x01
        };
        if COIL_DATA[1].load(Relaxed) {
            data |= 0x02
        };
        if COIL_DATA[2].load(Relaxed) {
            data |= 0x04
        };
        if COIL_DATA[3].load(Relaxed) {
            data |= 0x08
        };
        if INPUT_DATA[0].load(Relaxed) {
            data |= 0x10
        };
        if INPUT_DATA[0].load(Relaxed) {
            data |= 0x20
        };
        payload[0] = data;
        return Ok(1);
    }

    fn read_pdu(&mut self) -> Result<usize, u8> {
        defmt::trace!("Read PDU");
        if self.mute {
            return Err(0);
        }
        let payload = &mut self.serial_data[2..];
        let mei = payload[0];
        let read_device_id = payload[1];
        let mut oid = payload[2];
        if mei != 0x0E {
            return Err(0x01);
        }
        if read_device_id == 0 || read_device_id > 2 {
            // We support conformity level 2 only
            return Err(0x03);
        }
        let texts: [&[u8]; 5] = [
            b"Lemontart open source",
            b"MJ4",
            b"V 1.0.0",
            b"HTTPS://www.github.com",
            b"microJect",
        ];
        if oid >= texts.len() as u8 {
            // We only have 5 strings to report
            return Err(0x04);
        }

        payload[3] = 0x02; // Conformance level
        payload[4] = 0x00; // More follows
        payload[5] = 0x00; // Next object ID
        payload[6] = texts.len() as u8 - oid;
        let mut location = 7; // start of the list
        loop {
            if oid >= texts.len() as u8 {
                break;
            }
            payload[location] = oid;
            let text = texts[oid as usize];
            payload[location + 1] = text.len() as u8;
            location += 2;
            for (i, &byte) in text.iter().enumerate() {
                payload[location + i] = byte;
            }
            location += text.len();
            oid += 1;
        }
        return Ok(location);
    }

    fn read_diagnostics(&mut self) -> Result<usize, u8> {
        defmt::trace!("Read diagnostics");
        let payload = &mut self.serial_data[2..];
        let sub_function = (payload[0] as u16) << 8 | payload[1] as u16;
        let data = (payload[2] as u16) << 8 | payload[3] as u16;
        if (sub_function == 0x01) {
            // Special case because restart comms is a funny
            self.bus_message_count = 0;
            self.bus_comms_error_count = 0;
            self.bus_exception_count = 0;
            self.bus_server_message_count = 0;
            self.server_no_response_count = 0;
            self.exception_code = 0;
            self.event_count = 0;
            if self.mute {
                self.mute = false;
                return Err(0); // Clear the mute flag, but dont send a response
            } else {
                if data == 0xff00 {
                    // clear the event log
                    self.event_log_index = 0;
                }
            }
            return Ok(self.bytes_received - 4);
        }
        if self.mute {
            return Err(0);
        }
        // If we get here we are not muted and can handle this as a normal set of sub commands
        match sub_function {
            0x0000 => {
                // Echo everything back
                Ok(self.bytes_received - 4)
            }
            0x0004 => {
                // Force listen only mode
                self.mute = true;
                Err(0)
            }
            0x000A => {
                // Clear counters and diags register
                self.bus_server_message_count;
                self.bus_comms_error_count = 0;
                self.bus_exception_count = 0;
                self.bus_message_count = 0;
                self.event_count = 0;
                self.exception_code = 0;
                Ok(self.bytes_received - 4)
            }
            0x000B => {
                //Bus message count
                let value = self.bus_message_count;
                payload[2] = (value >> 8) as u8;
                payload[3] = value as u8;
                Ok(4)
            }
            0x000C => {
                // Bus communications error count
                let value = self.bus_comms_error_count;
                payload[2] = (value >> 8) as u8;
                payload[3] = value as u8;
                Ok(4)
            }
            0x000D => {
                // Bus exception count
                let value = self.bus_exception_count;
                payload[2] = (value >> 8) as u8;
                payload[3] = value as u8;
                Ok(4)
            }
            0x000E => {
                // Server message count
                let value = self.bus_server_message_count;
                payload[2] = (value >> 8) as u8;
                payload[3] = value as u8;
                Ok(4)
            }
            0x000F => {
                // Server non response count
                let value = self.server_no_response_count;
                payload[2] = (value >> 8) as u8;
                payload[3] = value as u8;
                Ok(4)
            }
            0x0010 => {
                // Server NAK count
                payload[2] = 0;
                payload[3] = 0;
                Ok(4)
            }
            0x0011 => {
                // Server busy
                payload[2] = 0;
                payload[3] = 0;
                Ok(4)
            }
            0x0012 => {
                // Fifo overrun count
                payload[2] = 0;
                payload[3] = 0;
                Ok(4)
            }
            0x0014 => {
                // Clear fifo overrun
                Ok(self.bytes_received - 4)
            }
            _ => Err(1),
        }
    }

    fn read_comm_event_counter(&mut self) -> Result<usize, u8> {
        defmt::trace!("Read comms event counter");
        let payload = &mut self.serial_data[2..];
        if self.mute {
            Err(0)
        } else {
            payload[0] = 0;
            payload[1] = 0;
            let value = self.event_count;
            payload[2] = (value >> 8) as u8;
            payload[3] = value as u8;
            Ok(4)
        }
    }
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
    // 3.5 character times at the given baudrate.
    // Per current spec, 3.5/1.5 char times are fixed at 1750, 750 us for high baudrates.
    let (timeout3_5, timeout1_5) = if baudrate > 19200 {
        (1_750, 750)
    } else {
        (11 * 3_500_000 / baudrate, 11 * 1_500_000 / baudrate)
    };

    // split the UART buffer into tx and rx halves so we can have a DMA ring buffer on the rx side
    // stm32 uarts are pathetic.
    let mut dma_buffer = [0u8; UART_BUF_LEN];
    let (mut uart_tx, uart_rx) = uart.split();
    let mut rx_ringbuffered = uart_rx.into_ring_buffered(&mut dma_buffer);
    let mut rx_buffer = [0u8; 1];
    let mut state = ModbusState::WaitForIdle3_5;
    let mut serial_timeout = timeout3_5;
    let mut modbus_data = modbus_data::default();
    let mut last_message = Instant::now();

    loop {
        let rx_status = rx_ringbuffered
            .read(&mut rx_buffer)
            .with_timeout(Duration::from_micros(serial_timeout as u64))
            .await;
        //        match rx_status {
        //            Ok(_) => {
        //                defmt::trace!("Received byte: {:?}", rx_buffer[0]);
        //            }
        //            Err(_) => {}
        //        }
        if Instant::now() - last_message > TIMEOUT_IDLE {
            modbus_data_led.set_low();
            defmt::trace!("Modbus timeout");
            if !manual_control {
                // Reset coil data if no message received for idle timeout thus stopping pulse generation
                COIL_DATA[0].store(false, Relaxed);
                COIL_DATA[1].store(false, Relaxed);
                COIL_DATA[2].store(false, Relaxed);
                COIL_DATA[3].store(false, Relaxed);
                defmt::trace!("Coils powered down");
            }
        }
        match state {
            ModbusState::WaitForIdle3_5 => {
                serial_timeout = timeout3_5;
                modbus_data.broadcast = false;
                match rx_status {
                    Ok(_) => {
                        defmt::debug!("Rejected character: {:?}", rx_buffer[0]);
                    }
                    Err(e) => {
                        if e == embassy_time::TimeoutError {
                            state = ModbusState::Idle;
                            modbus_data.bytes_received = 0;
                            serial_timeout = TIMEOUT_IDLE.as_micros() as u32; // Deliberately long timeout to wait in idle
                            defmt::trace!("Modbus state changed to Idle");
                        } else {
                            defmt::debug!("read error: {:?}", e);
                            modbus_err_led.set_high();
                            modbus_data.bytes_received = 0;
                        }
                    }
                }
            }
            ModbusState::Idle => {
                modbus_data.bytes_received = 0;
                match rx_status {
                    Ok(_) => {
                        modbus_data.serial_data[0] = rx_buffer[0];
                        modbus_data.bytes_received += 1;
                        modbus_data.bus_message_count += 1;
                        if (rx_buffer[0] == address) || (rx_buffer[0] == 0) {
                            // it is addressed to us
                            modbus_data.bus_server_message_count += 1;
                            defmt::trace!("Address match {}", rx_buffer[0]);
                            modbus_data.broadcast = rx_buffer[0] == 0;
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
                match rx_status {
                    Ok(_) => {
                        modbus_data.serial_data[modbus_data.bytes_received] = rx_buffer[0];
                        modbus_data.bytes_received += 1;
                        if modbus_data.bytes_received == modbus_data.serial_data.len() {
                            defmt::debug!("Receive length overrun");
                            modbus_err_led.set_high();
                            state = ModbusState::WaitForIdle3_5;
                            //modbus_data.bus_comms_error_count += 1;
                            break;
                        }
                    }
                    Err(e) => {
                        if e == embassy_time::TimeoutError {
                            // We got a possibly complete frame
                            defmt::debug!(
                                "Message received '{}'",
                                modbus_data.serial_data[0..modbus_data.bytes_received]
                            );
                            last_message = Instant::now(); // reset the timeout for loss of comms
                            if modbus_data.bytes_received > 3 {
                                // address, function code, CRC, CRC
                                // See if the CRC is good
                                let crc = crc16(
                                    &modbus_data.serial_data[0..modbus_data.bytes_received - 2],
                                );
                                if (crc.1
                                    == modbus_data.serial_data[modbus_data.bytes_received - 2])
                                    && (crc.0
                                        == modbus_data.serial_data[modbus_data.bytes_received - 1])
                                {
                                    defmt::trace!("CRC OK");
                                    modbus_data_led.toggle();
                                    let opcode = modbus_data.serial_data[1];
                                    let resultcode: Result<usize, u8>;
                                    match opcode {
                                        0x01 => resultcode = modbus_data.read_boolean(&COIL_DATA),
                                        0x02 => resultcode = modbus_data.read_boolean(&INPUT_DATA),
                                        0x03 => {
                                            resultcode = modbus_data.read_register(&HREGISTER_DATA)
                                        }
                                        0x04 => {
                                            resultcode = modbus_data.read_register(&IREGISTER_DATA)
                                        }

                                        0x05 => resultcode = modbus_data.write_coil(&COIL_DATA),

                                        0x06 => {
                                            resultcode = modbus_data.write_register(&HREGISTER_DATA)
                                        }
                                        0x07 => resultcode = modbus_data.read_exception_register(),
                                        0x08 => resultcode = modbus_data.read_diagnostics(),
                                        0x0b => resultcode = modbus_data.read_comm_event_counter(),

                                        0x0f => resultcode = modbus_data.write_coils(&COIL_DATA),

                                        0x10 => {
                                            resultcode =
                                                modbus_data.write_registers(&HREGISTER_DATA)
                                        }

                                        0x11 => {
                                            resultcode = modbus_data.report_server_id(&SERVER_ID)
                                        }
                                        0x2b => resultcode = modbus_data.read_pdu(),

                                        _ => {
                                            // Unknown command
                                            resultcode = Err(0x01)
                                        }
                                    }
                                    let response_bytes: usize; // Response length in bytes not including checksum, but including the address and opcode
                                    match resultcode {
                                        Ok(length) => {
                                            response_bytes = length + 2; // Address and opcode
                                            modbus_err_led.set_low();
                                            if opcode != 0x0b {
                                                // Read comm event count does not increment this per the spec.
                                                modbus_data.event_count += 1;
                                            }
                                        }
                                        Err(code) => {
                                            // send the exception response
                                            defmt::trace!("Sending exception response {}", code);
                                            modbus_data.serial_data[1] |= 0x80; // Mark this as an exception
                                            modbus_data.serial_data[2] = code;
                                            response_bytes = 3;
                                            modbus_err_led.set_high();
                                        }
                                    }
                                    let crc =
                                        crc16(&modbus_data.serial_data[..response_bytes as usize]);
                                    modbus_data.serial_data[response_bytes] = crc.1;
                                    modbus_data.serial_data[response_bytes + 1] = crc.0;
                                    defmt::trace!(
                                        "Sending {}",
                                        modbus_data.serial_data[..response_bytes + 2]
                                    );
                                    // resultcode  == Err(0) is a mute or crucially a just exiting mute operation which should not transmit.
                                    if (!modbus_data.broadcast)
                                        && (!modbus_data.mute)
                                        && (resultcode != Err(0))
                                    {
                                        uart_tx
                                            .write(&modbus_data.serial_data[..response_bytes + 2])
                                            .await
                                            .ok();
                                    } else {
                                        modbus_data.server_no_response_count += 1;
                                    }
                                } else {
                                    defmt::debug!("CRC fail");
                                    modbus_data.bus_comms_error_count += 1;
                                    modbus_err_led.set_high();
                                    defmt::trace!("Modbus state changed to Wait for Idle3_5");
                                }
                            } else {
                                modbus_data.bus_comms_error_count += 1;
                                // Messgae too short
                            }
                            modbus_data.bytes_received = 0;
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
