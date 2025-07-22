use defmt::{Format, error, info, warn};
use defmt_rtt as _;
use embedded_io_async::{Error, ErrorType, Read, Write};
use thiserror::Error;

#[derive(Format, Error, Debug, Clone, Copy)]
pub enum UartError<U: Error> {
    #[error("TxError: {0:?}")]
    TxError(U),
    #[error("RxError: {0:?}")]
    RxError(U),
    #[error("CRC mismatch")]
    CrcMismatch,
    #[error("Unexpected end of stream")]
    UnexpectedEos,
}

#[derive(Format, Debug)]
pub struct Tmc2209<U: Read + Write + ErrorType> {
    uart: U,
}

impl<U: Read + Write + ErrorType> Tmc2209<U> {
    pub fn new(uart: U) -> Self {
        Self { uart }
    }

    pub async fn write_register(
        &mut self,
        slave_address: u8,
        register: u8,
        data: u32,
    ) -> Result<(), UartError<U::Error>> {
        let message = Self::construct_write_uart_message(slave_address, register, data);
        self.uart
            .write_all(&message)
            .await
            .map_err(UartError::TxError)
    }

    // FIXME: Techincally, the magic bytes of [0x05, 0xff] could be part of the body of the message.
    pub async fn read_register(
        &mut self,
        slave_address: u8,
        register: u8,
    ) -> Result<u32, UartError<U::Error>> {
        const REPLY_BYTES: [u8; 2] = [0x05, 0xff];
        let sent = self
            .uart
            .write(&Self::construct_read_uart_message(slave_address, register))
            .await
            .map_err(UartError::TxError)?;
        info!("sent {} bytes through uart.", sent);

        let mut buffer: [u8; 8] = [0; 8];
        loop {
            let len = self
                .uart
                .read(&mut buffer)
                .await
                .map_err(UartError::RxError)?;
            info!("received: {=[u8]:02x}", buffer[..len]);

            // search for the 'magic bytes' indicating the start of a message.
            if let Some(message_start) = buffer[..len].windows(2).position(|b| b == REPLY_BYTES) {
                // if we find it, we put it at the very start of the buffer.
                buffer.copy_within(message_start..len, 0);
                let fragment_end = len - message_start;
                info!("Got fragment! {=[u8]:02x}", buffer[..fragment_end]);
                // now we continue reading till we fill the rest of the buffer.
                self.uart
                    .read_exact(&mut buffer[fragment_end..])
                    .await
                    .map_err(|e| match e {
                        embedded_io::ReadExactError::UnexpectedEof => UartError::UnexpectedEos,
                        embedded_io::ReadExactError::Other(i) => UartError::RxError(i),
                    })?;
                info!("Message is: {=[u8;8]:02x}", buffer);

                let returned_address = buffer[3];
                // That was a reply from a different adress than expected, hope no other task was
                // waiting for that!
                if returned_address != (slave_address & 0x7F) {
                    warn!(
                        "got reply from {:02x}, expected from {:02x}",
                        returned_address, slave_address
                    );
                    continue;
                }

                // calc the CRC and return either the message or the CRC error.
                return if Self::calc_uart_crc(&buffer[..buffer.len() - 1])
                    == buffer[buffer.len() - 1]
                {
                    Ok(u32::from_be_bytes(buffer[3..7].try_into().unwrap()))
                } else {
                    Err(UartError::CrcMismatch)
                };
            }
        }
    }

    fn construct_write_uart_message(slave_address: u8, register: u8, data: u32) -> [u8; 8] {
        let [d1, d2, d3, d4] = data.to_be_bytes();
        let mut msg: [u8; 8] = [
            0b1010_0101,
            slave_address,
            (register | 0x80),
            d1,
            d2,
            d3,
            d4,
            0,
        ];
        msg[7] = Self::calc_uart_crc(&msg[..7]);
        msg
    }

    fn construct_read_uart_message(slave_address: u8, register: u8) -> [u8; 4] {
        let mut msg: [u8; 4] = [0b1010_0101, (slave_address & 0x7F), register, 0];
        msg[3] = Self::calc_uart_crc(&msg[..3]);
        msg
    }

    fn calc_uart_crc(message: &[u8]) -> u8 {
        let mut crc: u8 = 0;
        for byte in message {
            for i in 0..8 {
                crc = if ((crc >> 7) ^ ((byte >> i) & 0x01)) != 0 {
                    (crc << 1) ^ 0x07
                } else {
                    crc << 1
                }
            }
        }
        crc
    }
}
