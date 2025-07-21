#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::{error, info, warn, Format};
use defmt_rtt as _;
use embassy_executor::Spawner;
use esp_hal::Async;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::uart::{Config, RxError, TxError, Uart, UartRx, UartTx};
use panic_rtt_target as _;
use thiserror::Error;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");

    // let rng = esp_hal::rng::Rng::new(peripherals.RNG);
    // let timer1 = TimerGroup::new(peripherals.TIMG0);
    // let wifi_init = esp_wifi::init(timer1.timer0, rng, peripherals.RADIO_CLK)
    //     .expect("Failed to initialize WIFI/BLE controller");
    // let (mut _wifi_controller, _interfaces) = esp_wifi::wifi::new(&wifi_init, peripherals.WIFI)
    //     .expect("Failed to initialize WIFI controller");

    let _step_pin = Output::new(peripherals.GPIO6, Level::Low, OutputConfig::default());
    let _dir_pin = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());
    let uart = Uart::new(
        peripherals.UART0,
        Config::default()
            .with_baudrate(115200)
            .with_parity(esp_hal::uart::Parity::None),
    )
    .unwrap()
    .with_tx(peripherals.GPIO21)
    .with_rx(peripherals.GPIO20)
    .into_async();
    let (mut rx, mut tx) = uart.split();

    let sent = tx
        .write_async(&construct_write_uart_message(0, 0, 0b0011000000))
        .await
        .unwrap();
    info!("sent {} bytes through uart.", sent);
    let register = read_register(0, 0, &mut rx, &mut tx).await.unwrap();
    info!("register 0 is {=[u8]:02x}", register)
}

#[derive(Format, Error, Debug, Clone, Copy)]
enum UartError {
    #[error("TxError: {0:?}")]
    TxError(TxError),
    #[error("RxError: {0:?}")]
    RxError(RxError),
    #[error("CRC mismatch")]
    CrcMismatch,
}

// FIXME: Techincally, the magic bytes of '0x05, 0xff' could be part of the body of the message.
async fn read_register(
    slave_address: u8,
    register: u8,
    rx: &mut UartRx<'_, Async>,
    tx: &mut UartTx<'_, Async>,
) -> Result<[u8; 4], UartError> {
    const REPLY_BYTES: [u8; 2] = [0x05, 0xff];
    let sent = tx
        .write_async(&construct_read_uart_message(slave_address, register))
        .await
        .map_err(UartError::TxError)?;
    info!("sent {} bytes through uart.", sent);

    let mut buffer: [u8; 8] = [0; 8];
    loop {
        let len = rx
            .read_async(&mut buffer)
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
            rx.read_exact_async(&mut buffer[fragment_end..])
                .await
                .map_err(UartError::RxError)?;
            info!("Message is: {=[u8;8]:02x}", buffer);

            let returned_address = buffer[3];
            // That was a reply from a different adress than expected, hope no other task was
            // waiting for that!
            if returned_address != (slave_address & 0x7F) {
                warn!("got reply from {:02x}, expected from {:02x}", returned_address, slave_address);
                continue;
            }

            // calc the CRC and return either the message or the CRC error.
            return if calc_uart_crc(&buffer[..buffer.len() - 1]) == buffer[buffer.len() - 1] {
                Ok(buffer[3..7].try_into().unwrap())
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
    msg[7] = calc_uart_crc(&msg[..7]);
    msg
}

fn construct_read_uart_message(slave_address: u8, register: u8) -> [u8; 4] {
    let mut msg: [u8; 4] = [0b1010_0101, (slave_address & 0x7F), register, 0];
    msg[3] = calc_uart_crc(&msg[..3]);
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
