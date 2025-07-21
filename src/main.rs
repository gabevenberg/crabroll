#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use esp_hal::Async;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::uart::{Config, Uart, UartRx};
use panic_rtt_target as _;

extern crate alloc;

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
    let (rx, mut tx) = uart.split();
    spawner.spawn(read_uart_response(rx)).unwrap();

    let sent = tx
        .write_async(&construct_write_uart_message(0, 0, 0b0000001100))
        .await
        .unwrap();
    info!("sent uart: {}", sent);
    let sent = tx
        .write_async(&construct_read_uart_message(0, 0))
        .await
        .unwrap();
    info!("sent uart: {}", sent);
}

#[embassy_executor::task]
async fn read_uart_response(mut rx: UartRx<'static, Async>) {
    let mut uart_buffer: [u8; 8] = [0; 8];
    loop {
        let len = rx.read_async(&mut uart_buffer).await.unwrap_or_else(|e| {
            error!("{}", e);
            0
        });
        info!("received: {=[u8]:02x}", uart_buffer[..len]);
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
