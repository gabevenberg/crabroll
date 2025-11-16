#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![allow(clippy::unusual_byte_groupings)]

mod tmc2209;
pub mod stepper;

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::{
    clock::CpuClock,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    interrupt::software::SoftwareInterruptControl,
    timer::systimer::SystemTimer,
    uart::{Config, Uart},
};
use panic_rtt_target as _;
use tmc2209::Tmc2209;

esp_bootloader_esp_idf::esp_app_desc!();

const _HOSTNAME: &str = env!("HOSTNAME");

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_rtos::start(timer0.alarm0, sw_int.software_interrupt0);

    info!("Embassy initialized!");

    let step_pin = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());
    let dir_pin = Output::new(peripherals.GPIO6, Level::Low, OutputConfig::default());
    let endstop_pin = Input::new(
        peripherals.GPIO2,
        InputConfig::default().with_pull(Pull::Up),
    );
    let green_led_pin = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());
    let red_led_pin = Output::new(peripherals.GPIO9, Level::Low, OutputConfig::default());

    let button_1_pin = Input::new(
        peripherals.GPIO10,
        InputConfig::default().with_pull(Pull::Up),
    );
    let button_2_pin = Input::new(
        peripherals.GPIO3,
        InputConfig::default().with_pull(Pull::Up),
    );
    let button_3_pin = Input::new(
        peripherals.GPIO4,
        InputConfig::default().with_pull(Pull::Up),
    );
    let button_4_pin = Input::new(
        peripherals.GPIO5,
        InputConfig::default().with_pull(Pull::Up),
    );

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

    let mut tmc2209 = Tmc2209::new(uart, [true, false, false, false])
        .await
        .unwrap();
    // setup general config
    tmc2209.write_register(0, 0, 0b0111000001).await.unwrap();

    // set microstepping to fullstep
    tmc2209
        .write_register(0, 0x6c, 0b0001_1000_000000000000000110010011)
        .await
        .unwrap();

    spawner
        .spawn(turn_motor(step_pin, dir_pin, green_led_pin))
        .unwrap();
    spawner
        .spawn(light_led_with_button(button_1_pin, red_led_pin))
        .unwrap();
}

#[embassy_executor::task]
async fn light_led_with_button(mut button: Input<'static>, mut led: Output<'static>) {
    loop {
        button.wait_for_low().await;
        led.toggle();
        Timer::after(Duration::from_millis(50)).await;
        button.wait_for_high().await;
        Timer::after(Duration::from_millis(50)).await;
    }
}

#[embassy_executor::task]
async fn turn_motor(
    mut step_pin: Output<'static>,
    mut dir_pin: Output<'static>,
    mut led: Output<'static>,
) {
    loop {
        for _ in 0..200 {
            step_pin.set_high();
            Timer::after(Duration::from_hz(200 * 2)).await;
            step_pin.set_low();
            Timer::after(Duration::from_hz(200 * 2)).await;
        }
        Timer::after_nanos(100).await;
        dir_pin.toggle();
        led.toggle();
        Timer::after_nanos(100).await;
    }
}
