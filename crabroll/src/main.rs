#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![allow(clippy::unusual_byte_groupings)]

mod tmc2209;

use core::num::NonZero;

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_hal::{
    clock::CpuClock,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    interrupt::{software::SoftwareInterruptControl, Priority},
    timer::systimer::SystemTimer,
    uart::{Config, Uart},
};
use esp_rtos::embassy::InterruptExecutor;
use iter_step_gen::{Direction, Stepper};
use panic_rtt_target as _;
use static_cell::StaticCell;
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

    static EXECUTOR: StaticCell<InterruptExecutor<2> >  = StaticCell::new();
    let step_executor = InterruptExecutor::new(sw_int.software_interrupt2);
    let step_executor = EXECUTOR.init(step_executor);
    let step_spawner = step_executor.start(Priority::Priority3);

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
    info!("IO initalized!");

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
    info!("UART initalized!");

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

    // TODO: figure out why I need this, else we stall here.
    // Bug in the UART code/TMC?
    Timer::after_millis(1).await;

    // set current limiting
    tmc2209
        .write_register(0, 0x10, 0b0000_10000_00000)
        .await
        .unwrap();

    step_spawner
        .spawn(turn_motor(step_pin, dir_pin, endstop_pin, green_led_pin))
        .unwrap();
    spawner
        .spawn(light_led_with_button(button_1_pin, red_led_pin))
        .unwrap();
    info!("Tasks spawned!");
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
    endstop_pin: Input<'static>,
    mut led: Output<'static>,
) {
    let mut step_planner = Stepper::new(
        NonZero::new(200 * 16).unwrap(),
        NonZero::new(200 * 16).unwrap(),
        NonZero::new(200 * 2).unwrap(),
        50,
        Direction::Cw,
    );

    dir_pin.set_low();

    let (plan, _) = step_planner.homing_move(|| endstop_pin.is_low());
    for delay in plan {
        let instant = Instant::now();
        step_pin.set_high();
        Timer::after(Duration::from_nanos(100)).await;
        step_pin.set_low();
        Timer::at(instant.saturating_add(delay)).await;
    }

    info!("homed!");

    loop {
        led.set_high();
        let (plan, _) = step_planner.planned_move(500).unwrap();
        for delay in plan {
            let instant = Instant::now();
            step_pin.set_high();
            Timer::after(Duration::from_nanos(100)).await;
            step_pin.set_low();
            Timer::at(instant.saturating_add(delay)).await;
        }
        dir_pin.set_high();

        led.set_low();
        let (plan, _) = step_planner.planned_move(0).unwrap();
        for delay in plan {
            let instant = Instant::now();
            step_pin.set_high();
            Timer::after(Duration::from_nanos(100)).await;
            step_pin.set_low();
            Timer::at(instant.saturating_add(delay)).await;
        }
        dir_pin.set_low();
    }
}
