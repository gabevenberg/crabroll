#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![warn(clippy::all)]
#![allow(clippy::unusual_byte_groupings)]

mod motor;
mod mqtt;
mod tmc2209;
mod wifi;

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net::StackResources;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, rwlock::RwLock, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use esp_alloc as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    interrupt::{Priority, software::SoftwareInterruptControl},
    timer::systimer::SystemTimer,
    uart::{Config, Uart},
};
use esp_radio::Controller;
use esp_rtos::embassy::InterruptExecutor;
use iter_step_gen::Direction;
use panic_rtt_target as _;
use static_cell::StaticCell;
use tmc2209::Tmc2209;

use crate::{
    motor::motor_task,
    mqtt::mqtt_task,
    wifi::{connection, net_task},
};

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_rtos::start(timer0.alarm0, sw_int.software_interrupt0);

    info!("Embassy initialized!");

    static EXECUTOR: StaticCell<InterruptExecutor<2>> = StaticCell::new();
    let step_executor = InterruptExecutor::new(sw_int.software_interrupt2);
    let step_executor = EXECUTOR.init(step_executor);
    let step_spawner = step_executor.start(Priority::Priority10);

    let step_pin = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());
    let dir_pin = Output::new(peripherals.GPIO6, Level::Low, OutputConfig::default());
    let endstop_pin = Input::new(
        peripherals.GPIO2,
        InputConfig::default().with_pull(Pull::Up),
    );
    let green_led_pin = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());
    let red_led_pin = Output::new(peripherals.GPIO9, Level::Low, OutputConfig::default());

    let home_button = Input::new(
        peripherals.GPIO5,
        InputConfig::default().with_pull(Pull::Up),
    );
    let raise_button = Input::new(
        peripherals.GPIO4,
        InputConfig::default().with_pull(Pull::Up),
    );
    let lower_button = Input::new(
        peripherals.GPIO3,
        InputConfig::default().with_pull(Pull::Up),
    );
    let bottom_button = Input::new(
        peripherals.GPIO10,
        InputConfig::default().with_pull(Pull::Up),
    );
    info!("IO initalized!");

    let uart = Uart::new(
        peripherals.UART0,
        Config::default()
            .with_baudrate(115_200)
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

    spawner.spawn(home_button_task(home_button)).unwrap();
    spawner.spawn(raise_button_task(raise_button)).unwrap();
    spawner.spawn(lower_button_task(lower_button)).unwrap();
    spawner.spawn(bottom_button_task(bottom_button)).unwrap();
    spawner.spawn(error_led_task(red_led_pin)).unwrap();
    step_spawner
        .spawn(motor_task(step_pin, dir_pin, endstop_pin))
        .unwrap();

    info!("Motor tasks spawned!");

    static RADIO_CONTROLLER: StaticCell<Controller> = StaticCell::new();
    let radio_controller = RADIO_CONTROLLER.init_with(|| esp_radio::init().unwrap());

    let (controller, interfaces) =
        esp_radio::wifi::new(radio_controller, peripherals.WIFI, Default::default()).unwrap();

    let wifi_interface = interfaces.sta;

    let config = embassy_net::Config::dhcpv4(Default::default());

    let rng = esp_hal::rng::Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    static STACK_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let stack_resources = STACK_RESOURCES.init_with(StackResources::<3>::new);

    // Init network stack
    let (stack, runner) = embassy_net::new(wifi_interface, config, stack_resources, seed);

    spawner.spawn(connection(controller)).unwrap();
    spawner.spawn(net_task(runner)).unwrap();
    spawner.spawn(mqtt_task(stack)).unwrap();
}

#[derive(Eq, PartialEq)]
enum Command {
    Home,
    StartJog(Direction),
    StopJog,
    SetBottom,
    MoveToPos(i8),
}

static DIR_TO_HOME: RwLock<CriticalSectionRawMutex, Level> = RwLock::new(Level::Low);
static LAST_COMMAND: Signal<CriticalSectionRawMutex, Command> = Signal::new();
// in percentage, if -1, current position is unknown. Should also try to replace with an atomic.
static CURRENT_POS: Signal<CriticalSectionRawMutex, i8> = Signal::new();
//TODO: Surely theres a way to use an atomicbool here? The main thing is we need to be able to
//await it.
static ERROR_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[embassy_executor::task]
async fn error_led_task(mut led: Output<'static>) {
    ERROR_SIGNAL.wait().await;
    led.set_high();
    Timer::after_secs(1).await;
    led.set_low();
}

#[embassy_executor::task]
async fn home_button_task(mut button: Input<'static>) {
    loop {
        button.wait_for_low().await;
        let start_press = Instant::now();
        Timer::after_millis(50).await;
        button.wait_for_high().await;
        if start_press.elapsed() > Duration::from_secs(1) {
            LAST_COMMAND.signal(Command::Home);
            info!("home button long pushed");
        } else {
            LAST_COMMAND.signal(Command::MoveToPos(0));
            info!("home button pushed");
        }
        Timer::after_millis(50).await;
    }
}

#[embassy_executor::task]
async fn raise_button_task(mut button: Input<'static>) {
    loop {
        button.wait_for_low().await;
        info!("raise button pushed");
        LAST_COMMAND.signal(Command::StartJog(Direction::ToHome));
        Timer::after_millis(50).await;
        button.wait_for_high().await;
        LAST_COMMAND.signal(Command::StopJog);
        Timer::after_millis(50).await;
    }
}

#[embassy_executor::task]
async fn lower_button_task(mut button: Input<'static>) {
    loop {
        button.wait_for_low().await;
        info!("lower button pushed");
        LAST_COMMAND.signal(Command::StartJog(Direction::AwayFromHome));
        Timer::after_millis(50).await;
        button.wait_for_high().await;
        LAST_COMMAND.signal(Command::StopJog);
        Timer::after_millis(50).await;
    }
}

#[embassy_executor::task]
async fn bottom_button_task(mut button: Input<'static>) {
    loop {
        button.wait_for_low().await;
        let start_press = Instant::now();
        Timer::after_millis(50).await;
        button.wait_for_high().await;
        if start_press.elapsed() > Duration::from_secs(1) {
            LAST_COMMAND.signal(Command::SetBottom);
            info!("bottom button long pushed");
        } else {
            LAST_COMMAND.signal(Command::MoveToPos(100));
            info!("bottom button pushed");
        }
        Timer::after_millis(50).await;
    }
}
