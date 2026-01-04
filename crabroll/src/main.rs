#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![allow(clippy::unusual_byte_groupings)]

mod motor;
mod tmc2209;

use core::net::Ipv4Addr;

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net::{Runner, StackResources, tcp::TcpSocket};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, rwlock::RwLock, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::Write;
use esp_alloc as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    interrupt::{Priority, software::SoftwareInterruptControl},
    timer::systimer::SystemTimer,
    uart::{Config, Uart},
};
use esp_radio::{
    Controller,
    wifi::{
        ClientConfig, ModeConfig, ScanConfig, WifiController, WifiDevice, WifiEvent, WifiStaState,
    },
};
use esp_rtos::embassy::InterruptExecutor;
use iter_step_gen::Direction;
use panic_rtt_target as _;
use static_cell::StaticCell;
use tmc2209::Tmc2209;

use crate::motor::motor_task;

esp_bootloader_esp_idf::esp_app_desc!();

const _HOSTNAME: &str = env!("HOSTNAME");

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // esp_alloc::heap_allocator!(size: 64 * 1024);
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
        peripherals.GPIO10,
        InputConfig::default().with_pull(Pull::Up),
    );
    let raise_button = Input::new(
        peripherals.GPIO3,
        InputConfig::default().with_pull(Pull::Up),
    );
    let lower_button = Input::new(
        peripherals.GPIO4,
        InputConfig::default().with_pull(Pull::Up),
    );
    let bottom_button = Input::new(
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

    spawner.spawn(home_button_task(home_button)).unwrap();
    spawner.spawn(raise_button_task(raise_button)).unwrap();
    spawner.spawn(lower_button_task(lower_button)).unwrap();
    spawner.spawn(bottom_button_task(bottom_button)).unwrap();
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

    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(runner)).ok();

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    info!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            info!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    loop {
        Timer::after(Duration::from_millis(1_000)).await;

        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);

        socket.set_keep_alive(Some(Duration::from_secs(5)));
        socket.set_timeout(Some(Duration::from_secs(10)));

        let remote_endpoint = (Ipv4Addr::new(142, 250, 185, 115), 80);
        info!("connecting...");
        let r = socket.connect(remote_endpoint).await;
        if let Err(e) = r {
            info!("connect error: {:?}", e);
            continue;
        }
        info!("connected!");
        let mut buf = [0; 1024];
        loop {
            let r = socket
                .write_all(b"GET / HTTP/1.0\r\nHost: www.mobile-j.de\r\n\r\n")
                .await;
            if let Err(e) = r {
                info!("write error: {:?}", e);
                break;
            }
            let n = match socket.read(&mut buf).await {
                Ok(0) => {
                    info!("read EOF");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    info!("read error: {:?}", e);
                    break;
                }
            };
            info!("{}", core::str::from_utf8(&buf[..n]).unwrap());
        }
        Timer::after(Duration::from_millis(3000)).await;
    }
}

#[derive(Eq, PartialEq)]
enum Commands {
    Home,
    StartJog(Direction),
    StopJog,
    Bottom,
    SetBottom,
    MoveToPos(u32),
}

static DIR_TO_HOME: RwLock<CriticalSectionRawMutex, Level> = RwLock::new(Level::Low);
static LAST_COMMAND: Signal<CriticalSectionRawMutex, Commands> = Signal::new();

#[embassy_executor::task]
async fn home_button_task(mut button: Input<'static>) {
    loop {
        button.wait_for_low().await;
        let start_press = Instant::now();
        Timer::after_millis(50).await;
        button.wait_for_high().await;
        if start_press.elapsed() > Duration::from_secs(1) {
            LAST_COMMAND.signal(Commands::Home);
            info!("home button long pushed");
        } else {
            LAST_COMMAND.signal(Commands::MoveToPos(0));
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
        LAST_COMMAND.signal(Commands::StartJog(Direction::ToHome));
        Timer::after_millis(50).await;
        button.wait_for_high().await;
        LAST_COMMAND.signal(Commands::StopJog);
        Timer::after_millis(50).await;
    }
}

#[embassy_executor::task]
async fn lower_button_task(mut button: Input<'static>) {
    loop {
        button.wait_for_low().await;
        info!("lower button pushed");
        LAST_COMMAND.signal(Commands::StartJog(Direction::AwayFromHome));
        Timer::after_millis(50).await;
        button.wait_for_high().await;
        LAST_COMMAND.signal(Commands::StopJog);
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
            LAST_COMMAND.signal(Commands::SetBottom);
            info!("bottom button long pushed");
        } else {
            LAST_COMMAND.signal(Commands::Bottom);
            info!("bottom button pushed");
        }
        Timer::after_millis(50).await;
    }
}

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    info!("start connection task");
    info!("Device capabilities: {:?}", controller.capabilities());
    loop {
        if esp_radio::wifi::sta_state() == WifiStaState::Connected {
            // wait until we're no longer connected
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            Timer::after(Duration::from_millis(5000)).await
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let station_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(SSID.into())
                    .with_password(PASSWORD.into()),
            );
            controller.set_config(&station_config).unwrap();
            info!("Starting wifi");
            controller.start_async().await.unwrap();
            info!("Wifi started!");

            info!("Scan");
            let scan_config = ScanConfig::default().with_max(10);
            let result = controller
                .scan_with_config_async(scan_config)
                .await
                .unwrap();
            for ap in result {
                info!("{:?}", ap);
            }
        }
        info!("About to connect...");

        match controller.connect_async().await {
            Ok(_) => info!("Wifi connected!"),
            Err(e) => {
                info!("Failed to connect to wifi: {:?}", e);
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}
