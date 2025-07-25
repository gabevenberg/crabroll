#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

mod tmc2209;
mod wifi;

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net::{DhcpConfig, StackResources};
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config, Uart};
use esp_wifi::EspWifiController;
use panic_rtt_target as _;
use static_cell::StaticCell;
use tmc2209::Tmc2209;
use wifi::{connection, network_task};

esp_bootloader_esp_idf::esp_app_desc!();

const HOSTNAME: &str = env!("HOSTNAME");

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");

    let mut rng = esp_hal::rng::Rng::new(peripherals.RNG);
    let timer1 = TimerGroup::new(peripherals.TIMG0);

    static WIFI_INIT: StaticCell<EspWifiController> = StaticCell::new();
    let wifi_init: &'static EspWifiController = WIFI_INIT.init_with(|| {
        esp_wifi::init(timer1.timer0, rng, peripherals.RADIO_CLK)
            .expect("Failed to initialize WIFI/BLE controller")
    });

    let (wifi_controller, interfaces) = esp_wifi::wifi::new(wifi_init, peripherals.WIFI)
        .expect("Failed to initialize WIFI controller");
    let wifi_interface = interfaces.sta;

    let mut dhcp_config = DhcpConfig::default();
    dhcp_config.hostname = HOSTNAME.try_into().ok();

    let config = embassy_net::Config::dhcpv4(dhcp_config);
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    static STACK_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let stack_resources: &'static mut StackResources<3> =
        STACK_RESOURCES.init_with(StackResources::<3>::new);
    let (stack, runner) = embassy_net::new(wifi_interface, config, stack_resources, seed);

    spawner.spawn(connection(wifi_controller)).unwrap();
    spawner.spawn(network_task(runner)).unwrap();

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

    let mut tmc2209 = Tmc2209::new(uart);
    tmc2209.write_register(0, 0, 0b0111000001).await.unwrap();
    info!(
        "0th register is {=u32:08x}",
        tmc2209.read_register(0, 0).await.unwrap()
    );
    info!(
        "IT counter register is {=u32:08x}",
        tmc2209.read_register(0, 0x02).await.unwrap()
    );

    while !stack.is_config_up() {
        Timer::after(Duration::from_millis(500)).await;
    }
    info!("got IP: {}", stack.config_v4().unwrap().address);
}
