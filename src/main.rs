#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![allow(clippy::unusual_byte_groupings)]

mod tmc2209;
mod wifi;

use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net::dns::DnsQueryType;
use embassy_net::tcp::TcpSocket;
use embassy_net::{DhcpConfig, IpEndpoint, StackResources};
use embassy_time::{Duration, Timer};
use embedded_tls::{Aes128GcmSha256, NoVerify, TlsConfig, TlsConnection, TlsContext};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::rng::{Rng, Trng};
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config, Uart};
use esp_wifi::EspWifiController;
use panic_rtt_target as _;
use rust_mqtt::client::client::MqttClient;
use rust_mqtt::client::client_config::{ClientConfig, MqttVersion};
use static_cell::StaticCell;
use tmc2209::Tmc2209;
use wifi::{connection, network_task};

esp_bootloader_esp_idf::esp_app_desc!();

const HOSTNAME: &str = env!("HOSTNAME");

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let mut peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    // gotten from https://github.com/esp-rs/esp-hal/issues/2372, use clonable trng when possible.
    let mut rng = Rng::new(peripherals.RNG.reborrow());
    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");

    let timer1 = TimerGroup::new(peripherals.TIMG0);

    static WIFI_INIT: StaticCell<EspWifiController> = StaticCell::new();
    let wifi_init: &'static EspWifiController = WIFI_INIT.init_with(|| {
        esp_wifi::init(timer1.timer0, rng).expect("Failed to initialize WIFI/BLE controller")
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

    let step_pin = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());
    let dir_pin = Output::new(peripherals.GPIO6, Level::Low, OutputConfig::default());
    let _enstop_pin = Input::new(
        peripherals.GPIO10,
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

    spawner.spawn(turn_motor(step_pin, dir_pin)).unwrap();

    while !stack.is_config_up() {
        Timer::after(Duration::from_millis(500)).await;
    }
    info!("got IP: {}", stack.config_v4().unwrap().address);

    const BROKER_HOST: &str = "linuxgamingrig.local";
    // open tcp socket:
    let mqtt_broker_address = stack
        .dns_query(BROKER_HOST, DnsQueryType::A)
        .await
        .unwrap();
    info!("broker adress is {}", mqtt_broker_address);
    let mqtt_endpoint = IpEndpoint::new(mqtt_broker_address[0], 1883);
    let mut rx_buffer = [0; 1024];
    let mut tx_buffer = [0; 1024];
    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    socket.connect(mqtt_endpoint).await.unwrap();
    info!("connected to endpoint");

    //tcp
    let config: TlsConfig<'_, Aes128GcmSha256> = TlsConfig::new()
        .with_server_name(BROKER_HOST)
        .enable_rsa_signatures();
    let mut read_record_buffer = [0; 16640];
    let mut write_record_buffer = [0; 16640];
    let mut tls_connection: TlsConnection<'_, _, Aes128GcmSha256> =
        TlsConnection::new(socket, &mut read_record_buffer, &mut write_record_buffer);
    let mut trng = esp_hal::rng::Trng::new(peripherals.RNG, peripherals.ADC1);
    tls_connection.open::<Trng, NoVerify>(TlsContext::new(&config, &mut trng)).await.unwrap();

    //mqtt connection.
    let mut config = ClientConfig::new(MqttVersion::MQTTv5, rng);
    config.max_packet_size = 100;
    let mut recv_buffer = [0; 1024];
    let recv_buffer_len = recv_buffer.len();
    let mut write_buffer = [0; 1024];
    let write_buffer_len = recv_buffer.len();
    let mut client = MqttClient::<_, 5, Rng>::new(
        tls_connection,
        &mut write_buffer,
        write_buffer_len,
        &mut recv_buffer,
        recv_buffer_len,
        config,
    );
    client.connect_to_broker().await.unwrap();
    info!("connected to mqtt!");
    loop {
        Timer::after(Duration::from_secs(30)).await;
        match client.send_ping().await {
            Ok(_) => continue,
            Err(e) => error!("mqtt error: {}", e),
        };
    }
}

#[embassy_executor::task]
async fn turn_motor(mut step_pin: Output<'static>, mut dir_pin: Output<'static>) {
    loop {
        for _ in 0..200 {
            step_pin.set_high();
            Timer::after(Duration::from_hz(200 * 2)).await;
            step_pin.set_low();
            Timer::after(Duration::from_hz(200 * 2)).await;
        }
        Timer::after_nanos(100).await;
        dir_pin.toggle();
        Timer::after_nanos(100).await;
    }
}
