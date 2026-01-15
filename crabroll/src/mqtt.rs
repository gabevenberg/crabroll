use core::net::Ipv4Addr;
use defmt::{error, info};
use embassy_futures::select::{Either3, select3};
use embassy_net::{IpAddress, Stack, tcp::TcpSocket};
use embassy_time::{Duration, Timer, WithTimeout};
use heapless::format;
use rust_mqtt::{
    Bytes,
    buffer::AllocBuffer,
    client::{
        Client,
        event::{Event, Suback},
        options::{
            ConnectOptions, PublicationOptions, RetainHandling, SubscriptionOptions, WillOptions,
        },
    },
    config::{KeepAlive, SessionExpiryInterval},
    types::{MqttBinary, MqttString, QoS, TopicName},
};

use crate::{CURRENT_POS, Command, LAST_COMMAND};

const HOST_ID: MqttString = unsafe { MqttString::from_slice_unchecked(env!("HOST_ID")) };
const COMMAND_TOPIC: MqttString =
    unsafe { MqttString::from_slice_unchecked(concat!(env!("MQTT_TOPIC_PREFIX"), "command")) };
const POS_TOPIC: MqttString =
    unsafe { MqttString::from_slice_unchecked(concat!(env!("MQTT_TOPIC_PREFIX"), "pos")) };
const MQTT_USERNAME: MqttString =
    unsafe { MqttString::from_slice_unchecked(env!("MQTT_USERNAME")) };
const MQTT_PASSWORD: MqttString =
    unsafe { MqttString::from_slice_unchecked(env!("MQTT_PASSWORD")) };
const MQTT_BROKER_IP: &str = env!("MQTT_BROKER_IP");
const KEEPALIVE_TIME: u16 = 60;

// TODO: this is messy, needs better error handling.
#[embassy_executor::task]
pub(crate) async fn mqtt_task(stack: Stack<'static>) {
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    loop {
        while !stack.is_link_up() {
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

        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);

        socket.set_keep_alive(Some(Duration::from_secs(5)));
        socket.set_timeout(Some(Duration::from_secs(10)));

        let mut buffer = AllocBuffer;

        let mut client = Client::<_, _, 5, 3, 3>::new(&mut buffer);
        let addr: IpAddress = MQTT_BROKER_IP.parse::<Ipv4Addr>().unwrap().into();
        if let Err(e) = socket.connect((addr, 1883)).await {
            error!("Error connecting to mqtt server: {}", e);
            socket.abort();
            if let Err(e) = socket.flush().with_timeout(Duration::from_secs(5)).await {
                error!("error aborting connection: {:?}", e);
            };
            continue;
        };

        match client
            .connect(
                socket,
                &ConnectOptions {
                    clean_start: false,
                    keep_alive: KeepAlive::Seconds(KEEPALIVE_TIME),
                    session_expiry_interval: SessionExpiryInterval::Seconds(
                        (KEEPALIVE_TIME * 2).into(),
                    ),
                    user_name: Some(MQTT_USERNAME),
                    password: Some(MQTT_PASSWORD.into()),
                    will: Some(WillOptions {
                        will_qos: QoS::ExactlyOnce,
                        will_retain: true,
                        will_topic: MqttString::try_from("crabroll-dead").unwrap(),
                        will_payload: MqttBinary::try_from("crabroll died :(").unwrap(),
                        will_delay_interval: 10,
                        is_payload_utf8: true,
                        message_expiry_interval: Some(20),
                        content_type: Some(MqttString::try_from("txt").unwrap()),
                        response_topic: None,
                        correlation_data: None,
                    }),
                },
                Some(HOST_ID),
            )
            .await
        {
            Ok(c) => {
                info!("Connected to server: {:?}", c);
                info!("{:?}", client.client_config());
                info!("{:?}", client.server_config());
                info!("{:?}", client.shared_config());
                info!("{:?}", client.session());
            }
            Err(e) => {
                error!("failed to oconnect to broker: {:?}", e);
                if let Err(e) = client.abort().with_timeout(Duration::from_secs(5)).await {
                    error!("error aborting connection: {:?}", e);
                };
                continue;
            }
        }

        let sub_options = SubscriptionOptions {
            retain_handling: RetainHandling::SendIfNotSubscribedBefore,
            retain_as_published: true,
            no_local: false,
            qos: QoS::ExactlyOnce,
        };

        // saftey: The string is static, we know it is the correct syntax. Also, since this is not a
        // memory saftey issue, I disagree this function needs to be unsafe at all.
        let command_topic = unsafe { TopicName::new_unchecked(COMMAND_TOPIC) };
        let pos_topic = unsafe { TopicName::new_unchecked(POS_TOPIC) };

        let pub_options = PublicationOptions {
            retain: true,
            topic: pos_topic,
            qos: QoS::AtMostOnce,
        };
        client
            .subscribe(command_topic.clone().into(), sub_options)
            .await
            .unwrap();

        match client.poll().await {
            Ok(Event::Suback(Suback {
                packet_identifier: _,
                reason_code,
            })) => info!("Subscribed with reason code {:?}", reason_code),
            Ok(e) => {
                error!("Expected Suback but received event {:?}", e);
                if let Err(e) = client.abort().with_timeout(Duration::from_secs(5)).await {
                    error!("error aborting connection: {:?}", e);
                };
                continue;
            }
            Err(e) => {
                error!("Failed to receive Suback {:?}", e);
                if let Err(e) = client.abort().with_timeout(Duration::from_secs(5)).await {
                    error!("error aborting connection: {:?}", e);
                };
                continue;
            }
        };
        loop {
            match select3(
                Timer::after_secs(KEEPALIVE_TIME.into()),
                client.poll_header(),
                CURRENT_POS.wait(),
            )
            .await
            {
                Either3::First(_) => {
                    if let Err(e) = client.ping().await {
                        error!("failed to ping: {:?}", e);
                        break;
                    } else {
                        info!("pinged broker");
                    }
                }
                Either3::Second(Err(e)) => {
                    error!("error polling: {:?}", e);
                    break;
                }
                Either3::Second(Ok(header)) => match client.poll_body(header).await {
                    Ok(Event::Publish(e)) => {
                        info!("Received Message {:?}", e);
                        if e.topic == COMMAND_TOPIC {
                            if let Ok(str) = str::from_utf8(&e.message) {
                                if let Ok(int) = str::parse::<i8>(str) {
                                    LAST_COMMAND.signal(Command::MoveToPos(int));
                                } else {
                                    error!("Received invalid number: {:?}", e.message);
                                    break;
                                }
                            } else {
                                error!("Received invalid utf-8: {:?}", e.message);
                                break;
                            }
                        };
                    }
                    Ok(e) => info!("Received Event {:?}", e),
                    Err(e) => {
                        error!("Failed to poll body: {:?}", e);
                        break;
                    }
                },
                Either3::Third(pos) => {
                    let payload = format!(4; "{}", pos).unwrap();
                    let payload = Bytes::Borrowed(payload.as_bytes());
                    if let Err(e) = client.publish(&pub_options, payload).await {
                        error!("failed to publish: {:?}", e);
                        break;
                    } else {
                        info!("publised pos")
                    };
                }
            };
        }
        if let Err(e) = client.abort().with_timeout(Duration::from_secs(5)).await {
            error!("error aborting connection: {:?}", e);
            continue;
        };
    }
}
