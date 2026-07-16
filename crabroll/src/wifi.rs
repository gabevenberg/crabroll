use defmt::{error, info};
use embassy_net::Runner;
use embassy_time::Timer;
use esp_radio::wifi::{Interface, WifiController};

#[embassy_executor::task]
pub(crate) async fn connection(mut controller: WifiController<'static>) {
    info!("start connection task");
    loop {
        info!("about to connect");
        match controller.connect_async().await {
            Ok(info) => {
                info!("Wifi connected to {:?}", info);
                let info = controller.wait_for_disconnect_async().await.ok();
                info!("Disconnected: {:?}", info);
            }
            Err(e) => error!("Failed to connect to wifi: {:?}", e),
        }
        Timer::after_secs(5).await
    }
}

#[embassy_executor::task]
pub(crate) async fn net_task(mut runner: Runner<'static, Interface<'static>>) {
    runner.run().await
}
