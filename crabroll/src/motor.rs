use core::{iter::FusedIterator, num::NonZeroU32};

use super::LAST_COMMAND;
use crate::{CONFIRM_SIGNAL, CURRENT_POS, Command, DIR_TO_HOME, ERROR_SIGNAL, ErrorSeverity};

use defmt::{error, info};
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_time::{Duration, Instant, Timer};
use esp_bootloader_esp_idf::partitions::{
    self, DataPartitionSubType, PARTITION_TABLE_MAX_LEN, PartitionType,
};
use esp_hal::gpio::{Input, Output};
use esp_storage::FlashStorage;
use iter_step_gen::{Direction, Stepper, StepperError};
use sequential_storage::{
    cache::NoCache,
    map::{MapConfig, MapStorage},
};

const DEFAULT_TRAVEL_LIMIT: NonZeroU32 = NonZeroU32::new(2048).unwrap();
const MAX_VEL: NonZeroU32 = NonZeroU32::new(2048).unwrap();
const MAX_ACCEL: NonZeroU32 = NonZeroU32::new(225).unwrap();
const START_VEL: u32 = 64;

// storage consts
const TRAVEL_LIMIT_KEY: u8 = 0;

#[embassy_executor::task]
pub(crate) async fn motor_task(
    mut step_pin: Output<'static>,
    mut dir_pin: Output<'static>,
    endstop_pin: Input<'static>,
    mut flash: FlashStorage<'static>,
) {
    let mut pt_mem = [0u8; PARTITION_TABLE_MAX_LEN];
    let pt = partitions::read_partition_table(&mut flash, &mut pt_mem).unwrap();
    let nvs = pt
        .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
        .unwrap()
        .unwrap();
    let partition = nvs.as_embedded_storage(&mut flash);
    let mut flash = MapStorage::<u8, _, _>::new(
        BlockingAsync::new(partition),
        MapConfig::new(0x0000..0x6000),
        NoCache::new(),
    );

    let mut flash_buffer = [0u8; 4096];
    let travel_limit = match flash
        .fetch_item::<u32>(&mut flash_buffer, &TRAVEL_LIMIT_KEY)
        .await
    {
        Ok(Some(l)) => {
            CONFIRM_SIGNAL.signal(());
            NonZeroU32::new(l).unwrap()
        }
        Ok(None) => {
            match flash
                .store_item(
                    &mut flash_buffer,
                    &TRAVEL_LIMIT_KEY,
                    &DEFAULT_TRAVEL_LIMIT.get(),
                )
                .await
            {
                Ok(()) => {
                    CONFIRM_SIGNAL.signal(());
                }
                Err(_) => {
                    error!("Error storing item in flash");
                    ERROR_SIGNAL.signal(ErrorSeverity::Hard);
                }
            };
            DEFAULT_TRAVEL_LIMIT
        }
        Err(_) => {
            error!("Error getting item in flash");
            ERROR_SIGNAL.signal(ErrorSeverity::Hard);
            DEFAULT_TRAVEL_LIMIT
        }
    };

    let mut stepper = Stepper::new(travel_limit, MAX_VEL, MAX_ACCEL, START_VEL);
    execute_home(&mut step_pin, &mut dir_pin, &mut stepper, &endstop_pin).await;
    loop {
        match LAST_COMMAND.wait().await {
            Command::Home => {
                info!("homing");
                execute_home(&mut step_pin, &mut dir_pin, &mut stepper, &endstop_pin).await;
                CONFIRM_SIGNAL.signal(());
                info!("homed");
            }
            Command::StartJog(direction) => {
                info!("jogging in {} direction", direction);
                match execute_jog(&mut step_pin, &mut dir_pin, &mut stepper, direction).await {
                    Ok(_) => info!("jogged"),
                    Err(e) => {
                        info!("Error: {}", e);
                        ERROR_SIGNAL.signal(ErrorSeverity::Soft);
                    }
                };
            }
            Command::StopJog => (),
            Command::SetBottom => {
                if let Some(pos) = stepper.pos() {
                    info!("Setting current position as bottom");
                    let pos = NonZeroU32::new(pos).unwrap_or(NonZeroU32::MIN);
                    stepper.set_travel_limit(pos);
                    match flash
                        .store_item(&mut flash_buffer, &TRAVEL_LIMIT_KEY, &pos.get())
                        .await
                    {
                        Ok(()) => CONFIRM_SIGNAL.signal(()),
                        Err(_) => {
                            error!("Error storing item in flash");
                            ERROR_SIGNAL.signal(ErrorSeverity::Hard);
                        }
                    };
                } else {
                    info!("Attempted to set travel limit while unhomed");
                    ERROR_SIGNAL.signal(ErrorSeverity::Soft);
                }
            }
            Command::MoveToPos(percent) => {
                info!("moving to {}", percent);
                let pos = (percent as u32 * stepper.travel_limit().get()) / 100_u32;
                info!("moving to {}", pos);
                match execute_move(&mut step_pin, &mut dir_pin, &mut stepper, pos).await {
                    Ok(_) => info!("moved to pos"),
                    Err(e) => {
                        info!("Error: {}", e);
                        ERROR_SIGNAL.signal(ErrorSeverity::Soft);
                    }
                };
            }
        }
        CURRENT_POS.signal(if let Some(p) = stepper.pos() {
            ((p * 100_u32) / stepper.travel_limit())
                .try_into()
                .unwrap_or(100)
        } else {
            0
        });
    }
}

async fn execute_home<'a>(
    step_pin: &mut Output<'a>,
    dir_pin: &mut Output<'a>,
    stepper: &mut Stepper,
    endstop_pin: &Input<'a>,
) {
    dir_pin.set_level(*DIR_TO_HOME.read().await);
    let plan = stepper.homing_move(|| endstop_pin.is_low());
    execute_step_plan(step_pin, plan).await;
}

async fn execute_move<'a>(
    step_pin: &mut Output<'a>,
    dir_pin: &mut Output<'a>,
    stepper: &mut Stepper,
    target_pos: u32,
) -> Result<(), StepperError> {
    let (plan, dir) = stepper.planned_move(target_pos)?;
    let home_level = *DIR_TO_HOME.read().await;
    if dir == Direction::ToHome {
        dir_pin.set_level(home_level);
    } else {
        dir_pin.set_level(!home_level);
    }
    execute_step_plan(step_pin, plan).await;
    Ok(())
}

async fn execute_jog<'a>(
    step_pin: &mut Output<'a>,
    dir_pin: &mut Output<'a>,
    stepper: &mut Stepper,
    dir: Direction,
) -> Result<(), StepperError> {
    let plan = stepper.continuous_jog(
        || {
            !LAST_COMMAND
                .try_take()
                .is_some_and(|c| c == Command::StopJog)
        },
        dir,
    )?;
    let home_level = *DIR_TO_HOME.read().await;
    if dir == Direction::ToHome {
        dir_pin.set_level(home_level);
    } else {
        dir_pin.set_level(!home_level);
    }
    execute_step_plan(step_pin, plan.fuse()).await;
    Ok(())
}

async fn execute_step_plan<'a>(
    step_pin: &mut Output<'a>,
    plan: impl FusedIterator<Item = Duration>,
) {
    for delay in plan {
        let now = Instant::now();
        step_pin.set_high();
        Timer::after_nanos(100).await;
        step_pin.set_low();
        Timer::at(now.saturating_add(delay)).await;
    }
}
