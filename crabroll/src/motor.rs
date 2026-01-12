use core::{iter::FusedIterator, num::NonZeroU32};

use super::LAST_COMMAND;
use crate::{CURRENT_POS, Command, DIR_TO_HOME, ERROR_SIGNAL};

use defmt::info;
use embassy_time::{Duration, Instant, Timer};
use esp_hal::gpio::{Input, Output};
use iter_step_gen::{Direction, Stepper, StepperError};

const TRAVEL_LIMIT: NonZeroU32 = NonZeroU32::new(2048).unwrap();
const MAX_VEL: NonZeroU32 = NonZeroU32::new(2048).unwrap();
const MAX_ACCEL: NonZeroU32 = NonZeroU32::new(225).unwrap();
const START_VEL: u32 = 64;

#[embassy_executor::task]
pub(crate) async fn motor_task(
    mut step_pin: Output<'static>,
    mut dir_pin: Output<'static>,
    endstop_pin: Input<'static>,
) {
    let mut stepper = Stepper::new(TRAVEL_LIMIT, MAX_VEL, MAX_ACCEL, START_VEL);
    execute_home(&mut step_pin, &mut dir_pin, &mut stepper, &endstop_pin).await;
    loop {
        match LAST_COMMAND.wait().await {
            Command::Home => {
                info!("homing");
                execute_home(&mut step_pin, &mut dir_pin, &mut stepper, &endstop_pin).await;
                info!("homed");
            }
            Command::StartJog(direction) => {
                info!("jogging in {} direction", direction);
                match execute_jog(&mut step_pin, &mut dir_pin, &mut stepper, direction).await {
                    Ok(_) => info!("jogged"),
                    Err(e) => {
                        info!("Error: {}", e);
                        ERROR_SIGNAL.signal(());
                    }
                };
            }
            Command::StopJog => (),
            Command::SetBottom => {
                if let Some(pos) = stepper.pos() {
                    info!("Setting current position as bottom");
                    let pos = NonZeroU32::new(pos).unwrap_or(NonZeroU32::MIN);
                    stepper.set_travel_limit(pos);
                } else {
                    info!("Attempted to set travel limit while unhomed");
                    ERROR_SIGNAL.signal(());
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
                        ERROR_SIGNAL.signal(());
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
