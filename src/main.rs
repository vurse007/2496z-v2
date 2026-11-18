//! 2496Z v2. Module map:
//!
//! - `config`        every port, direction, gearset, voltage, and PID gain
//! - `robot`         `Robot` struct, driver-control loop, competition dispatch
//! - `motion`        straight / turn / swing motions
//! - `drive`, `lift` subsystems
//! - `motor_group`   fixed-size motor groups with write-on-change
//! - `localization`  `Pose`, `Encoder`, `Localizer` (odometry + MCL plug in here)
//! - `pid`, `ticker`, `display`  control building blocks

mod config;
mod display;
mod drive;
mod lift;
mod localization;
#[expect(dead_code, reason = "auton motion stack; used again once autons come back")]
mod motion;
mod motor_group;
mod pid;
mod robot;
mod ticker;

use vexide::peripherals::DynamicPeripherals;
use vexide::prelude::*;

use robot::Robot;

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let mut peripherals = DynamicPeripherals::new(peripherals);
    let robot = Robot::new(&mut peripherals).await;
    robot.compete().await;
}
