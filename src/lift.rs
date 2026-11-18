//! Two-motor lift with a rotation sensor for auton position control.

use std::time::Instant;

use vexide::peripherals::DynamicPeripherals;
use vexide::prelude::RotationSensor;

use crate::config;
use crate::localization::Encoder;
use crate::motor_group::MotorGroup;
use crate::motion::MotionTimer;
use crate::pid::{Pid, Settle};
use crate::ticker::Ticker;

pub struct Lift {
    pub motors: MotorGroup<2>,
    rotation: RotationSensor,
    pid: Pid,
    /// Rotation-sensor degrees.
    target: f64,
}

impl Lift {
    pub fn new(p: &mut DynamicPeripherals) -> Self {
        let port = p
            .take_smart_port(config::LIFT_ROTATION_PORT)
            .expect("smart port is already in use");
        let mut rotation = RotationSensor::new(port, config::LIFT_ROTATION_DIR);
        // 0 = wherever the lift is at program start, so start every match with the lift all the way down.
        let _ = rotation.reset_position();
        let _ = rotation.set_data_interval(RotationSensor::MIN_DATA_INTERVAL);

        Self {
            motors: MotorGroup::new(p, config::LIFT_MOTORS, config::LIFT_GEARSET, config::LIFT_BRAKE),
            rotation,
            pid: Pid::new(config::LIFT_PID),
            target: 0.0,
        }
    }

}

#[expect(dead_code, reason = "auton motion stack; used again once autons come back")]
impl Lift {
    /// Raw volts (clamped), bypassing the PID. Also moves the PID target to the current
    /// position, so a later PID move starts from here instead of snapping back to an old target.
    pub fn set_voltage(&mut self, volts: f64) {
        if let Some(current) = self.rotation.position_deg() {
            self.target = current;
        }
        self.motors.set_voltage(volts.clamp(-12.0, 12.0));
    }

    /// PID the lift to `target` until it settles or `timeout_ms` runs out. Afterward the
    /// motors keep their last output (no brake), so the lift doesn't fall.
    pub async fn move_to(&mut self, target: f64, timeout_ms: u64, scale: f64) {
        self.pid.reset();
        self.target = target;

        let mut settle = Settle::new(self.pid.config());
        let timer = MotionTimer::start(timeout_ms);
        let mut ticker = Ticker::new(config::LOOP_PERIOD);

        loop {
            // One sensor read per tick (v1 read it twice).
            let Some(current) = self.rotation.position_deg() else {
                break; // sensor unplugged: bail out
            };

            let output = self.pid.calculate(self.target, current, scale);
            self.motors.set_voltage((output + config::LIFT_KG).clamp(-12.0, 12.0));

            let now = Instant::now();
            if settle.update(self.pid.error().abs(), now) || timer.expired(now) {
                break;
            }
            ticker.tick().await;
        }
    }
}
