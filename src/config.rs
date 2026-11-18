//! Every tunable number on the robot lives here: ports, directions, gearsets,
//! driver voltages, PID gains, and loop timing. Values are copied 1:1 from v1.
//!
//! Everything is `const`, so the compiler folds it straight into the code that
//! uses it. There's nothing to compute or look up at runtime.

use std::time::Duration;

use vexide::prelude::{Direction, Gearset};
use vexide::smart::motor::BrakeMode;

use crate::pid::PidConfig;

use Direction::{Forward, Reverse};

/// `(smart port, direction)`. Reverse flips that one motor.
pub type MotorPort = (u8, Direction);

// ---------------------------------------------------------------- timing

/// Every control loop (driver and auton motions) runs on this fixed period.
/// The per-loop PID gains below assume it, so retune them if you change it.
pub const LOOP_PERIOD: Duration = Duration::from_millis(10);

// ---------------------------------------------------------------- drivetrain

pub const DRIVE_LEFT: [MotorPort; 3] = [(4, Reverse), (10, Reverse), (21, Reverse)];
pub const DRIVE_RIGHT: [MotorPort; 3] = [(3, Forward), (14, Forward), (20, Forward)];
pub const DRIVE_GEARSET: Gearset = Gearset::Blue;
/// What `Drivetrain::stop()` does at the end of every auton motion.
pub const DRIVE_BRAKE: BrakeMode = BrakeMode::Coast;

/// Exponential stick curve strength (0 = steepest, bigger = closer to linear).
pub const DRIVE_CURVE_T: f64 = 2.8;

// ---------------------------------------------------------------- sensors

pub const IMU_PORT: u8 = 7;

// ---------------------------------------------------------------- lift

pub const LIFT_MOTORS: [MotorPort; 2] = [(13, Forward), (1, Forward)];
pub const LIFT_GEARSET: Gearset = Gearset::Green;
pub const LIFT_BRAKE: BrakeMode = BrakeMode::Coast;
/// Lift angle sensor. Direction must make "lift up" read positive.
pub const LIFT_ROTATION_PORT: u8 = 8;
pub const LIFT_ROTATION_DIR: Direction = Reverse;
/// Constant volts added to the lift PID output to fight gravity (feedforward).
pub const LIFT_KG: f64 = 0.0;

// ---------------------------------------------------------------- claw / toggles / pneumatics

pub const CLAW: [MotorPort; 1] = [(11, Forward)];
pub const CLAW_GEARSET: Gearset = Gearset::Blue;
pub const CLAW_BRAKE: BrakeMode = BrakeMode::Coast;

pub const TOGGLE_MOTORS: [MotorPort; 2] = [(6, Forward), (16, Reverse)];
pub const TOGGLE_GEARSET: Gearset = Gearset::Green;
pub const TOGGLE_BRAKE: BrakeMode = BrakeMode::Coast;

/// Three-wire ports: 1 = A ... 8 = H.
pub const FLIPPER_ADI_PORT: u8 = 1; // A
pub const PARK_ADI_PORT: u8 = 8; // H

// ---------------------------------------------------------------- driver control voltages
//
// Bindings (see `Robot::driver`):
//   left stick Y / right stick X  split arcade
//   L1                            lift up
//   L2                            lift down
//   R1                            claw open + lift down (slower)
//   R2                            claw close
//   Y (hold)                      toggles
//   D-pad left (press)            flipper piston toggle
//   D-pad right (press)           park piston toggle

pub const LIFT_UP_V: f64 = 12.0;
pub const LIFT_DOWN_V: f64 = -8.0;
/// Lift voltage while R1 is held. R1 overrides L1/L2 for the lift, like in v1.
pub const LIFT_DOWN_WITH_CLAW_V: f64 = -7.0;
pub const CLAW_OPEN_V: f64 = 12.0;
pub const CLAW_CLOSE_V: f64 = -7.0;
pub const TOGGLES_V: f64 = 12.0;

// ---------------------------------------------------------------- PID gains
//
// Gains are per loop, not per second:
//   kp: V per unit of error, ki: V per (error summed each loop),
//   kd: V per (change in error since the last loop), slew: max V change per loop.

/// Heading hold during `straight()`.
pub const HEADING_PID: PidConfig = PidConfig {
    general: (0.3, 0.0, 0.0),
    refined: (0.0, 0.0, 0.0),
    refined_range: 0.0,
    settle_range: 0.0,
    slew: 12.0,
    integral_threshold: 0.0,
    max_integral: 0.0,
    deadband: 0.0,
    settle_ms: 0,
};

/// Point turns and swings (degrees).
pub const TURN_PID: PidConfig = PidConfig {
    general: (0.6, 0.0, 3.73),
    refined: (0.6, 0.0, 3.73),
    refined_range: 3.0,
    settle_range: 1.0,
    slew: 3.0,
    integral_threshold: 5.0,
    max_integral: 100.0,
    deadband: 0.0,
    settle_ms: 150,
};

/// `straight()` (motor-encoder degrees).
pub const DRIVE_PID: PidConfig = PidConfig {
    general: (0.3, 0.0, 3.21),
    refined: (0.05, 0.0, 0.205),
    refined_range: 25.0,
    settle_range: 5.0,
    slew: 2.0,
    integral_threshold: 7.0,
    max_integral: 100.0,
    deadband: 3.0,
    settle_ms: 200,
};

/// Lift position (rotation-sensor degrees).
pub const LIFT_PID: PidConfig = PidConfig {
    general: (0.2, 0.0, 0.0),
    refined: (0.2, 0.0, 0.0),
    refined_range: 0.0,
    settle_range: 2.0,
    slew: 12.0,
    integral_threshold: 0.0,
    max_integral: 0.0,
    deadband: 0.0,
    settle_ms: 200,
};
