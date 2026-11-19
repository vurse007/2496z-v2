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

// ---------------------------------------------------------------- controller screen

/// How often the pose readout writes a line. The controller only takes one write per ~50 ms
/// anyway, and the readout alternates lines 2 and 3, so each line refreshes every 100 ms.
pub const TELEMETRY_PERIOD: Duration = Duration::from_millis(50);

// ---------------------------------------------------------------- odometry
//
// Each tracking wheel is optional. `None` for the vertical pod = use the drive motors instead.
// See the table in `localization/mod.rs` for all four setups.

/// One tracking wheel (a rotation sensor on an unpowered omni).
#[derive(Clone, Copy)]
pub struct PodConfig {
    pub port: u8,
    /// Must make forward (vertical pod) / rightward (horizontal pod) travel read positive.
    pub direction: Direction,
    pub wheel_diameter_in: f64,
    /// Distance from the tracking center in inches. Vertical pod: + = left of center.
    /// Horizontal pod: + = forward of center. Sanity check: spinning in place should leave
    /// x/y near 0. If it drifts, the sign or value is wrong.
    pub offset_in: f64,
}

// Both pods are set, so odometry uses them and not the drive motors.
// PLACEHOLDERS: set port / direction / wheel size / offset to match the robot. Check:
//   - push straight forward: the vertical pod reads positive (else flip direction)
//   - push straight right:   the horizontal pod reads positive (else flip direction)
//   - spin in place: x/y on the controller should stay near 0 (else fix the offsets)
// Port 5 / Forward / 2 in for the vertical pod are v1's placeholder values. Port 9 is just a
// free port.

/// Forward/backward tracking wheel.
pub const VERTICAL_POD: Option<PodConfig> = Some(PodConfig {
    port: 5,
    direction: Forward,
    wheel_diameter_in: 2.0,
    offset_in: 0.0,
});

/// Sideways tracking wheel.
pub const HORIZONTAL_POD: Option<PodConfig> = Some(PodConfig {
    port: 9,
    direction: Forward,
    wheel_diameter_in: 2.0,
    offset_in: 0.0,
});

/// Only used by the motor fallback (`VERTICAL_POD = None`), so these don't matter right now.
/// PLACEHOLDERS: rampage-v3's values, not measured on this robot. Measure before trusting x/y.
pub const DRIVE_WHEEL_DIAMETER_IN: f64 = 2.75;
/// Wheel revolutions per motor revolution.
pub const DRIVE_GEAR_RATIO: f64 = 0.8;

/// Inches of travel per degree of wheel rotation: pi * d / 360.
pub const fn inches_per_deg(wheel_diameter_in: f64) -> f64 {
    std::f64::consts::PI * wheel_diameter_in / 360.0
}

/// Precomputed at compile time, so the fallback costs one multiply per tick.
pub const DRIVE_IN_PER_MOTOR_DEG: f64 = inches_per_deg(DRIVE_WHEEL_DIAMETER_IN) * DRIVE_GEAR_RATIO;

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
