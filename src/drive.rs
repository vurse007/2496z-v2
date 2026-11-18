//! Six-motor tank drivetrain and the driver-control stick mapping.

use vexide::peripherals::DynamicPeripherals;

use crate::config;
use crate::localization::Encoder;
use crate::motor_group::MotorGroup;
use crate::pid::Pid;

pub struct Drivetrain {
    pub left: MotorGroup<3>,
    pub right: MotorGroup<3>,
    /// Heading that `straight()` holds, which is the target of the last turn. Holding this
    /// instead of wherever the robot drifted keeps errors from stacking up over an auton.
    pub target_heading: f64,
    pub heading_pid: Pid,
    pub turn_pid: Pid,
    pub drive_pid: Pid,
    curve: StickCurve,
}

impl Drivetrain {
    pub fn new(p: &mut DynamicPeripherals) -> Self {
        Self {
            left: MotorGroup::new(p, config::DRIVE_LEFT, config::DRIVE_GEARSET, config::DRIVE_BRAKE),
            right: MotorGroup::new(p, config::DRIVE_RIGHT, config::DRIVE_GEARSET, config::DRIVE_BRAKE),
            target_heading: 0.0,
            heading_pid: Pid::new(config::HEADING_PID),
            turn_pid: Pid::new(config::TURN_PID),
            drive_pid: Pid::new(config::DRIVE_PID),
            curve: StickCurve::new(config::DRIVE_CURVE_T),
        }
    }

    /// Split arcade from raw stick values (-127..=127). Same output as v1. When saturated, it
    /// multiplies by `12 / max` instead of dividing by `max`, which can differ in the last bit
    /// of the f64. That's far below the 1 mV the motor can resolve.
    pub fn split_arcade(&mut self, forward_raw: i8, turn_raw: i8) {
        let forward = self.curve.get(forward_raw);
        let turn = self.curve.get(turn_raw);

        let left = forward + turn;
        let right = forward - turn;

        // If either side would exceed full power, scale both down so the left/right ratio is
        // kept. Only then do we need a division (one, shared by both sides).
        let max = left.abs().max(right.abs());
        let to_volts = if max > 1.0 { 12.0 / max } else { 12.0 };

        self.left.set_voltage(left * to_volts);
        self.right.set_voltage(right * to_volts);
    }

    /// Volts, clamped to -12..=12.
    pub fn move_raw(&mut self, left: f64, right: f64) {
        self.left.set_voltage(left.clamp(-12.0, 12.0));
        self.right.set_voltage(right.clamp(-12.0, 12.0));
    }

    pub fn stop(&mut self) {
        self.left.brake();
        self.right.brake();
    }

    pub fn resync(&mut self) {
        self.left.resync();
        self.right.resync();
    }

    /// Distance driven in motor-encoder degrees: average of both sides.
    pub fn distance_deg(&self) -> Option<f64> {
        Some((self.left.position_deg()? + self.right.position_deg()?) * 0.5)
    }
}

/// v1's exponential drive curve, precomputed for every possible stick value.
///
/// The controller only ever reports integers -127..=127, and the curve is odd
/// (`f(-x) == -f(x)`), so a 128-entry table covers every input. That turns two `exp()` calls
/// per stick per loop into one array read. Entries are computed with v1's exact formula,
/// so the output is bit-identical.
struct StickCurve([f64; 128]);

impl StickCurve {
    fn new(t: f64) -> Self {
        Self(std::array::from_fn(|i| drive_curve(i as f64 / 127.0, t)))
    }

    fn get(&self, raw: i8) -> f64 {
        // min(127): the SDK never reports -128, but v1 clamped the input, so do the same.
        let magnitude = self.0[usize::from(raw.unsigned_abs().min(127))];
        if raw < 0 { -magnitude } else { magnitude }
    }
}

/// y = (e^(-t/10) + e^((|x|-127)/10) * (1 - e^(-t/10))) * x with x in -127..=127, scaled to -1..=1.
/// Only runs at startup to fill the table.
fn drive_curve(input: f64, t: f64) -> f64 {
    let x = input.clamp(-1.0, 1.0) * 127.0;
    let a = (-t / 10.0).exp();
    let y = (a + ((x.abs() - 127.0) / 10.0).exp() * (1.0 - a)) * x;
    y / 127.0
}
