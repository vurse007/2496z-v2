//! Fixed-size motor group that skips redundant writes.

use vexide::peripherals::DynamicPeripherals;
use vexide::prelude::{Gearset, Motor};
use vexide::smart::motor::{BrakeMode, MotorControl};

use crate::config::MotorPort;
use crate::localization::Encoder;

/// `N` motors in a plain array, so there's no `Vec` and no heap use, even at startup.
///
/// **Write-on-change.** Every `Motor::set_target` costs three SDK calls (gearset read, brake-mode
/// write, voltage write). vexide remembers each motor's last successful target, so we compare
/// against it and skip the write when nothing would change. Voltages are compared in whole
/// millivolts, which is what the SDK actually receives (`(volts * 1000) as i32`), so stick noise
/// below 1 mV never causes a write.
///
/// A failed write (motor unplugged) leaves vexide's stored target unchanged, so the write is
/// retried automatically every loop until the motor comes back.
pub struct MotorGroup<const N: usize> {
    motors: [Motor; N],
    brake_mode: BrakeMode,
    /// Set by `resync()`. Forces the next command to be written even if it looks unchanged.
    force_write: bool,
}

impl<const N: usize> MotorGroup<N> {
    pub fn new(
        p: &mut DynamicPeripherals,
        ports: [MotorPort; N],
        gearset: Gearset,
        brake_mode: BrakeMode,
    ) -> Self {
        let motors = ports.map(|(port, dir)| {
            let port = p.take_smart_port(port).expect("smart port is already in use");
            Motor::new(port, gearset, dir)
        });
        Self { motors, brake_mode, force_write: true }
    }

    /// Volts, -12..=12. Callers clamp; vexide passes the value straight through.
    pub fn set_voltage(&mut self, volts: f64) {
        self.command(MotorControl::Voltage(volts));
    }

    /// Stop with this group's configured brake mode.
    pub fn brake(&mut self) {
        self.command(MotorControl::Brake(self.brake_mode));
    }

    /// Re-send the next command no matter what. Called at the start of each competition mode
    /// because VEXos stops all motors on disable, so our cached targets may be stale.
    pub fn resync(&mut self) {
        self.force_write = true;
    }

    fn command(&mut self, target: MotorControl) {
        for m in &mut self.motors {
            if self.force_write || !same_output(m.target(), target) {
                let _ = m.set_target(target); // Err = unplugged; retried next loop (see above)
            }
        }
        self.force_write = false;
    }
}

/// True if the motor would receive exactly the same command.
fn same_output(a: MotorControl, b: MotorControl) -> bool {
    match (a, b) {
        (MotorControl::Voltage(a), MotorControl::Voltage(b)) => millivolts(a) == millivolts(b),
        _ => a == b,
    }
}

/// Matches vexide's own conversion in `Motor::set_target`.
fn millivolts(volts: f64) -> i32 {
    (volts * 1000.0) as i32
}

/// Average encoder position of the group in degrees, over the motors that could be read.
impl<const N: usize> Encoder for MotorGroup<N> {
    fn position_deg(&self) -> Option<f64> {
        let mut sum = 0.0;
        let mut count = 0usize;
        for m in &self.motors {
            if let Ok(p) = m.position() {
                sum += p.as_degrees();
                count += 1;
            }
        }
        // count is at most N, so this division is a few cycles. A lookup table isn't worth it.
        (count > 0).then(|| sum / count as f64)
    }
}
