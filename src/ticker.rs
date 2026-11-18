//! Fixed-rate loop timing.

use std::time::{Duration, Instant};

use vexide::time::sleep_until;

/// Wakes on a fixed grid (t0 + period, t0 + 2*period, ...) instead of "work, then sleep".
/// That way the loop period doesn't stretch by however long the work took, and per-loop PID
/// gains see the same dt every iteration.
pub struct Ticker {
    period: Duration,
    next: Instant,
}

impl Ticker {
    pub fn new(period: Duration) -> Self {
        Self { period, next: Instant::now() + period }
    }

    /// Await the next tick. Never awaits in a busy loop and never allocates.
    pub async fn tick(&mut self) {
        sleep_until(self.next).await;
        self.next += self.period;

        // If we overran by a whole period (e.g. a slow controller write), re-anchor instead of
        // firing a burst of back-to-back catch-up ticks.
        let now = Instant::now();
        if self.next <= now {
            self.next = now + self.period;
        }
    }
}
