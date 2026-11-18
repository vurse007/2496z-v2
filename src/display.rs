//! Live PID error on the controller screen during auton, written without heap allocation.

use std::fmt::Write;
use std::time::Duration;

use vex_sdk::V5_ControllerId;
use vexide::controller::ControllerConnection;
use vexide::prelude::Controller;
use vexide::time::sleep;

/// v1 padded the text to the full 19-column width so leftovers from longer numbers get wiped.
const PAD_TO: usize = Controller::MAX_COLUMNS;

/// Best-effort write of `"<label> err: <error>"` to line 1. If the controller screen is still
/// busy with the previous write (~50 ms), it's skipped, so it never slows the loop down.
/// Returns `false` only when the screen was busy.
pub fn show_error(controller: &Controller, label: &str, error: f64) -> bool {
    if controller.connection() == ControllerConnection::Offline {
        return true; // nothing to wait for, same as v1
    }

    let mut line = Line::default();
    let _ = write!(line, "{label} err: {error:.2}");
    while line.len < PAD_TO {
        let _ = line.write_char(' ');
    }

    // Why not `Controller::try_set_text`? It builds a heap `CString` on every call, even when
    // the screen is busy, which means ~100 allocations/s inside motion loops. This is the same
    // SDK call with a stack buffer instead. Line 1, column 1 match v1's `try_set_text(.., 1, 1)`.
    //
    // SAFETY: `line.buf` is NUL-terminated (`Line` always keeps a trailing 0) and outlives the
    // call. The SDK copies the text before returning, the same thing vexide relies on when it
    // drops its `CString` right after this call.
    let id = V5_ControllerId::from(controller.id());
    unsafe { vex_sdk::vexControllerTextSet(u32::from(id.0), 1, 1, line.buf.as_ptr()) == 1 }
}

/// Like `show_error`, but waits (non-blocking, 10 ms at a time) until the write goes through.
/// Used once per motion for the final settled error.
pub async fn show_final_error(controller: &Controller, label: &str, error: f64) {
    while !show_error(controller, label, error) {
        sleep(Duration::from_millis(10)).await;
    }
}

/// Fixed-capacity, NUL-terminated text buffer. Text that doesn't fit is cut off.
#[derive(Default)]
struct Line {
    buf: [u8; 32],
    len: usize,
}

impl Write for Line {
    fn write_str(&mut self, s: &str) -> std::fmt::Result {
        let room = self.buf.len() - 1 - self.len; // keep the last byte as the NUL terminator
        let n = s.len().min(room);
        self.buf[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
        self.len += n;
        Ok(())
    }
}
