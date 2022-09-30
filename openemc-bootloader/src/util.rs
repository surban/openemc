//! Debug functions.

use crate::board::Board;

/// Approximately delay execution by specified amount of milliseconds.
pub fn delay_ms(ms: u32) {
    let cycles = ms * (crate::boards::Chosen::CPU_CLOCK / 1000);
    cortex_m::asm::delay(cycles)
}
