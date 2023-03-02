//! [`defmt`](https://github.com/knurling-rs/defmt) global logger into a persistent ring buffer.
//! To use this crate, link to it by importing it somewhere in your project.
//!
//! ```
//! // src/main.rs or src/bin/my-app.rs
//! use defmt_ringbuf as _;
//! ```
//!
//! Call [init] to initialize logging and [read] to read buffered log data.
//!
//! # Critical section implementation
//!
//! This crate uses [`critical-section`](https://github.com/rust-embedded/critical-section) to ensure only one thread
//! is writing to the buffer at a time. You must import a crate that provides a `critical-section` implementation
//! suitable for the current target. See the `critical-section` README for details.
//!
//! For example, for single-core privileged-mode Cortex-M targets, you can add the following to your Cargo.toml.
//!
//! ```toml
//! [dependencies]
//! cortex-m = { version = "0.7.6", features = ["critical-section-single-core"]}
//! ```

#![no_std]

use core::sync::atomic::{AtomicBool, Ordering};

mod ring_buffer;

pub use ring_buffer::{RingBuf, RingBuffer};

#[defmt::global_logger]
struct Logger;

/// Global logger lock.
static TAKEN: AtomicBool = AtomicBool::new(false);

/// Crticial section.
static mut CS_RESTORE: critical_section::RestoreState = critical_section::RestoreState::invalid();

/// Encoder.
static mut ENCODER: defmt::Encoder = defmt::Encoder::new();

/// Ring buffer.
static mut RING_BUFFER: Option<&'static mut dyn RingBuf> = None;

/// Callback when new log data is available.
static mut LOG_AVAILABLE: fn() = || ();

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        let restore = unsafe { critical_section::acquire() };
        if TAKEN.load(Ordering::Relaxed) {
            panic!("defmt logger taken reentrantly")
        }
        TAKEN.store(true, Ordering::Relaxed);
        unsafe { CS_RESTORE = restore };

        unsafe { ENCODER.start_frame(do_write) }
    }

    unsafe fn flush() {
        // Flush is a no-op.
    }

    unsafe fn release() {
        ENCODER.end_frame(do_write);

        TAKEN.store(false, Ordering::Relaxed);
        let restore = CS_RESTORE;
        critical_section::release(restore);
    }

    unsafe fn write(bytes: &[u8]) {
        ENCODER.write(bytes, do_write);
    }
}

fn do_write(data: &[u8]) {
    unsafe {
        if let Some(buffer) = RING_BUFFER.as_mut() {
            buffer.write(data);
            LOG_AVAILABLE();
        }
    }
}

/// Initializes logging to a ring buffer.
///
/// `ring_buffer` specifies the location of the log buffer.
/// It is not cleared if it contains vailid data from the previous boot.
///
/// `log_available` is called when new log messages are available.
///
/// This must be called exactly once.
/// Log messages received before initiailization are discarded.
pub unsafe fn init<const SIZE: usize>(ring_buffer: &'static mut RingBuffer<SIZE>, log_available: fn()) {
    defmt::assert!(RING_BUFFER.is_none());

    RING_BUFFER = Some(ring_buffer as &mut dyn RingBuf);
    LOG_AVAILABLE = log_available;
}

/// Reads and removes data from the log buffer.
///
/// Returns the number of bytes read and whether data was lost.
pub fn read(data: &mut [u8]) -> (usize, bool) {
    unsafe {
        critical_section::with(|_cs| {
            if let Some(buffer) = RING_BUFFER.as_mut() {
                buffer.read(data)
            } else {
                (0, false)
            }
        })
    }
}
