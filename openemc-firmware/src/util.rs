//! Utility functions.

use core::mem::size_of;
use defmt::unwrap;

use crate::{watchman::Watchman, Delay};

/// Byte representation of an array of u64s.
pub fn array_from_u64<const D: usize, const F: usize>(data: &[u64; D]) -> [u8; F] {
    defmt::assert!(F == D * size_of::<u64>());

    let mut flat = [0; F];
    let mut pos = 0;
    for d in data {
        for f in d.to_le_bytes() {
            flat[pos] = f;
            pos += 1;
        }
    }
    flat
}

/// Convert byte representation to an array of u64s.
pub fn array_to_u64<const D: usize>(flat: &[u8]) -> [u64; D] {
    defmt::assert!(flat.len() == D * size_of::<u64>());

    let mut data = [0; D];
    for (i, d) in data.iter_mut().enumerate() {
        let b: [u8; size_of::<u64>()] =
            unwrap!(flat[i * size_of::<u64>()..(i + 1) * size_of::<u64>()].try_into());
        *d = u64::from_le_bytes(b);
    }
    data
}

/// Byte representation of an array of u16s.
pub fn array_from_u16<const D: usize, const F: usize>(data: &[u16; D]) -> [u8; F] {
    defmt::assert!(F == D * size_of::<u16>());

    let mut flat = [0; F];
    let mut pos = 0;
    for d in data {
        for f in d.to_le_bytes() {
            flat[pos] = f;
            pos += 1;
        }
    }
    flat
}

/// Convert byte representation to an array of u16s.
pub fn array_to_u16<const D: usize>(flat: &[u8]) -> [u16; D] {
    defmt::assert!(flat.len() == D * size_of::<u16>());

    let mut data = [0; D];
    for (i, d) in data.iter_mut().enumerate() {
        let b: [u8; size_of::<u16>()] =
            unwrap!(flat[i * size_of::<u16>()..(i + 1) * size_of::<u16>()].try_into());
        *d = u16::from_le_bytes(b);
    }
    data
}

/// Blink a LED.
#[allow(unused_variables, unused_mut)]
pub fn blink(mut set: impl FnMut(bool), delay: &mut Delay, mut watchman: &mut Watchman, times: usize) {
    #[cfg(feature = "debug-blink")]
    {
        use systick_monotonic::fugit::ExtU32;

        watchman.force_pet();
        set(false);
        delay.delay(1u32.secs());

        for _ in 0..times {
            watchman.force_pet();

            set(true);
            delay.delay(300u32.millis());
            set(false);
            delay.delay(300u32.millis());
        }

        delay.delay(700u32.millis());
        watchman.force_pet();
    }
}

/// Blinks the charging LED of a board.
#[macro_export]
macro_rules! blink_charging {
    ($board:expr, $delay:expr, $watchman:expr, $times:expr) => {
        blink(|v| $board.set_charging_led(v), &mut $delay, &mut $watchman, $times);
    };
}
