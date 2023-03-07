//! CRC32 calculation functions.

use stm32f1xx_hal::crc::Crc;

/// Calculate the CRC32 of a slice of binary data.
pub fn crc32(crc: &mut Crc, mut data: &[u8]) -> u32 {
    crc.reset();

    loop {
        if data.len() >= 4 {
            let (part, rest) = data.split_at(4);
            crc.write(u32::from_ne_bytes(defmt::unwrap!(part.try_into())).reverse_bits());
            data = rest;
        } else if !data.is_empty() {
            let mut padded = [0; 4];
            padded[..data.len()].copy_from_slice(data);
            crc.write(u32::from_ne_bytes(padded).reverse_bits());
            break;
        } else {
            break;
        }
    }

    cortex_m::asm::nop();
    !crc.read().reverse_bits()
}
