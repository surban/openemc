//! Offline ring buffer reader.

use byteorder::{ByteOrder, ReadBytesExt};
use displaydoc::Display;
use thiserror::Error;

pub use byteorder::{BE, LE};

/// Error reading offline ring buffer.
#[derive(Display, Error, Debug)]
pub enum Error {
    /// Data too short.
    TooShort,
    /// Invalid signature: 0x{0:08x}
    InvalidSignature(u32),
    /// Invalid read position: {0}
    InvalidReadPos(u32),
    /// Invalid write position: {0}
    InvalidWritePos(u32),
    /// Invalid overwritten state: 0x{0:02x}
    InvalidOverwritten(u8),
}

impl From<std::io::Error> for Error {
    fn from(_: std::io::Error) -> Self {
        Error::TooShort
    }
}

const SIGNATURE: u32 = 0xb0ffe300;

/// Read offline ring buffer.
pub fn read<O: ByteOrder>(mut ring_buffer: &[u8]) -> Result<(Vec<u8>, bool), Error> {
    // Dissect structure.
    let signature = ring_buffer.read_u32::<O>()?;
    let mut read_pos = ring_buffer.read_u32::<O>()? as usize;
    let write_pos = ring_buffer.read_u32::<O>()? as usize;
    let overwritten = ring_buffer.read_u32::<O>()? as u8;
    let buf = ring_buffer;
    let size = buf.len();

    // Validate state.
    if signature != SIGNATURE {
        return Err(Error::InvalidSignature(signature));
    }
    if read_pos >= size {
        return Err(Error::InvalidReadPos(read_pos as u32));
    }
    if write_pos >= size {
        return Err(Error::InvalidWritePos(write_pos as u32));
    }
    let overwritten = match overwritten {
        0 => false,
        1 => true,
        other => return Err(Error::InvalidOverwritten(other)),
    };

    // Extract data.
    let mut data = Vec::new();
    loop {
        let n = if read_pos > write_pos { size - read_pos } else { write_pos - read_pos };
        if n == 0 {
            break;
        }

        let from = read_pos;
        let to = read_pos + n;
        data.extend_from_slice(&buf[from..to]);

        read_pos = if to == size { 0 } else { to };
    }

    Ok((data, overwritten))
}
