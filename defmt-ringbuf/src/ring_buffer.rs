//! Ring buffer.

/// Ring buffer access methods.
pub trait RingBuf {
    /// Write into the ring buffer.
    fn write(&mut self, data: &[u8]);

    /// Read from the ring buffer.
    ///
    /// Returns the number of bytes read and whether data was lost.
    fn read(&mut self, data: &mut [u8]) -> (usize, bool);
}

/// Ring buffer.
#[repr(C)]
pub struct RingBuffer<const SIZE: usize> {
    /// Signature.
    signature: u32,
    /// Read position.
    read_pos: usize,
    /// Write position.
    write_pos: usize,
    /// Unread data overwritten?
    overwritten: bool,
    /// Buffer.
    buf: [u8; SIZE],
}

impl<const SIZE: usize> RingBuffer<SIZE> {
    /// Signature for validity check.
    const SIGNATURE: u32 = 0xb0ffe300;

    /// Validates the ring buffer and clears it if it is invalid.
    fn validate(&mut self) {
        let valid = self.signature == Self::SIGNATURE
            && self.read_pos < self.buf.len()
            && self.write_pos < self.buf.len();

        if !valid {
            self.buf.fill(0);
            self.read_pos = 0;
            self.write_pos = 0;
            self.overwritten = false;
            self.signature = Self::SIGNATURE;
        }
    }
}

impl<const SIZE: usize> RingBuf for RingBuffer<SIZE> {
    fn write(&mut self, mut data: &[u8]) {
        self.validate();

        while !data.is_empty() {
            // Split data into part that fits remaining buffer.
            let to_end = self.buf.len() - self.write_pos;
            let (part, rest) = data.split_at(to_end.min(data.len()));
            data = rest;

            // Copy.
            let from = self.write_pos;
            let to = self.write_pos + part.len();
            self.buf[from..to].copy_from_slice(part);

            // Update read position if we overwrote unread data.
            if from < self.read_pos && to >= self.read_pos {
                self.overwritten = true;
                self.read_pos = to + 1;
                if self.read_pos >= self.buf.len() {
                    self.read_pos = 0;
                }
            }

            // Update write position.
            self.write_pos = to;
            if self.write_pos == self.buf.len() {
                self.write_pos = 0;
            }
        }
    }

    fn read(&mut self, data: &mut [u8]) -> (usize, bool) {
        self.validate();

        // Calculate available data.
        let avail = if self.read_pos > self.write_pos {
            self.buf.len() - self.read_pos
        } else {
            self.write_pos - self.read_pos
        };
        let n = avail.min(data.len());

        // Copy.
        let from = self.read_pos;
        let to = self.read_pos + n;
        data[..n].copy_from_slice(&self.buf[from..to]);

        // Update read position.
        self.read_pos = to;
        if self.read_pos == self.buf.len() {
            self.read_pos = 0;
        }

        // Update overwritten status.
        let overwritten = self.overwritten;
        self.overwritten = false;

        (n, overwritten)
    }
}
