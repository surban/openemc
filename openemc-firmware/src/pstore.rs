//! Persistent platform store.

/// Persistent platform store access interface.
pub struct Pstore<'a> {
    /// Data buffer.
    pub data: &'a mut [u8],
    /// Address for next IO in buffer.
    pub io_address: usize,
    /// Size of next IO.
    pub io_size: usize,
}

impl<'a> Pstore<'a> {
    /// Creates a new instance of the access interface.
    pub fn new(data: &'a mut [u8]) -> Self {
        Self { data, io_address: 0, io_size: 1 }
    }

    /// Read into buffer.
    pub fn read(&mut self, buf: &mut [u8]) -> usize {
        let avail = self.data.len().saturating_sub(self.io_address);
        let len = buf.len().min(self.io_size).min(avail);

        buf[..len].copy_from_slice(&self.data[self.io_address..(self.io_address + len)]);

        self.io_address += len;
        len
    }

    /// Write from buffer.
    pub fn write(&mut self, buf: &[u8]) -> usize {
        let avail = self.data.len().saturating_sub(self.io_address);
        let len = buf.len().min(self.io_size).min(avail);

        self.data[self.io_address..(self.io_address + len)].copy_from_slice(&buf[..len]);

        self.io_address += len;
        len
    }
}
