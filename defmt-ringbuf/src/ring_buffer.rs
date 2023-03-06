//! Ring buffer.

use core::{
    mem::MaybeUninit,
    ptr::{addr_of, addr_of_mut},
    sync::atomic::{AtomicBool, AtomicU32, AtomicU8, AtomicUsize, Ordering},
};

use cortex_m::Peripherals;

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
    /// Signature for validity check.
    signature: AtomicU32,
    /// Read position.
    read_pos: AtomicUsize,
    /// Write position.
    write_pos: AtomicUsize,
    /// Unread data overwritten?
    overwritten: AtomicBool,
    /// Buffer.
    buf: [AtomicU8; SIZE],
}

impl<const SIZE: usize> RingBuffer<SIZE> {
    /// Signature for validity check.
    const SIGNATURE: u32 = 0xb0ffe300;

    /// Initializes the ring buffer, keeping its data if it appears valid.
    pub fn init(uninit: &mut MaybeUninit<Self>) -> &mut Self {
        unsafe {
            let mut scb = Peripherals::steal().SCB;
            let ptr = uninit.as_mut_ptr();

            let signature = (addr_of!((*ptr).signature) as *const u32).read_volatile();
            let mut read_pos = (addr_of!((*ptr).read_pos) as *const usize).read_volatile();
            let mut write_pos = (addr_of!((*ptr).write_pos) as *const usize).read_volatile();
            let mut overwritten = (addr_of!((*ptr).overwritten) as *const u8).read_volatile();

            let valid = signature == Self::SIGNATURE
                && read_pos < SIZE
                && write_pos < SIZE
                && (overwritten == 0 || overwritten == 1);

            if !valid {
                addr_of_mut!((*ptr).signature).write_volatile(AtomicU32::new(0));
                scb.clean_dcache_by_ref(&(*ptr).signature);

                read_pos = 0;
                write_pos = 0;
                overwritten = 0;
            }

            for i in 0..SIZE {
                let b = if valid { (addr_of!((*ptr).buf[i]) as *const u8).read_volatile() } else { 0 };
                addr_of_mut!((*ptr).buf[i]).write_volatile(AtomicU8::new(b));
            }
            scb.clean_dcache_by_slice(&(*ptr).buf);

            addr_of_mut!((*ptr).read_pos).write_volatile(AtomicUsize::new(read_pos));
            scb.clean_dcache_by_ref(&(*ptr).read_pos);
            addr_of_mut!((*ptr).write_pos).write_volatile(AtomicUsize::new(write_pos));
            scb.clean_dcache_by_ref(&(*ptr).write_pos);
            addr_of_mut!((*ptr).overwritten).write_volatile(AtomicBool::new(overwritten != 0));
            scb.clean_dcache_by_ref(&(*ptr).overwritten);
            addr_of_mut!((*ptr).signature).write_volatile(AtomicU32::new(Self::SIGNATURE));
            scb.clean_dcache_by_ref(&(*ptr).signature);

            // SAFETY: all fields have been initialized with either default values or their previous contents.
            uninit.assume_init_mut()
        }
    }
}

impl<const SIZE: usize> RingBuf for RingBuffer<SIZE> {
    fn write(&mut self, mut data: &[u8]) {
        let mut scb = unsafe { Peripherals::steal().SCB };

        while !data.is_empty() {
            // Split data into part that fits remaining buffer.
            let write_pos = self.write_pos.load(Ordering::SeqCst);
            let to_end = SIZE - write_pos;
            let (part, rest) = data.split_at(to_end.min(data.len()));
            data = rest;

            // Calculate write boundaries.
            let from = write_pos;
            let to = write_pos + part.len();

            // Update read position if we overwrite unread data.
            let read_pos = self.read_pos.load(Ordering::SeqCst);
            if from < read_pos && to >= read_pos {
                self.overwritten.store(true, Ordering::SeqCst);

                let mut new_read_pos = to + 1;
                if new_read_pos == SIZE {
                    new_read_pos = 0;
                }
                self.read_pos.store(new_read_pos, Ordering::SeqCst);
                scb.clean_dcache_by_ref(&self.read_pos);
            }

            // Copy.
            for (dst, src) in self.buf[from..to].iter_mut().zip(part.iter()) {
                dst.store(*src, Ordering::SeqCst);
            }
            scb.clean_dcache_by_slice(&self.buf[from..to]);

            // Update write position.
            let new_write_pos = if to == SIZE { 0 } else { to };
            self.write_pos.store(new_write_pos, Ordering::SeqCst);
            scb.clean_dcache_by_ref(&self.write_pos);
        }
    }

    fn read(&mut self, data: &mut [u8]) -> (usize, bool) {
        let mut scb = unsafe { Peripherals::steal().SCB };

        let read_pos = self.read_pos.load(Ordering::SeqCst);
        let write_pos = self.write_pos.load(Ordering::SeqCst);
        let overwritten = self.overwritten.load(Ordering::SeqCst);

        // Calculate available data.
        let avail = if read_pos > write_pos { SIZE - read_pos } else { write_pos - read_pos };
        let n = avail.min(data.len());

        // Copy.
        let from = read_pos;
        let to = read_pos + n;
        for (dst, src) in data[..n].iter_mut().zip(self.buf[from..to].iter()) {
            *dst = src.load(Ordering::SeqCst);
        }

        // Update read position and overwritten status.
        let new_read_pos = if to == SIZE { 0 } else { to };
        self.read_pos.store(new_read_pos, Ordering::SeqCst);
        scb.clean_dcache_by_ref(&self.read_pos);
        self.overwritten.store(false, Ordering::SeqCst);
        scb.clean_dcache_by_ref(&self.overwritten);

        (n, overwritten)
    }
}
