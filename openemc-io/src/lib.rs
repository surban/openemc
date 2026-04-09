#![doc = include_str!("../README.md")]

use rustix::ioctl::{self, opcode, Opcode};
use std::{
    fmt,
    fs::File,
    io,
    os::fd::{AsFd, AsRawFd, BorrowedFd, FromRawFd, IntoRawFd, OwnedFd},
};

/// Error returned by the firmware's board ioctl handler.
///
/// Contains the error code set by the firmware. This is wrapped
/// in an [`std::io::Error`] when returned from [`OpenEmc::ioctl`];
/// use [`std::io::Error::get_ref`] to access it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IoctlError(pub u8);

impl fmt::Display for IoctlError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "OpenEMC board ioctl error {}", self.0)
    }
}

impl std::error::Error for IoctlError {}

impl From<IoctlError> for io::Error {
    fn from(e: IoctlError) -> Self {
        io::Error::other(e)
    }
}

/// Ioctl magic byte used by the OpenEMC kernel driver.
pub const OPENEMC_IOC_MAGIC: u8 = 0xEC;

#[repr(C)]
struct OpenEmcIoctlData {
    len: u8,
    data: [u8; Self::MAX_DATA_SIZE],
}

impl OpenEmcIoctlData {
    const MAX_DATA_SIZE: usize = 32;
    const OPCODE: Opcode = opcode::read_write::<Self>(OPENEMC_IOC_MAGIC, 0);
}

/// Handle to the OpenEMC board IO device (`/dev/openemc`).
///
/// Provides a message-based read/write interface (via [`std::io::Read`] and
/// [`std::io::Write`]) and a synchronous request/response channel
/// (via [`ioctl`](Self::ioctl)) for communicating with the board-specific
/// firmware running on the OpenEMC embedded controller.
#[derive(Debug)]
pub struct OpenEmc(File);

impl OpenEmc {
    /// Maximum ioctl request and response size in bytes.
    pub const MAX_IOCTL_SIZE: usize = OpenEmcIoctlData::MAX_DATA_SIZE;

    /// Required minimum read buffer size in bytes.
    pub const READ_BUF_SIZE: usize = OpenEmcIoctlData::MAX_DATA_SIZE - 1;

    /// Maximum board IO write message size in bytes.
    pub const MAX_WRITE_SIZE: usize = OpenEmcIoctlData::MAX_DATA_SIZE;

    /// Opens the OpenEMC board IO device.
    pub fn new() -> io::Result<Self> {
        Ok(Self(File::options().read(true).write(true).open("/dev/openemc")?))
    }

    /// Sends an ioctl request to the firmware and returns the response.
    ///
    /// The request is forwarded to the firmware's board ioctl handler, which
    /// processes it synchronously and produces a response or an error.
    ///
    /// # Panics
    ///
    /// Panics if `req` is longer than [`Self::MAX_IOCTL_SIZE`] bytes.
    pub fn ioctl(&self, req: &[u8]) -> io::Result<Vec<u8>> {
        assert!(req.len() <= OpenEmcIoctlData::MAX_DATA_SIZE, "OpenEMC ioctl request too big");

        let mut ioc = OpenEmcIoctlData { len: req.len() as u8, data: [0; OpenEmcIoctlData::MAX_DATA_SIZE] };
        ioc.data[..req.len()].copy_from_slice(req);

        unsafe { ioctl::ioctl(&self.0, ioctl::Updater::<{ OpenEmcIoctlData::OPCODE }, _>::new(&mut ioc)) }
            .map_err(|e| IoctlError(e.raw_os_error() as u8))?;

        Ok(ioc.data[..ioc.len as usize].to_vec())
    }
}

impl io::Read for OpenEmc {
    /// Reads a single board IO message.
    ///
    /// Blocks until the firmware has data available.
    /// The buffer must be at least [`Self::READ_BUF_SIZE`] bytes long.
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.0.read(buf)
    }
}

impl io::Write for OpenEmc {
    /// Sends a board IO message of up to [`Self::MAX_WRITE_SIZE`] bytes to the firmware.
    ///
    /// Blocks if the firmware is not ready to accept data.
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.0.write(buf)
    }

    /// This is a no-op.
    fn flush(&mut self) -> io::Result<()> {
        self.0.flush()
    }
}

impl AsFd for OpenEmc {
    fn as_fd(&self) -> BorrowedFd<'_> {
        self.0.as_fd()
    }
}

impl AsRawFd for OpenEmc {
    fn as_raw_fd(&self) -> i32 {
        self.0.as_raw_fd()
    }
}

impl IntoRawFd for OpenEmc {
    fn into_raw_fd(self) -> i32 {
        self.0.into_raw_fd()
    }
}

impl FromRawFd for OpenEmc {
    unsafe fn from_raw_fd(fd: i32) -> Self {
        Self(File::from_raw_fd(fd))
    }
}

impl From<OwnedFd> for OpenEmc {
    fn from(fd: OwnedFd) -> Self {
        Self(File::from(fd))
    }
}

impl From<OpenEmc> for OwnedFd {
    fn from(emc: OpenEmc) -> Self {
        emc.0.into()
    }
}
