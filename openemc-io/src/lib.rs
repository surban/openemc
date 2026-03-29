//! OpenEMC board IO.

use rustix::ioctl::{self, opcode, Opcode};
use std::{fmt, fs::File, io};

/// OpenEMC ioctl error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OpenEmcIoctlError(pub u8);

impl fmt::Display for OpenEmcIoctlError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "OpenEMC board ioctl error {}", self.0)
    }
}

impl std::error::Error for OpenEmcIoctlError {}

impl From<OpenEmcIoctlError> for io::Error {
    fn from(e: OpenEmcIoctlError) -> Self {
        io::Error::other(e)
    }
}

const OPENEMC_IOC_MAGIC: u8 = 0xEC;

#[repr(C)]
struct OpenEmcIoctlData {
    len: u8,
    data: [u8; Self::MAX_DATA_SIZE],
}

impl OpenEmcIoctlData {
    const MAX_DATA_SIZE: usize = 32;
    const OPCODE: Opcode = opcode::read_write::<Self>(OPENEMC_IOC_MAGIC, 0);
}

/// OpenEMC access.
#[derive(Debug)]
pub struct OpenEmc(File);

impl OpenEmc {
    /// Maximum request size.
    pub const MAX_REQ_SIZE: usize = OpenEmcIoctlData::MAX_DATA_SIZE;

    /// Opens the OpenEMC board IO device.
    pub fn new() -> io::Result<Self> {
        Ok(Self(File::options().read(true).write(true).open("/dev/openemc")?))
    }

    /// Performs an OpenEMC board ioctl.
    ///
    /// `req` length must not exceed [`Self::MAX_REQ_SIZE`].
    pub fn ioctl(&self, req: &[u8]) -> io::Result<Vec<u8>> {
        assert!(req.len() <= OpenEmcIoctlData::MAX_DATA_SIZE, "OpenEMC ioctl request too big");

        let mut ioc = OpenEmcIoctlData { len: req.len() as u8, data: [0; OpenEmcIoctlData::MAX_DATA_SIZE] };
        ioc.data[..req.len()].copy_from_slice(req);

        unsafe { ioctl::ioctl(&self.0, ioctl::Updater::<{ OpenEmcIoctlData::OPCODE }, _>::new(&mut ioc)) }
            .map_err(|e| OpenEmcIoctlError(e.raw_os_error() as u8))?;

        Ok(ioc.data[..ioc.len as usize].to_vec())
    }
}

impl io::Read for OpenEmc {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.0.read(buf)
    }
}

impl io::Write for OpenEmc {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.0.write(buf)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.0.flush()
    }
}
