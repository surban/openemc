//! OpenEMC board IO test.
//!
//! For use with STM Nucleo-F103RB example board (`stm_nucleo_f103rb.rs`).
//!

use nix::ioctl_readwrite;
use rand::prelude::*;
use std::{
    fs::File,
    io::{Read, Write},
    os::fd::AsRawFd,
};

const OPENEMC_MAX_DATA_SIZE: usize = 32;
const OPENEMC_IOC_MAGIC: u8 = 0xEC;

#[repr(C)]
struct OpenemcIoctlData {
    len: u8,
    data: [u8; OPENEMC_MAX_DATA_SIZE],
}

ioctl_readwrite!(openemc_board_ioctl, OPENEMC_IOC_MAGIC, 0, OpenemcIoctlData);

fn io_test(openemc: &mut File) {
    let mut test_data = vec![0u8; 32];
    rand::rng().fill(test_data.as_mut_slice());

    println!("Writing data");
    assert_eq!(openemc.write(&test_data).unwrap(), test_data.len());

    println!("Reading back data stream");
    test_data.truncate(31);
    while !test_data.is_empty() {
        let mut buf = vec![0u8; 32];
        let n = openemc.read(&mut buf).unwrap();
        buf.truncate(n);

        assert_eq!(buf, test_data);
        test_data.truncate(test_data.len() - 1);
    }

    println!("IO test done");
}

fn ioctl_test(openemc: &mut File) {
    println!("ioctl test");

    let mut test_data = vec![0u8; 32];
    rand::rng().fill(test_data.as_mut_slice());

    let mut ioctl_data = OpenemcIoctlData { len: 30, data: [0u8; OPENEMC_MAX_DATA_SIZE] };
    ioctl_data.data[..30].copy_from_slice(&test_data[..30]);

    let ret = unsafe { openemc_board_ioctl(openemc.as_raw_fd(), &mut ioctl_data) };
    assert!(ret.is_ok(), "ioctl failed");
    assert_eq!(ret.unwrap(), 25, "ioctl return value mismatch");
    assert_eq!(ioctl_data.len, 25, "ioctl response length mismatch");

    let expected: Vec<_> = test_data.iter().map(|v| v.wrapping_add(1)).collect();
    assert_eq!(&ioctl_data.data[..25], &expected[..25], "ioctl data mismatch");

    ioctl_data.len = 32;
    ioctl_data.data.copy_from_slice(&test_data);
    let ret = unsafe { openemc_board_ioctl(openemc.as_raw_fd(), &mut ioctl_data) };
    assert!(ret.is_err(), "ioctl should have failed");
    let errno = ret.unwrap_err() as i32;
    assert_eq!(errno, 10, "wrong errno");

    println!("ioctl test done");
}

fn main() {
    println!("Opening /dev/openemc");
    let mut openemc =
        File::options().read(true).write(true).open("/dev/openemc").expect("cannot open /dev/openemc");

    io_test(&mut openemc);
    ioctl_test(&mut openemc);

    println!("Done");
}
