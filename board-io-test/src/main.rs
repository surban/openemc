//! OpenEMC board IO test.
//!
//! For use with STM Nucleo-F103RB example board (`stm_nucleo_f103rb.rs`).
//!

use rand::Rng;
use std::{
    fs::File,
    io::{Error, Read, Write},
    os::fd::AsRawFd,
};

fn io_test(openemc: &mut File) {
    let mut test_data = vec![0u8; 32];
    rand::thread_rng().fill(test_data.as_mut_slice());

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
    rand::thread_rng().fill(test_data.as_mut_slice());

    let mut data = test_data.clone();
    let ret = unsafe { libc::ioctl(openemc.as_raw_fd(), 30, data.as_mut_ptr()) };
    assert_eq!(ret, 25, "ioctl failed");

    let expected: Vec<_> = test_data.iter().map(|v| v.wrapping_add(1)).collect();
    assert_eq!(&data[..25], &expected[..25], "ioctl data mismatch");

    let ret = unsafe { libc::ioctl(openemc.as_raw_fd(), 32, data.as_mut_ptr()) };
    assert_eq!(ret, -1, "ioctl should have failed");
    let errno = Error::last_os_error().raw_os_error().unwrap();
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
