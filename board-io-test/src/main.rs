//! OpenEMC board IO test.
//!
//! For use with STM Nucleo-F103RB example board (`stm_nucleo_f103rb.rs`).
//!

use openemc_io::{OpenEmc, OpenEmcIoctlError};
use rand::prelude::*;
use std::io::{Read, Write};

fn io_test(openemc: &mut OpenEmc) {
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

fn ioctl_test(openemc: &mut OpenEmc) {
    println!("ioctl test");

    let mut test_data = vec![0u8; 32];
    rand::rng().fill(test_data.as_mut_slice());

    let result = openemc.ioctl(&test_data[..30]).unwrap();
    assert_eq!(result.len(), 25, "ioctl response length mismatch");

    let expected: Vec<_> = test_data.iter().map(|v| v.wrapping_add(1)).collect();
    assert_eq!(&result[..25], &expected[..25], "ioctl data mismatch");

    let result = openemc.ioctl(&test_data);
    assert!(result.is_err(), "ioctl should have failed");
    let err = result.unwrap_err();
    let ioctl_err = err.get_ref().unwrap().downcast_ref::<OpenEmcIoctlError>().unwrap();
    assert_eq!(ioctl_err.0, 10, "wrong errno");

    println!("ioctl test done");
}

fn main() {
    println!("Opening /dev/openemc");
    let mut openemc = OpenEmc::new().expect("cannot open /dev/openemc");

    io_test(&mut openemc);
    ioctl_test(&mut openemc);

    println!("Done");
}
