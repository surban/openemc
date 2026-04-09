# OpenEMC board IO

Userspace interface for communicating with an [OpenEMC](https://github.com/surban/openemc)
embedded controller via the `/dev/openemc` device node exposed by the
OpenEMC Linux kernel driver.

## Board IO

The board IO interface provides message-based, bidirectional communication
between userspace and the board-specific firmware running on the OpenEMC.

Reads block until the firmware has data available. Writes block if the
firmware is not ready to accept data and automatically retry once the
firmware signals availability. 

## Board ioctl

The [`ioctl`](OpenEmc::ioctl) method provides a synchronous request/response
channel. A request of up to 32 bytes is sent to the firmware, which processes it
and returns a response (or an error). Calls are serialized; only one ioctl
can be in flight at a time.

When the firmware's board handler reports failure, the call returns an
[`std::io::Error`] whose source is an [`IoctlError`] containing
the error code set by the firmware.

## Example

```rust,no_run
use openemc_io::OpenEmc;
use std::io::{Read, Write};

let mut emc = OpenEmc::new().expect("cannot open /dev/openemc");

// Send a board ioctl request and receive the response.
let response = emc.ioctl(&[0x01, 0x02, 0x03]).expect("ioctl failed");
println!("response: {response:?}");

// Read and write board IO messages.
let mut buf = [0u8; 32];
let n = emc.read(&mut buf).expect("read failed");
emc.write(&buf[..n]).expect("write failed");
```

## License

Licensed under the MIT license. This only applies to the `openemc-io` crate, not the OpenEMC firmware.
