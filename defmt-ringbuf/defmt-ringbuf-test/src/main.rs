#![no_std]
#![no_main]

use core::mem;
use cortex_m::{asm::nop, peripheral::SCB};
use cortex_m_rt::entry;

use defmt_ringbuf::{RingBuf, RingBuffer};
use defmt_rtt as _;
use panic_probe as _;
use stm32f1::stm32f103::Peripherals;

#[no_mangle]
#[used]
#[link_section = ".uninit.BUFFER"]
pub static mut BUFFER: mem::MaybeUninit<defmt_ringbuf::RingBuffer<4096>> = mem::MaybeUninit::uninit();

#[entry]
fn main() -> ! {
    let _dp = Peripherals::take().unwrap();

    defmt::info!("Init");
    let buffer = unsafe { RingBuffer::init(&mut BUFFER) };

    let mut data = [0; 128];
    let (n, lost) = buffer.read(&mut data);
    let data = &data[..n];
    defmt::info!("Buffer contents (lost={}): {}", lost, data);

    let mut data = [0; 255];
    for (i, d) in data.iter_mut().enumerate() {
        *d = i as u8;
    }
    buffer.write(&data);
    defmt::info!("Wrote {} bytes to buffer", data.len());

    defmt::info!("Done");
    for _ in 0..1_000_000 {
        nop();
    }
    SCB::sys_reset();
}
