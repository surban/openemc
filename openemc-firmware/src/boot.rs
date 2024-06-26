//! Boot.

use core::{
    mem::MaybeUninit,
    ptr,
    ptr::{addr_of, addr_of_mut},
    slice,
    sync::atomic::{AtomicBool, Ordering},
};
use cortex_m::peripheral::{scb::Exception, SCB};
use stm32f1::stm32f103::Peripherals;
use stm32f1xx_hal::{backup_domain::BackupDomain, crc::Crc, prelude::_stm32_hal_gpio_GpioExt};

use crate::{backup::BackupReg, board::Board, bootloader_max_size, crc::crc32, ThisBoard};
use openemc_shared::{
    boot::{BootInfo, BootReason, ResetStatus},
    flash,
};

/// Boot reason extension.
pub trait BootReasonExt {
    /// Sets the boot reason.
    fn set(&self, bkp: &mut BackupDomain);

    /// Gets the boot reason.
    fn get(bkp: &BackupDomain) -> u16;
}

impl BootReasonExt for BootReason {
    /// Sets the boot reason.
    fn set(&self, bkp: &mut BackupDomain) {
        BackupReg::BootReason.set(bkp, *self as _);
    }

    /// Gets the boot reason.
    fn get(bkp: &BackupDomain) -> u16 {
        BackupReg::BootReason.get(bkp)
    }
}

/// Boot info block.
///
/// Initialized by bootloader.
#[no_mangle]
#[used]
#[link_section = ".boot_info"]
pub static mut BOOT_INFO: MaybeUninit<BootInfo> = MaybeUninit::uninit();

/// Standalone boot info block.
///
/// Used when firmware is running without bootloader.
static mut STANDALONE_BOOT_INFO: BootInfo = BootInfo {
    signature: BootInfo::SIGNATURE,
    emc_model: 0x01,
    board_model: ptr::null(),
    board_model_len: 0,
    bootloader_version: ptr::null(),
    bootloader_version_len: 0,
    boot_reason: 0,               // Obtained from backup registers.
    reset_status: ResetStatus(0), // Obtained from reset status register.
    start_reason: 0,
    powered_on: false,
    id: 0,
    reserved: BootInfo::RESERVED,
    board_data_len: 0,
    board_data: BootInfo::EMPTY_BOARD_DATA,
};

pub trait BootInfoExt {
    /// Initialize boot info.
    unsafe fn init(bkp: &BackupDomain);

    /// Get the boot information.
    fn get() -> &'static BootInfo;

    /// Returns whether boot information is from the bootloader.
    fn is_from_bootloader() -> bool;

    /// Board model.
    fn board_model(&self) -> &[u8];

    /// Bootloader version.
    fn bootloader_version(&self) -> Option<&[u8]>;
}

/// Whether boot information has been initialized.
static BOOT_INFO_INITIALIZED: AtomicBool = AtomicBool::new(false);

impl BootInfoExt for BootInfo {
    unsafe fn init(bkp: &BackupDomain) {
        defmt::assert!(!BOOT_INFO_INITIALIZED.load(Ordering::SeqCst));

        // Make sure BOOT_INFO is read from memory.
        let ptr = BOOT_INFO.as_mut_ptr();
        let powered_on_ptr = addr_of_mut!((*ptr).powered_on) as *mut u8;
        let powered_on = powered_on_ptr.read_volatile();
        if powered_on != 0 && powered_on != 1 {
            powered_on_ptr.write_volatile(0);
        }
        ptr.read_volatile();

        if !Self::is_from_bootloader() {
            let dp = Peripherals::steal();

            // Get reset status.
            STANDALONE_BOOT_INFO.reset_status =
                ResetStatus::from_rcc_pwr(dp.RCC.csr.read().bits(), dp.PWR.csr.read().bits());
            dp.RCC.csr.modify(|_, w| w.rmvf().clear());
            dp.RCC.csr.modify(|_, w| w.rmvf().clear_bit());
            dp.PWR.cr.modify(|_, w| w.cwuf().set_bit());
            dp.PWR.cr.modify(|_, w| w.cwuf().clear_bit());

            // Get boot reason.
            STANDALONE_BOOT_INFO.boot_reason = BootReason::get(bkp);
        }

        BOOT_INFO_INITIALIZED.store(true, Ordering::SeqCst);
    }

    fn get() -> &'static Self {
        defmt::assert!(BOOT_INFO_INITIALIZED.load(Ordering::SeqCst));

        if Self::is_from_bootloader() {
            // Safety: BOOT_INFO has been verified during init()
            unsafe { BOOT_INFO.assume_init_ref() }
        } else {
            // Safety: STANDALONE_BOOT_INFO is only modified during init()
            unsafe { &*ptr::addr_of!(STANDALONE_BOOT_INFO) }
        }
    }

    fn is_from_bootloader() -> bool {
        unsafe {
            let ptr = BOOT_INFO.as_ptr();
            addr_of!((*ptr).signature).read_volatile() == Self::SIGNATURE
        }
    }

    fn board_model(&self) -> &[u8] {
        if self.board_model.is_null() {
            ThisBoard::model()
        } else {
            unsafe { slice::from_raw_parts(self.board_model, self.board_model_len as _) }
        }
    }

    fn bootloader_version(&self) -> Option<&[u8]> {
        if self.bootloader_version.is_null() {
            None
        } else {
            Some(unsafe { slice::from_raw_parts(self.bootloader_version, self.bootloader_version_len as _) })
        }
    }
}

/// Initialize system.
pub fn init() {
    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    let dp = unsafe { Peripherals::steal() };

    // Disable deep sleep.
    cp.SCB.clear_sleepdeep();
    cp.SCB.clear_sleeponexit();

    // Apply workaround to make logging work when device is sleeping.
    match (cfg!(feature = "defmt-rtt"), option_env!("DEFMT_LOG")) {
        (false, _) | (true, Some("off") | None) => {
            dp.DBGMCU
                .cr
                .modify(|_, w| w.dbg_sleep().clear_bit().dbg_stop().clear_bit().dbg_standby().clear_bit());
        }
        _ => {
            dp.DBGMCU.cr.modify(|_, w| w.dbg_sleep().set_bit().dbg_stop().set_bit().dbg_standby().set_bit());
            dp.RCC.ahbenr.modify(|_, w| w.dma1en().enabled());
        }
    }

    // Make sure backup domain is not in reset.
    dp.RCC.bdcr.modify(|_, w| w.bdrst().disabled());
}

/// Switches to the bootloader without reset.
pub fn start_bootloader() -> ! {
    const BOOTLOADER_ADDR: usize = 0x08000000;

    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    cortex_m::interrupt::disable();

    defmt::warn!("starting bootloader");

    // Disable and clear SysTick interrupt.
    cp.SYST.disable_interrupt();
    cp.SYST.disable_counter();
    SCB::clear_pendst();

    // Disable and clear all interrupts.
    for icer in cp.NVIC.icer.iter() {
        unsafe { icer.write(0xffffffff) };
    }
    for icpr in cp.NVIC.icpr.iter() {
        unsafe { icpr.write(0xffffffff) };
    }

    // Disable fault handlers.
    cp.SCB.disable(Exception::BusFault);
    cp.SCB.disable(Exception::UsageFault);
    cp.SCB.disable(Exception::MemoryManagement);

    unsafe {
        let cp = cortex_m::Peripherals::steal();
        cp.SCB.vtor.write(BOOTLOADER_ADDR as u32);
        cortex_m::asm::bootload(BOOTLOADER_ADDR as *const u32)
    }
}

/// Calculates the CRC32 checksum of the bootloader.
pub fn bootloader_crc32(crc: &mut Crc) -> u32 {
    let data = unsafe { slice::from_raw_parts(flash::START as *const u8, bootloader_max_size()) };
    crc32(crc, data)
}

/// Powers off system by entering standby mode after reset.
pub fn power_off(bkp: &mut BackupDomain) -> ! {
    defmt::warn!("resetting for power off");
    BootReason::PowerOff.set(bkp);
    SCB::sys_reset();
}

/// Restart system by powering it off and then on again.
pub fn restart(bkp: &mut BackupDomain, reason: BootReason) -> ! {
    defmt::warn!("restarting with boot reason {:?}", reason);
    reason.set(bkp);
    SCB::sys_reset();
}

/// Reset the EMC.
pub fn reset(bkp: &mut BackupDomain) -> ! {
    defmt::warn!("resetting");
    BootReason::Reset.set(bkp);
    SCB::sys_reset();
}

/// Enters standby mode.
pub fn enter_standby() -> ! {
    defmt::info!("preparing to enter standby mode");

    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    let dp = unsafe { Peripherals::steal() };

    // Check if WKUP pin is low.
    let mut gpioa = unsafe { dp.GPIOA.split_without_reset() };
    let wkup = gpioa.pa0.into_floating_input(&mut gpioa.crl);
    if wkup.is_high() {
        // Device should stay awake, convert shutdown into reset.
        defmt::info!("WKUP pin is high, turning shutdown into reset");
        SCB::sys_reset();
    }

    // Disable and clear SysTick interrupt.
    cp.SYST.disable_interrupt();
    cp.SYST.disable_counter();
    SCB::clear_pendst();

    // Disable and clear all interrupts.
    for icer in cp.NVIC.icer.iter() {
        unsafe { icer.write(0xffffffff) };
    }
    for icpr in cp.NVIC.icpr.iter() {
        unsafe { icpr.write(0xffffffff) };
    }

    // Enable standby mode and wake up using WKUP pin.
    dp.PWR.csr.modify(|_, w| w.ewup().set_bit());
    dp.PWR.cr.modify(|_, w| w.lpds().set_bit().cwuf().set_bit().pdds().standby_mode());
    cp.SCB.set_sleepdeep();

    // Enter standby mode.
    defmt::warn!("entering standby mode");
    loop {
        rtic::export::wfi()
    }
}
