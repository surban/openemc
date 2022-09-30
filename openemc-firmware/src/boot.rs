//! Boot.

use core::{mem::MaybeUninit, ptr, slice};
use cortex_m::peripheral::{scb::Exception, SCB};
use openemc_shared::{BootInfo, ResetStatus};
use stm32f1::stm32f103::Peripherals;
use stm32f1xx_hal::backup_domain::BackupDomain;

use crate::{backup::BackupReg, board::Board, ThisBoard};

/// Boot reason.
#[allow(dead_code)]
#[repr(u16)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum BootReason {
    /// Unknown boot reason.
    /// Most likely caused by loss of power to backup domain.
    Unknown = 0x0000,
    /// Surprise reboot while in bootloader.
    SurpriseInBootloader = 0xb000,
    /// Surprise reboot while in user program.
    SurpriseInUser = 0xb001,
    /// Boot caused by invalid user program.
    InvalidUserProgram = 0xb003,
    /// Boot caused by power on event.
    PowerOn = 0xb010,
    /// Power off system and go to standby mode.
    PowerOff = 0xb011,
    /// Power off system and then restart it.
    Restart = 0xb012,
    /// Reset of EMC.
    Reset = 0xb013,
    /// Restart into bootloader.
    StartBootloader = 0xb014,
    /// Boot caused by factory reset.
    FactoryReset = 0xb020,
    /// Timeout of user controlled watchdog.
    WatchdogTimeout = 0xb030,
}

impl BootReason {
    /// Sets the boot reason.
    pub fn set(&self, bkp: &mut BackupDomain) {
        BackupReg::BootReason.set(bkp, *self as _);
    }

    /// Gets the boot reason.
    pub fn get(bkp: &BackupDomain) -> u16 {
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
    i2c_addr: ThisBoard::STANDALONE_I2C_ADDR,
    i2c_remap: ThisBoard::STANDALONE_I2C_REMAP,
    irq_pin: ThisBoard::STANDALONE_IRQ_PIN,
    irq_pin_cfg: ThisBoard::STANDALONE_IRQ_PIN_CFG,
    boot_reason: 0,               // Obtained from backup registers.
    reset_status: ResetStatus(0), // Obtained from reset status register.
    start_reason: 0,
    id: 0,
    reserved: BootInfo::RESERVED,
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

impl BootInfoExt for BootInfo {
    unsafe fn init(bkp: &BackupDomain) {
        let dp = Peripherals::steal();

        let bi = BOOT_INFO.assume_init_ref();
        if bi.signature != Self::SIGNATURE {
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
    }

    fn get() -> &'static Self {
        let bi = unsafe { BOOT_INFO.assume_init_ref() };
        if bi.signature == Self::SIGNATURE {
            bi
        } else {
            unsafe { &STANDALONE_BOOT_INFO }
        }
    }

    fn is_from_bootloader() -> bool {
        let bi = unsafe { BOOT_INFO.assume_init_ref() };
        bi.signature == Self::SIGNATURE
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
    match option_env!("DEFMT_LOG") {
        Some("off") | None => {
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

/// Powers off system and enter standby mode after reset.
pub fn power_off(bkp: &mut BackupDomain) -> ! {
    BootReason::PowerOff.set(bkp);
    SCB::sys_reset();
}

/// Restart system by powering it off and then on again.
pub fn restart(bkp: &mut BackupDomain) -> ! {
    BootReason::Restart.set(bkp);
    SCB::sys_reset();
}

/// Reset the EMC.
pub fn reset(bkp: &mut BackupDomain) -> ! {
    BootReason::Reset.set(bkp);
    SCB::sys_reset();
}

/// Enters standby mode.
pub fn enter_standby() -> ! {
    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    let dp = unsafe { Peripherals::steal() };

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

    // Enable standby mode and waking up using WKUP pin.
    dp.PWR.csr.modify(|_, w| w.ewup().set_bit());
    dp.PWR.cr.modify(|_, w| w.lpds().set_bit().cwuf().set_bit().pdds().standby_mode());
    cp.SCB.set_sleepdeep();

    // Enter standby mode.
    loop {
        rtic::export::wfi()
    }
}
