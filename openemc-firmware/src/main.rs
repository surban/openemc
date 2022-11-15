//
// OpenEMC firmware for embedded controllers
// Copyright (C) 2022 Sebastian Urban <surban@surban.net>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//

//! OpenEMC Firmware.

#![no_std]
#![no_main]

mod adc;
mod backup;
mod board;
mod boards;
mod boot;
mod i2c_reg_slave;
mod i2c_slave;
mod irq;
mod pio;
mod pwm;
mod reg;
mod rtc;
mod util;
mod watchman;

use defmt_rtt as _;
use panic_probe as _;
use stm32f1xx_hal as _;

use core::mem::size_of;
use cortex_m::peripheral::NVIC;
use defmt::unwrap;
use stm32f1::stm32f103::Interrupt;
use stm32f1xx_hal::{
    adc::Adc,
    backup_domain::BackupDomain,
    gpio::{Alternate, OpenDrain, Pin, CRH, CRL},
    pac::{ADC1, I2C1},
    prelude::*,
    watchdog::IndependentWatchdog,
};
use systick_monotonic::*;

use crate::{
    adc::{AdcBuf, AdcInputs},
    backup::BackupReg,
    board::Board,
    boot::{BootInfoExt, BootReason},
    i2c_reg_slave::{Event, I2CRegSlave},
    i2c_slave::I2cSlave,
    irq::IrqState,
    pio::MaskedGpio,
    pwm::PwmTimer,
    rtc::{ClockSrc, Rtc},
    util::{array_from_u16, array_from_u64, array_to_u16, array_to_u64},
    watchman::Watchman,
};
use openemc_shared::BootInfo;

/// OpenEMC firmware version.
pub static VERSION: &[u8] = env!("CARGO_PKG_VERSION").as_bytes();

/// Firmware copyright.
static COPYRIGHT: &[u8] = b"(c) 2022 Sebastian Urban <surban@surban.net> license: GNU GPL version 3";

/// MFD cells.
static MFD_CELLS: &[&[u8]] = &[
    b"openemc,openemc_adc",
    b"openemc,openemc_gpio",
    b"openemc,openemc_pinctrl",
    b"openemc,openemc_power",
    b"openemc,openemc_rtc",
    b"openemc,openemc_wdt",
];

/// Supported EMC models.
static EMC_MODELS: &[u8] = &[0x01, 0xd1];

/// Device id when firmware started via bootloader.
const ID_WITH_BOOTLOADER: u8 = 0xfb;

/// Device id when firmware running standalone.
const ID_STANDALONE: u8 = 0xf0;

/// I2C transfer buffer size.
pub const I2C_BUFFER_SIZE: usize = 32;

/// I2C register slave that may have remapped pins.
#[allow(clippy::type_complexity)]
pub enum RemappableI2CRegSlave {
    Remapped(
        I2CRegSlave<
            I2C1,
            (Pin<Alternate<OpenDrain>, CRH, 'B', 8>, Pin<Alternate<OpenDrain>, CRH, 'B', 9>),
            I2C_BUFFER_SIZE,
        >,
    ),
    Normal(
        I2CRegSlave<
            I2C1,
            (Pin<Alternate<OpenDrain>, CRL, 'B', 6>, Pin<Alternate<OpenDrain>, CRL, 'B', 7>),
            I2C_BUFFER_SIZE,
        >,
    ),
}

impl RemappableI2CRegSlave {
    /// Gets the next I2C register slave event event.
    pub fn event(&mut self) -> nb::Result<Event<I2C_BUFFER_SIZE>, i2c_reg_slave::Error> {
        match self {
            Self::Remapped(i2c) => i2c.event(),
            Self::Normal(i2c) => i2c.event(),
        }
    }
}

/// Instant in time.
pub type Instant = systick_monotonic::fugit::Instant<u64, 1, 100>;

/// Time duration.
pub type Duration = systick_monotonic::fugit::Duration<u64, 1, 100>;

/// Board.
pub use boards::Chosen as ThisBoard;

#[rtic::app(device = stm32f1::stm32f103, peripherals = true, dispatchers = [SPI1, SPI2, SPI3])]
mod app {
    use super::*;

    /// System timer.
    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<100>;

    /// Shared resources.
    #[shared]
    struct Shared {
        /// If true, idle task starts bootloader.
        start_bootloader: bool,
        /// User GPIO.
        ugpio: MaskedGpio<{ board::PORTS }>,
        /// IRQ state.
        irq: IrqState<{ board::PORTS }>,
        /// Backup domain.
        bkp: BackupDomain,
        /// Watchdog manager.
        watchman: Watchman,
        /// Real-time clock.
        rtc: Rtc,
        /// Result of latest AD conversion.
        adc_buf: Option<AdcBuf>,
        /// PWM timers.
        pwm_timers: [PwmTimer; 4],
        /// Board.
        board: ThisBoard,
    }

    /// Exclusive resources.
    #[local]
    struct Local {
        /// Offset in copyright string.
        copyright_offset: usize,
        /// MFD cell index.
        mfd_cell_index: usize,
        /// I2C slave.
        i2c_reg_slave: RemappableI2CRegSlave,
        /// AD converter.
        adc: Adc<ADC1>,
        /// AD converter inputs.
        adc_inp: AdcInputs,
        /// Index of PWM timer.
        pwm_timer_index: u8,
        /// Index of PWM channel.
        pwm_channel_index: u8,
    }

    /// Initialization (entry point).
    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("OpenEMC version {:a}", VERSION);
        defmt::info!("{:a}", COPYRIGHT);

        boot::init();

        // Create HAL objects.
        let rcc = cx.device.RCC.constrain();
        let mut bkp = rcc.bkp.constrain(cx.device.BKP, &mut cx.device.PWR);
        let mut flash = cx.device.FLASH.constrain();
        let mut afio = cx.device.AFIO.constrain();
        let clocks = rcc.cfgr.use_hse(8.MHz()).freeze(&mut flash.acr);
        let mono = Systick::new(cx.core.SYST, clocks.sysclk().to_Hz());
        let mut delay = cx.device.TIM1.delay_ms(&clocks);
        let adc = Adc::adc1(cx.device.ADC1, clocks);
        let adc_inp = AdcInputs::new();
        let _gpioa = cx.device.GPIOA.split();
        let mut gpiob = cx.device.GPIOB.split();
        let _gpioc = cx.device.GPIOC.split();
        let _gpiod = cx.device.GPIOD.split();

        // Initialize backup registers.
        BackupReg::init(&mut bkp);

        // Get boot information.
        unsafe { BootInfo::init(&bkp) };
        let bi = BootInfo::get();

        let mut board = ThisBoard::new(if BootInfo::is_from_bootloader() { Some(&bi.board_data) } else { None });

        // Print boot information.
        if !BootInfo::is_from_bootloader() {
            defmt::warn!("firmware is running without bootloader");
        }
        defmt::info!("EMC model:      0x{:02x}", bi.emc_model);
        defmt::info!("board model:    {:a}", bi.board_model());
        defmt::info!("I2C address:    0x{:02x} (pins remapped: {:?})", bi.i2c_addr, bi.i2c_remap);
        defmt::info!("IRQ pin:        {} (mode: 0b{:04b})", bi.irq_pin, bi.irq_pin_cfg);
        defmt::info!("boot reason:    0x{:04x}", bi.boot_reason);
        defmt::info!("reset status:   0x{:02x}", bi.reset_status.0);
        defmt::info!("start reason:   0x{:02x}", bi.start_reason);

        // Verify compatibility.
        if BootInfo::is_from_bootloader() {
            if !EMC_MODELS.contains(&bi.emc_model) {
                defmt::panic!("unsupported EMC model");
            }
            if !board.is_supported(bi.board_model()) {
                defmt::panic!("unsupported board model");
            }
        }

        // Handle boot reason.
        BootReason::SurpriseInUser.set(&mut bkp);
        if (!BootInfo::is_from_bootloader()
            && bi.boot_reason != BootReason::PowerOn as _
            && bi.boot_reason != BootReason::Reset as _)
            || bi.boot_reason == BootReason::Restart as _
        {
            // Make sure system is powered off for at least one second.
            delay.delay(1u32.secs());
        }
        if bi.boot_reason == BootReason::PowerOff as _ {
            BootReason::PowerOn.set(&mut bkp);
            board.shutdown();
        }

        // Start watchdog and its manager.
        let dog = IndependentWatchdog::new(cx.device.IWDG);
        let watchman = Watchman::new(dog, 120u64.secs(), true);
        unwrap!(watchdog_petter::spawn());

        // Initialize RTC.
        let mut rtc = Rtc::new(cx.device.RTC, &mut bkp);
        if let Some(prescaler) = rtc.clock_src().and_then(|src| src.prescaler()) {
            let _ = rtc.set_prescalar(prescaler - Rtc::PRESCALER_NEG_OFFSET);
        }
        unwrap!(check_rtc_src::spawn_after(5u64.secs()));

        // Print RTC information.
        defmt::info!("RTC source:     {}", rtc.clock_src());
        defmt::info!("RTC ready:      {}", rtc.is_ready());
        unwrap!(print_rtc_info::spawn());

        // Initializes board.
        board.init();

        // Configure user GPIO.
        let mut usable = [0xffff; board::PORTS];
        board.limit_usable(&mut usable);
        if bi.i2c_remap {
            usable[1] &= !(1 << 8 | 1 << 9);
        } else {
            usable[1] &= !(1 << 6 | 1 << 7);
        }
        usable[bi.irq_pin as usize / 16] &= !(1 << (bi.irq_pin % 16));
        let ugpio = MaskedGpio::new(usable);

        // Initialize PWM timers.
        let pwm_timers = [
            PwmTimer::new(pwm::Timer::Timer1, &clocks),
            PwmTimer::new(pwm::Timer::Timer2, &clocks),
            PwmTimer::new(pwm::Timer::Timer3, &clocks),
            PwmTimer::new(pwm::Timer::Timer4, &clocks),
        ];

        // Configure IRQ pin.
        let irq = IrqState::new(bi.irq_pin, bi.irq_pin_cfg);
        unsafe { irq::unmask_exti() };

        // Enable I2C register slave.
        let i2c_reg_slave = match BootInfo::get().i2c_remap {
            true => {
                let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
                let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);
                let mut i2c =
                    I2cSlave::i2c1(cx.device.I2C1, (scl, sda), &mut afio.mapr, BootInfo::get().i2c_addr, clocks);
                i2c.listen_buffer();
                i2c.listen_error();
                i2c.listen_event();
                RemappableI2CRegSlave::Remapped(I2CRegSlave::<_, _, I2C_BUFFER_SIZE>::new(i2c))
            }
            false => {
                let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
                let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
                let mut i2c =
                    I2cSlave::i2c1(cx.device.I2C1, (scl, sda), &mut afio.mapr, BootInfo::get().i2c_addr, clocks);
                i2c.listen_buffer();
                i2c.listen_error();
                i2c.listen_event();
                RemappableI2CRegSlave::Normal(I2CRegSlave::<_, _, I2C_BUFFER_SIZE>::new(i2c))
            }
        };
        unsafe { NVIC::unmask(Interrupt::I2C1_ER) };
        unsafe { NVIC::unmask(Interrupt::I2C1_EV) };

        defmt::debug!("init done");
        (
            Shared { start_bootloader: false, irq, ugpio, bkp, watchman, rtc, adc_buf: None, pwm_timers, board },
            Local {
                copyright_offset: 0,
                mfd_cell_index: 0,
                i2c_reg_slave,
                adc,
                adc_inp,
                pwm_timer_index: 0,
                pwm_channel_index: 0,
            },
            init::Monotonics(mono),
        )
    }

    /// Idle task.
    #[idle(shared = [start_bootloader])]
    fn idle(mut cx: idle::Context) -> ! {
        while !cx.shared.start_bootloader.lock(|sb| *sb) {
            // Sleep and wait for interrupt.
            rtic::export::wfi()
        }

        defmt::info!("starting bootloader");
        boot::start_bootloader();
    }

    /// Pets the independent hardware watchdog.
    #[task(shared = [watchman, bkp], priority = 3)]
    fn watchdog_petter(mut cx: watchdog_petter::Context) {
        if !cx.shared.watchman.lock(|watchman| watchman.pet_hardware_watchdog()) {
            cx.shared.bkp.lock(|bkp| BootReason::WatchdogTimeout.set(bkp));
        }
        unwrap!(watchdog_petter::spawn_after(Watchman::pet_interval()));
    }

    /// Power off system.
    #[task(shared = [bkp])]
    fn power_off(mut cx: power_off::Context) {
        defmt::info!("requesting power off");
        cx.shared.bkp.lock(|bkp| boot::power_off(bkp));
    }

    /// Power off system and then restart it.
    #[task(shared = [bkp])]
    fn power_restart(mut cx: power_restart::Context) {
        defmt::info!("requesting restart");
        cx.shared.bkp.lock(|bkp| boot::restart(bkp));
    }

    /// Reads ADC values and stores them in the ADC buffer.
    #[task(local = [adc, adc_inp], shared = [adc_buf], priority = 1)]
    fn adc_convert(mut cx: adc_convert::Context) {
        let adc_convert::LocalResources { adc, adc_inp } = cx.local;
        let buf = AdcBuf::acquire(adc, adc_inp);
        cx.shared.adc_buf.lock(|adc_buf| *adc_buf = Some(buf));
    }

    /// Check RTC clock source and change if necessary.
    #[task(shared = [rtc, bkp])]
    fn check_rtc_src(cx: check_rtc_src::Context) {
        (cx.shared.rtc, cx.shared.bkp).lock(|rtc, bkp| {
            let lse_ready = ClockSrc::Lse.is_ready();
            let cur_src = rtc.clock_src();

            let change = match cur_src {
                Some(ClockSrc::Lsi) if lse_ready => true,
                Some(ClockSrc::Lse) if !lse_ready => true,
                Some(ClockSrc::HseDiv128) => true,
                None => true,
                _ => false,
            };

            if change {
                let new_src = if lse_ready { ClockSrc::Lse } else { ClockSrc::Lsi };
                defmt::info!("changing RTC source from {} to {}", cur_src, new_src);
                rtc.set_clock_src(bkp, new_src);
                if rtc.set_prescalar(unwrap!(new_src.prescaler()) - Rtc::PRESCALER_NEG_OFFSET).is_err() {
                    defmt::warn!("cannot set RTC prescaler");
                }
                rtc.set_slowdown(bkp, 64);
            }
        });

        unwrap!(check_rtc_src::spawn_after(1u64.secs()));
    }

    // Prints RTC information.
    #[task(shared = [rtc, bkp])]
    fn print_rtc_info(cx: print_rtc_info::Context) {
        (cx.shared.rtc, cx.shared.bkp).lock(|rtc, bkp| {
            defmt::trace!("");
            defmt::trace!("RTC ready:      {}", rtc.is_ready());
            if let Ok(clock) = rtc.clock() {
                defmt::trace!("RTC time:       {}.{:03} s", clock.secs(), clock.millis());
            }
            defmt::trace!("RTC alarming:   {}", rtc.is_alarming());
            defmt::trace!("RTC source:     {}", rtc.clock_src());
            defmt::trace!("RTC prescaler:  {}", rtc.prescaler());
            defmt::trace!("RTC slowdown:   {}", rtc.slowdown(bkp));
            defmt::trace!("LSE ready:      {}", ClockSrc::Lse.is_ready());
            defmt::trace!("");
        });

        unwrap!(print_rtc_info::spawn_after(15u64.secs()));
    }

    /// RTC alarm interrupt.
    #[task(binds = RTCALARM, shared = [irq])]
    fn rtc_alarm(mut cx: rtc_alarm::Context) {
        defmt::info!("RTC alarm");
        cx.shared.irq.lock(|irq| {
            irq.pend_gpio_exti();
        });
    }

    /// External interrupt handler 0, handling EXTI0 - EXTI15.
    #[task(binds = EXTI0, shared = [irq])]
    fn exti0(mut cx: exti0::Context) {
        cx.shared.irq.lock(|irq| {
            irq.pend_gpio_exti();
        });
    }

    /// External interrupt handler 1, forwarding to EXTI0.
    #[task(binds = EXTI1)]
    fn exti1(_cx: exti1::Context) {
        NVIC::pend(Interrupt::EXTI0);
    }

    /// External interrupt handler 2, forwarding to EXTI0.
    #[task(binds = EXTI2)]
    fn exti2(_cx: exti2::Context) {
        NVIC::pend(Interrupt::EXTI0);
    }

    /// External interrupt handler 3, forwarding to EXTI0.
    #[task(binds = EXTI3)]
    fn exti3(_cx: exti3::Context) {
        NVIC::pend(Interrupt::EXTI0);
    }

    /// External interrupt handler 4, forwarding to EXTI0.
    #[task(binds = EXTI4)]
    fn exti4(_cx: exti4::Context) {
        NVIC::pend(Interrupt::EXTI0);
    }

    /// External interrupt handler 5-9, forwarding to EXTI0.
    #[task(binds = EXTI9_5)]
    fn exti9_5(_cx: exti9_5::Context) {
        NVIC::pend(Interrupt::EXTI0);
    }

    /// External interrupt handler 10-15, forwarding to EXTI0.
    #[task(binds = EXTI15_10)]
    fn exti15_10(_cx: exti15_10::Context) {
        NVIC::pend(Interrupt::EXTI0);
    }

    /// I2C error interrupt handler.
    #[task(binds = I2C1_ER, priority = 2)]
    fn i2c1_er(_cx: i2c1_er::Context) {
        NVIC::pend(Interrupt::I2C1_EV);
    }

    /// I2C event interrupt handler.
    #[task(binds = I2C1_EV,
        local = [i2c_reg_slave, copyright_offset, mfd_cell_index, pwm_timer_index, pwm_channel_index],
        shared = [start_bootloader, irq, ugpio, bkp, watchman, rtc, adc_buf, pwm_timers, board],
        priority = 2)]
    fn i2c1_ev(mut cx: i2c1_ev::Context) {
        let res = cx.local.i2c_reg_slave.event();

        let evt = match res {
            Ok(evt) => {
                match &evt {
                    Event::Read { reg, .. } => defmt::debug!("I2C read  0x{:02x}", reg),
                    Event::Write { reg, value } => defmt::debug!("I2C write 0x{:02x}: {=[u8]:#x}", reg, value),
                }
                evt
            }
            Err(nb::Error::WouldBlock) => {
                defmt::trace!("I2C event");
                return;
            }
            Err(nb::Error::Other(_err)) => {
                defmt::error!("I2C error");
                return;
            }
        };

        match evt {
            Event::Read { reg: reg::ID, value } => {
                value.set_u8(if BootInfo::is_from_bootloader() { ID_WITH_BOOTLOADER } else { ID_STANDALONE })
            }
            Event::Read { reg: reg::VERSION, value } => value.set(VERSION),
            Event::Read { reg: reg::EMC_MODEL, value } => value.set_u8(BootInfo::get().emc_model),
            Event::Read { reg: reg::BOARD_MODEL, value } => value.set(BootInfo::get().board_model()),
            Event::Read { reg: reg::BOOTLOADER_VERSION, value } => match BootInfo::get().bootloader_version() {
                Some(v) => value.set(v),
                None => value.set(&[0]),
            },
            Event::Read { reg: reg::COPYRIGHT, value } => {
                value.set_clipped(COPYRIGHT.get(*cx.local.copyright_offset..).unwrap_or_default())
            }
            Event::Write { reg: reg::COPYRIGHT, value } => {
                *cx.local.copyright_offset = value.as_u8() as usize;
            }
            Event::Read { reg: reg::MFD_CELL, value } => match MFD_CELLS.get(*cx.local.mfd_cell_index) {
                Some(cell) => value.set(cell),
                None => value.set(&[0]),
            },
            Event::Write { reg: reg::MFD_CELL, value } => {
                *cx.local.mfd_cell_index = value.as_u8() as usize;
            }
            Event::Read { reg: reg::BOOT_REASON, value } => value.set_u16(BootInfo::get().boot_reason),
            Event::Read { reg: reg::RESET_STATUS, value } => value.set_u8(BootInfo::get().reset_status.0),
            Event::Read { reg: reg::START_REASON, value } => value.set_u8(BootInfo::get().start_reason),
            Event::Read { reg: reg::PROGRAM_ID, value } => value.set_u32(BootInfo::get().id),
            Event::Read { reg: reg::IRQ_MASK, value } => cx.shared.irq.lock(|irq| value.set_u32(irq.get_mask())),
            Event::Write { reg: reg::IRQ_MASK, value } => cx.shared.irq.lock(|irq| irq.set_mask(value.as_u32())),
            Event::Read { reg: reg::IRQ_PENDING, value } => {
                cx.shared.irq.lock(|irq| value.set_u32(irq.get_pending_and_clear()))
            }
            Event::Read { reg: reg::IRQ_EXTI_TRIGGER_RISING_EDGE, value } => {
                cx.shared.irq.lock(|irq| value.set_u32(irq.get_exti_trigger_rising_edge()))
            }
            Event::Write { reg: reg::IRQ_EXTI_TRIGGER_RISING_EDGE, value } => {
                cx.shared.irq.lock(|irq| irq.set_exti_trigger_rising_edge(value.as_u32()))
            }
            Event::Read { reg: reg::IRQ_EXTI_TRIGGER_FALLING_EDGE, value } => {
                cx.shared.irq.lock(|irq| value.set_u32(irq.get_exti_trigger_falling_edge()))
            }
            Event::Write { reg: reg::IRQ_EXTI_TRIGGER_FALLING_EDGE, value } => {
                cx.shared.irq.lock(|irq| irq.set_exti_trigger_falling_edge(value.as_u32()))
            }
            Event::Read { reg: reg::IRQ_EXTI_TRIGGER_HIGH_LEVEL, value } => {
                cx.shared.irq.lock(|irq| value.set_u16(irq.get_exti_gpio_trigger_high_level()))
            }
            Event::Write { reg: reg::IRQ_EXTI_TRIGGER_HIGH_LEVEL, value } => {
                cx.shared.irq.lock(|irq| irq.set_exti_gpio_trigger_high_level(value.as_u16()))
            }
            Event::Read { reg: reg::IRQ_EXTI_TRIGGER_LOW_LEVEL, value } => {
                cx.shared.irq.lock(|irq| value.set_u16(irq.get_exti_gpio_trigger_low_level()))
            }
            Event::Write { reg: reg::IRQ_EXTI_TRIGGER_LOW_LEVEL, value } => {
                cx.shared.irq.lock(|irq| irq.set_exti_gpio_trigger_low_level(value.as_u16()))
            }
            Event::Read { reg: reg::IRQ_EXTI_GPIO_SRC, value } => {
                cx.shared.irq.lock(|irq| value.set_u64(irq.get_exti_gpio_source()))
            }
            Event::Write { reg: reg::IRQ_EXTI_GPIO_SRC, value } => {
                cx.shared.irq.lock(|irq| irq.set_exti_gpio_source(value.as_u64()))
            }
            Event::Write { reg: reg::IRQ_EXTI_DO_TRIGGER_LEVEL, .. } => {
                (cx.shared.irq, cx.shared.ugpio).lock(|irq, ugpio| irq.do_trigger_exti(&ugpio.get_in()))
            }
            Event::Read { reg: reg::WDG_UNLOCK, value } => {
                cx.shared.watchman.lock(|watchman| value.set_u8(u8::from(watchman.unlocked())))
            }
            Event::Write { reg: reg::WDG_UNLOCK, value } => {
                cx.shared.watchman.lock(|watchman| watchman.unlock(value.as_u64()))
            }
            Event::Read { reg: reg::WDG_INTERVAL, value } => {
                cx.shared.watchman.lock(|watchman| value.set_u32(watchman.interval().to_millis() as u32))
            }
            Event::Write { reg: reg::WDG_INTERVAL, value } => {
                cx.shared.watchman.lock(|watchman| watchman.set_interval((value.as_u32() as u64).millis()))
            }
            Event::Read { reg: reg::WDG_ACTIVE, value } => {
                cx.shared.watchman.lock(|watchman| value.set_u8(u8::from(watchman.active())))
            }
            Event::Write { reg: reg::WDG_ACTIVE, value } => {
                cx.shared.watchman.lock(|watchman| watchman.set_active(value.as_u8() != 0))
            }
            Event::Write { reg: reg::WDG_PET, value } => {
                cx.shared.watchman.lock(|watchman| watchman.pet(value.as_u32()))
            }
            Event::Write { reg: reg::WDG_PET_CODE, value } => {
                cx.shared.watchman.lock(|watchman| watchman.set_pet_code(value.as_u32()))
            }
            Event::Read { reg: reg::RTC_READY, value } => {
                cx.shared.rtc.lock(|rtc| value.set_u8(u8::from(rtc.is_ready())))
            }
            Event::Read { reg: reg::RTC_SRC, value } => {
                cx.shared.rtc.lock(|rtc| value.set_u8(rtc.clock_src().map(|src| src as u8).unwrap_or_default()))
            }
            Event::Read { reg: reg::RTC_PRESCALER, value } => {
                cx.shared.rtc.lock(|rtc| value.set_u32(rtc.prescaler().unwrap_or_default()))
            }
            Event::Read { reg: reg::RTC_SLOWDOWN, value } => {
                (cx.shared.rtc, cx.shared.bkp).lock(|rtc, bkp| value.set_u8(rtc.slowdown(bkp)))
            }
            Event::Write { reg: reg::RTC_SLOWDOWN, value } => {
                (cx.shared.rtc, cx.shared.bkp).lock(|rtc, bkp| rtc.set_slowdown(bkp, value.as_u8()))
            }
            Event::Read { reg: reg::RTC_CLOCK, value } => {
                cx.shared.rtc.lock(|rtc| {
                    if let Ok(clock) = rtc.clock() {
                        value.set_u64((clock.secs() as u64) << 32 | clock.millis() as u64);
                    }
                });
            }
            Event::Write { reg: reg::RTC_CLOCK, value } => {
                if cx.shared.rtc.lock(|rtc| rtc.set_clock(value.as_u32())).is_err() {
                    defmt::error!("setting RTC clock failed");
                }
            }
            Event::Read { reg: reg::RTC_ALARM, value } => {
                (cx.shared.rtc, cx.shared.bkp).lock(|rtc, bkp| value.set_u32(rtc.alarm(bkp)))
            }
            Event::Write { reg: reg::RTC_ALARM, value } => {
                if (cx.shared.rtc, cx.shared.bkp).lock(|rtc, bkp| rtc.set_alarm(value.as_u32(), bkp)).is_err() {
                    defmt::error!("setting RTC alarm failed");
                }
            }
            Event::Read { reg: reg::RTC_ALARM_ARMED, value } => {
                cx.shared.rtc.lock(|rtc| value.set_u8(u8::from(rtc.is_alarm_listened())));
            }
            Event::Write { reg: reg::RTC_ALARM_ARMED, value } => {
                cx.shared.rtc.lock(|rtc| {
                    let res = if value.as_u8() != 0 { rtc.listen_alarm() } else { rtc.unlisten_alarm() };
                    if res.is_err() {
                        defmt::error!("setting RTC alarm armed failed");
                    }
                });
            }
            Event::Read { reg: reg::RTC_ALARM_OCCURRED, value } => {
                cx.shared.rtc.lock(|rtc| value.set_u8(u8::from(rtc.is_alarming().unwrap_or_default())));
            }
            Event::Write { reg: reg::RTC_ALARM_OCCURRED, .. } => {
                cx.shared.rtc.lock(|rtc| {
                    if rtc.silence_alarm().is_err() {
                        defmt::error!("silencing RTC alarm failed");
                    }
                });
            }
            Event::Read { reg: reg::RTC_ALARM_AT_BOOT, value } => {
                cx.shared.rtc.lock(|rtc| {
                    value.set_u8(match rtc.was_alarming_at_init() {
                        Ok(true) => 0x01,
                        Ok(false) => 0x00,
                        Err(_) => 0xff,
                    })
                });
            }
            Event::Read { reg: reg::GPIO_COUNT, value } => {
                cx.shared.ugpio.lock(|ugpio| value.set_u8(ugpio.gpios() as u8));
            }
            Event::Read { reg: reg::GPIO_CFG, value } => {
                cx.shared.ugpio.lock(|ugpio| {
                    let v: [u8; board::PORTS * size_of::<u64>()] = array_from_u64(&ugpio.get_cfg());
                    value.set(&v);
                });
            }
            Event::Write { reg: reg::GPIO_CFG, value } if value.len() == board::PORTS * size_of::<u64>() => {
                cx.shared.ugpio.lock(|ugpio| {
                    let v: [u64; board::PORTS] = array_to_u64(&value);
                    ugpio.set_cfg(&v);
                });
            }
            Event::Read { reg: reg::GPIO_OUT, value } => {
                cx.shared.ugpio.lock(|ugpio| {
                    let v: [u8; board::PORTS * size_of::<u16>()] = array_from_u16(&ugpio.get_out());
                    value.set(&v);
                });
            }
            Event::Write { reg: reg::GPIO_OUT, value } if value.len() == board::PORTS * size_of::<u16>() => {
                cx.shared.ugpio.lock(|ugpio| {
                    let v: [u16; board::PORTS] = array_to_u16(&value);
                    ugpio.set_out(&v);
                });
            }
            Event::Read { reg: reg::GPIO_IN, value } => {
                cx.shared.ugpio.lock(|ugpio| {
                    let v: [u8; board::PORTS * size_of::<u16>()] = array_from_u16(&ugpio.get_in());
                    value.set(&v);
                });
            }
            Event::Read { reg: reg::GPIO_USABLE, value } => {
                cx.shared.ugpio.lock(|ugpio| {
                    let v: [u8; board::PORTS * size_of::<u16>()] = array_from_u16(ugpio.usable());
                    value.set(&v);
                });
            }
            Event::Write { reg: reg::ADC_CONVERT, .. } => {
                cx.shared.adc_buf.lock(|adc_buf| *adc_buf = None);
                unwrap!(adc_convert::spawn());
            }
            Event::Read { reg: reg::ADC_READY, value } => {
                cx.shared.adc_buf.lock(|adc_buf| value.set_u8(u8::from(adc_buf.is_some())));
            }
            Event::Read { reg: reg::ADC_VREF, value } => {
                cx.shared
                    .adc_buf
                    .lock(|adc_buf| value.set_u16(adc_buf.as_ref().map(|b| b.vref).unwrap_or_default()));
            }
            Event::Read { reg: reg::ADC_VALUES, value } => cx.shared.adc_buf.lock(|adc_buf| {
                if let Some(adc_buf) = &adc_buf {
                    let v: [u8; AdcInputs::CHANNELS * size_of::<u16>()] = array_from_u16(&adc_buf.voltages());
                    value.set(&v);
                }
            }),
            Event::Read { reg: reg::ADC_TEMPERATURE, value } => {
                cx.shared
                    .adc_buf
                    .lock(|adc_buf| value.set_u32(adc_buf.as_ref().map(|b| b.temp as u32).unwrap_or_default()));
            }
            Event::Write { reg: reg::POWER_OFF, value } => {
                let delay = value.as_u16();
                defmt::info!("requesting power off in {} ms", delay);
                unwrap!(power_off::spawn_after((delay as u64).millis()));
            }
            Event::Write { reg: reg::POWER_RESTART, value } => {
                let delay = value.as_u16();
                defmt::info!("requesting restart in {} ms", delay);
                unwrap!(power_restart::spawn_after((delay as u64).millis()));
            }
            Event::Read { reg: reg::PWM_TIMERS, value } => {
                cx.shared.pwm_timers.lock(|pwm_timers| value.set_u8(pwm_timers.len() as u8))
            }
            Event::Read { reg: reg::PWM_TIMER, value } => value.set_u8(*cx.local.pwm_timer_index),
            Event::Write { reg: reg::PWM_TIMER, value } => {
                *cx.local.pwm_timer_index = value.as_u8();
            }
            Event::Read { reg: reg::PWM_TIMER_CHANNELS, value } => {
                value.set_u8(pwm::Channel::count() as u8);
            }
            Event::Write { reg: reg::PWM_TIMER_REMAP, value } => cx.shared.pwm_timers.lock(|pwm_timers| {
                if let Some(pwm_timer) = pwm_timers.get_mut(*cx.local.pwm_timer_index as usize) {
                    pwm_timer.set_remap(value.as_u8());
                }
            }),
            Event::Write { reg: reg::PWM_TIMER_FREQUENCY, value } => cx.shared.pwm_timers.lock(|pwm_timers| {
                if let Some(pwm_timer) = pwm_timers.get_mut(*cx.local.pwm_timer_index as usize) {
                    pwm_timer.set_remap(value.as_u8());
                }
            }),
            Event::Read { reg: reg::PWM_CHANNEL, value } => value.set_u8(*cx.local.pwm_channel_index),
            Event::Write { reg: reg::PWM_CHANNEL, value } => {
                *cx.local.pwm_channel_index = value.as_u8();
            }
            Event::Write { reg: reg::PWM_CHANNEL_DUTY_CYCLE, value } => {
                cx.shared.pwm_timers.lock(|pwm_timers| {
                    if let Some(pwm_timer) = pwm_timers.get_mut(*cx.local.pwm_timer_index as usize) {
                        if let Ok(ch) = pwm::Channel::try_from(*cx.local.pwm_channel_index) {
                            pwm_timer.set_duty_cycle(&ch, value.as_u16());
                        }
                    }
                });
            }
            Event::Write { reg: reg::PWM_CHANNEL_POLARITY, value } => {
                cx.shared.pwm_timers.lock(|pwm_timers| {
                    if let Some(pwm_timer) = pwm_timers.get_mut(*cx.local.pwm_timer_index as usize) {
                        if let Ok(ch) = pwm::Channel::try_from(*cx.local.pwm_channel_index) {
                            pwm_timer.set_polarity(&ch, value.as_u8() != 0);
                        }
                    }
                });
            }
            Event::Write { reg: reg::PWM_CHANNEL_OUTPUT, value } => {
                cx.shared.pwm_timers.lock(|pwm_timers| {
                    if let Some(pwm_timer) = pwm_timers.get_mut(*cx.local.pwm_timer_index as usize) {
                        if let Ok(ch) = pwm::Channel::try_from(*cx.local.pwm_channel_index) {
                            pwm_timer.set_output(&ch, value.as_u8() != 0);
                        }
                    }
                });
            }
            Event::Write { reg: reg::RESET, .. } => {
                defmt::info!("reset");
                cx.shared.bkp.lock(|bkp| boot::reset(bkp));
            }
            Event::Write { reg: reg::START_BOOTLOADER, .. } => {
                defmt::info!("requesting start of bootloader");
                (cx.shared.start_bootloader, cx.shared.bkp).lock(|start_bootloader, bkp| {
                    BootReason::StartBootloader.set(bkp);
                    *start_bootloader = true;
                });
                irq::mask_exti();
            }
            event => {
                cx.shared.board.lock(|board| {
                    if let Err(err) = board.i2c_event(event) {
                        match err.0 {
                            Event::Read { reg, value } => {
                                defmt::warn!("I2C read from unknown register 0x{:02x}", reg);
                                value.set(&[]);
                            }
                            Event::Write { reg, value: _ } => {
                                defmt::warn!("I2C write to unknown register 0x{:02x}", reg);
                            }
                        }
                    }
                });
            }
        }
    }
}
