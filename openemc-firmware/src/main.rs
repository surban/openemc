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
mod bq25713;
mod i2c_reg_slave;
mod i2c_slave;
mod irq;
mod pio;
mod pwm;
mod reg;
mod rtc;
mod supply;
mod util;
mod watchman;

// Logging provider.
#[cfg(feature = "defmt-ringbuf")]
use defmt_ringbuf as _;
#[cfg(feature = "defmt-rtt")]
use defmt_rtt as _;

use panic_probe as _;
use stm32f1xx_hal as _;

use core::{
    cell::Cell,
    mem::{replace, size_of},
};
use cortex_m::peripheral::NVIC;
use defmt::{unwrap, Format};
use stm32f1::stm32f103::Interrupt;
use stm32f1xx_hal::{
    adc::Adc,
    backup_domain::BackupDomain,
    gpio::{Alternate, OpenDrain, Pin},
    i2c,
    i2c::I2c,
    pac::{ADC1, I2C1},
    prelude::*,
    watchdog::IndependentWatchdog,
};
use systick_monotonic::*;

use crate::{
    adc::{AdcBuf, AdcInputs},
    backup::BackupReg,
    board::Board,
    boot::{BootInfoExt, BootReasonExt},
    bq25713::{Battery, Bq25713},
    i2c_reg_slave::{Event, I2CRegSlave},
    i2c_slave::I2cSlave,
    pio::MaskedGpio,
    pwm::PwmTimer,
    rtc::{ClockSrc, Rtc},
    supply::{max14636::Max14636, stusb4500::StUsb4500, PowerSupply},
    util::{array_from_u16, array_from_u64, array_to_u16, array_to_u64},
    watchman::Watchman,
};
use openemc_shared::{BootInfo, BootReason};

/// OpenEMC firmware version.
pub static VERSION: &[u8] = env!("CARGO_PKG_VERSION").as_bytes();

/// Firmware copyright.
static COPYRIGHT: &[u8] = b"(c) 2022-2023 Sebastian Urban <surban@surban.net> license: GNU GPL version 3";

/// MFD cells.
static MFD_CELLS: &[&[u8]] = &[
    b"openemc,openemc_adc",
    b"openemc,openemc_battery",
    b"openemc,openemc_gpio",
    b"openemc,openemc_pinctrl",
    b"openemc,openemc_power",
    b"openemc,openemc_pwm",
    b"openemc,openemc_rtc",
    b"openemc,openemc_supply",
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

/// Bootloader log buffer.
#[cfg(feature = "defmt-ringbuf")]
#[no_mangle]
#[used]
#[link_section = ".defmt_log"]
pub static mut BOOTLOADER_LOG: core::mem::MaybeUninit<
    defmt_ringbuf::RingBuffer<{ openemc_shared::BOOTLOADER_LOG_SIZE }>,
> = core::mem::MaybeUninit::uninit();

/// Firmware log buffer.
#[cfg(feature = "defmt-ringbuf")]
#[no_mangle]
#[used]
#[link_section = ".defmt_log"]
pub static mut LOG: core::mem::MaybeUninit<defmt_ringbuf::RingBuffer<{ openemc_shared::LOG_SIZE }>> =
    core::mem::MaybeUninit::uninit();

/// Bootloader log reader.
#[cfg(feature = "defmt-ringbuf")]
static mut BOOTLOADER_LOG_REF: Option<
    &'static mut defmt_ringbuf::RingBuffer<{ openemc_shared::BOOTLOADER_LOG_SIZE }>,
> = None;

/// System power mode.
#[derive(Clone, Copy, Format, PartialEq, Eq)]
pub enum PowerMode {
    /// Full power on.
    Full,
    /// Power only components required for charging.
    Charging,
}

/// I2C register slave that may have remapped pins.
#[allow(clippy::type_complexity)]
pub enum RemappableI2CRegSlave {
    Remapped(
        I2CRegSlave<
            I2C1,
            (Pin<'B', 8, Alternate<OpenDrain>>, Pin<'B', 9, Alternate<OpenDrain>>),
            I2C_BUFFER_SIZE,
        >,
    ),
    Normal(
        I2CRegSlave<
            I2C1,
            (Pin<'B', 6, Alternate<OpenDrain>>, Pin<'B', 7, Alternate<OpenDrain>>),
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

    /// Responds to a read register event.
    pub fn respond(&mut self, response: i2c_reg_slave::Response<I2C_BUFFER_SIZE>) {
        match self {
            Self::Remapped(i2c) => i2c.respond(response),
            Self::Normal(i2c) => i2c.respond(response),
        }
    }
}

/// I2C request handling status.
pub enum I2cReqHandling {
    /// No request is being handled.
    Idle,
    /// The response to a read request is being computed.
    Read(u8),
    /// The response to a read request is ready.
    ReadResponse { reg: u8, response: i2c_reg_slave::Response<I2C_BUFFER_SIZE> },
    /// A write request is being handled.
    Write(u8),
}

/// I2C 2 master.
type I2c2Master = i2c::BlockingI2c<
    stm32f1xx_hal::pac::I2C2,
    (
        stm32f1xx_hal::gpio::Pin<'B', 10, Alternate<OpenDrain>>,
        stm32f1xx_hal::gpio::Pin<'B', 11, Alternate<OpenDrain>>,
    ),
>;

/// Instant in time.
pub type Instant = systick_monotonic::fugit::Instant<u64, 1, 100>;

/// Time duration.
pub type Duration = systick_monotonic::fugit::Duration<u64, 1, 100>;

/// Delay.
pub type Delay = stm32f1xx_hal::timer::Delay<stm32f1::stm32f103::TIM4, 1000>;

/// Board.
pub use boards::Chosen as ThisBoard;

/// IRQ state.
pub type IrqState = irq::IrqState<{ board::PORTS }>;

#[rtic::app(device = stm32f1::stm32f103, peripherals = true, dispatchers = [SPI1, SPI2, SPI3])]
mod app {
    use super::*;

    /// System timer.
    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<100>;

    /// Shared resources.
    #[shared]
    struct Shared {
        /// I2C slave request handling status.
        i2c_req: I2cReqHandling,
        /// If true, idle task starts bootloader.
        start_bootloader: bool,
        /// IRQ state.
        irq: IrqState,
        /// Backup domain.
        bkp: BackupDomain,
        /// Watchdog manager.
        watchman: Watchman,
        /// Real-time clock.
        rtc: Rtc,
        /// Result of latest AD conversion.
        adc_buf: Option<AdcBuf>,
        /// I2C 2 master.
        i2c2: Option<I2c2Master>,
        /// STUSB4500 USB PD controller.
        stusb4500: Option<StUsb4500<I2c2Master>>,
        /// MAX14636 USB charger detector.
        max14636: Option<Max14636>,
        /// BQ25713 batter charge controller.
        bq25713: Option<Bq25713<I2c2Master>>,
        /// Latest power supply report.
        power_supply: Option<PowerSupply>,
        /// Latest battery report.
        battery: Option<Battery>,
        /// Board.
        board: ThisBoard,
        /// Power mode.
        power_mode: PowerMode,
    }

    /// Exclusive resources.
    #[local]
    struct Local {
        /// I2C slave.
        i2c_reg_slave: Option<RemappableI2CRegSlave>,
        /// User GPIO.
        ugpio: MaskedGpio<{ board::PORTS }>,
        /// PWM timers.
        pwm_timers: [PwmTimer; 4],
        /// AD converter.
        adc: Adc<ADC1>,
        /// AD converter inputs.
        adc_inp: AdcInputs,
    }

    /// Initialization (entry point).
    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Initialize logging.
        #[cfg(feature = "defmt-ringbuf")]
        unsafe {
            defmt_ringbuf::init(&mut LOG, || NVIC::pend(Interrupt::USART3));
            BOOTLOADER_LOG_REF = Some(defmt_ringbuf::RingBuffer::init(&mut BOOTLOADER_LOG));
        }

        defmt::warn!("OpenEMC version {:a}", VERSION);
        defmt::warn!("{:a}", COPYRIGHT);

        boot::init();

        // Create HAL objects.
        let rcc = cx.device.RCC.constrain();
        let mut bkp = rcc.bkp.constrain(cx.device.BKP, &mut cx.device.PWR);
        let mut flash = cx.device.FLASH.constrain();
        let mut afio = cx.device.AFIO.constrain();
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        let mono = Systick::new(cx.core.SYST, clocks.sysclk().to_Hz());
        let adc = Adc::adc1(cx.device.ADC1, clocks);
        let adc_inp = AdcInputs::new();
        let mut delay = cx.device.TIM4.delay_ms(&clocks);
        let gpioa = unsafe { cx.device.GPIOA.split_without_reset() };
        let mut gpiob = unsafe { cx.device.GPIOB.split_without_reset() };
        let _gpioc = unsafe { cx.device.GPIOC.split_without_reset() };
        let mut _gpiod = unsafe { cx.device.GPIOD.split_without_reset() };

        // Configure GPIO remapping.
        let (_pa15, _pb3, _pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);
        afio.mapr.modify_mapr(|_, w| w.pd01_remap().set_bit());

        // Initialize backup registers.
        BackupReg::init(&mut bkp);

        // Get boot information.
        unsafe { BootInfo::init(&bkp) };
        let bi = BootInfo::get();

        // Create board handler.
        defmt::info!("board new");
        let mut board = ThisBoard::new(bi, &mut afio, &mut delay);
        let power_mode = board.power_mode();
        defmt::info!("board new done");
        board.set_power_led(bi.powered_on);

        // Print boot information.
        if !BootInfo::is_from_bootloader() {
            defmt::warn!("firmware is running without bootloader");
        }
        defmt::info!("EMC model:      0x{:02x}", bi.emc_model);
        defmt::info!("board model:    {:a}", bi.board_model());
        defmt::info!("I2C address:    0x{:02x} (pins remapped: {:?})", ThisBoard::I2C_ADDR, ThisBoard::I2C_REMAP);
        defmt::info!("IRQ pin:        {} (mode: 0b{:04b})", ThisBoard::IRQ_PIN, ThisBoard::IRQ_PIN_CFG);
        BootReason::log(bi.boot_reason);
        bi.reset_status.log();
        defmt::info!("start reason:   0x{:02x}", bi.start_reason);
        defmt::info!("powered on:     {}", bi.powered_on);

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
            defmt::info!("Board off delay");
            delay.delay(1u32.secs());
        }
        if bi.boot_reason == BootReason::PowerOff as _ {
            board.set_power_led(false);
            BootReason::PowerOn.set(&mut bkp);
            board.shutdown();
        }

        // Start watchdog and its manager.
        let dog = IndependentWatchdog::new(cx.device.IWDG);
        let watchman = Watchman::new(dog, 120u64.secs(), power_mode == PowerMode::Full);
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
        defmt::info!("board power on in mode {:?}", power_mode);
        board.set_power_led(power_mode == PowerMode::Full);
        board.power_on(&mut delay);
        defmt::info!("board power on done");

        // Configure user GPIO.
        let mut usable = [0xffff; board::PORTS];
        board.limit_usable(&mut usable);
        if ThisBoard::I2C_REMAP {
            usable[1] &= !(1 << 8 | 1 << 9);
        } else {
            usable[1] &= !(1 << 6 | 1 << 7);
        }
        usable[ThisBoard::IRQ_PIN as usize / 16] &= !(1 << (ThisBoard::IRQ_PIN % 16));
        let ugpio = MaskedGpio::new(usable);

        // Initialize I2C bus 2 master.
        let i2c2 = ThisBoard::I2C2_MODE.map(|mode| {
            // Required for I2C timeouts to work.
            cx.core.DCB.enable_trace();
            cx.core.DWT.enable_cycle_counter();

            let scl = gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh);
            let sda = gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh);
            let i2c2 = I2c::i2c2(cx.device.I2C2, (scl, sda), mode, clocks);
            i2c2.blocking_default(clocks)
        });

        // Initialize BQ25713.
        let bq25713 = match ThisBoard::BQ25713_CFG {
            Some(cfg) => {
                defmt::unwrap!(bq25713_periodic::spawn_after(1u64.secs()));
                Some(Bq25713::new(cfg))
            }
            None => None,
        };

        // Initialize power supplies.
        let stusb4500 = match ThisBoard::STUSB4500_I2C_ADDR {
            Some(addr) => {
                defmt::unwrap!(stusb4500_periodic::spawn_after(1u64.secs()));
                Some(StUsb4500::new(addr, &ThisBoard::USB_INITIAL_PDO, ThisBoard::USB_MAXIMUM_VOLTAGE))
            }
            None => None,
        };
        let max14636 = board.max14636();
        defmt::unwrap!(power_supply_update::spawn_after(500u64.millis()));

        // Initialize PWM timers.
        delay.release();
        let pwm_timers = [
            PwmTimer::new(pwm::Timer::Timer1, &clocks),
            PwmTimer::new(pwm::Timer::Timer2, &clocks),
            PwmTimer::new(pwm::Timer::Timer3, &clocks),
            PwmTimer::new(pwm::Timer::Timer4, &clocks),
        ];

        // Configure IRQ pin.
        let mut usable_exti = 0b0000_0000_0000_1111_1111_1111_1111_1111;
        board.limit_usable_exti(&mut usable_exti);
        let irq = IrqState::new(ThisBoard::IRQ_PIN, ThisBoard::IRQ_PIN_CFG, usable_exti);
        unsafe { irq::unmask_exti() };

        // Simulate PD IRQs.
        if ThisBoard::SIMULATE_PD_IRQS.is_some() {
            defmt::unwrap!(simulate_pd_irqs::spawn());
        }

        // Enable I2C register slave.
        let i2c_reg_slave = match power_mode {
            PowerMode::Full => {
                let slave = match ThisBoard::I2C_REMAP {
                    true => {
                        let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
                        let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);
                        let mut i2c = I2cSlave::i2c1(
                            cx.device.I2C1,
                            (scl, sda),
                            &mut afio.mapr,
                            ThisBoard::I2C_ADDR,
                            clocks,
                        );
                        i2c.listen_buffer();
                        i2c.listen_error();
                        i2c.listen_event();
                        RemappableI2CRegSlave::Remapped(I2CRegSlave::<_, _, I2C_BUFFER_SIZE>::new(i2c))
                    }
                    false => {
                        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
                        let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
                        let mut i2c = I2cSlave::i2c1(
                            cx.device.I2C1,
                            (scl, sda),
                            &mut afio.mapr,
                            ThisBoard::I2C_ADDR,
                            clocks,
                        );
                        i2c.listen_buffer();
                        i2c.listen_error();
                        i2c.listen_event();
                        RemappableI2CRegSlave::Normal(I2CRegSlave::<_, _, I2C_BUFFER_SIZE>::new(i2c))
                    }
                };
                unsafe { NVIC::unmask(Interrupt::I2C1_ER) };
                unsafe { NVIC::unmask(Interrupt::I2C1_EV) };
                Some(slave)
            }
            PowerMode::Charging => None,
        };

        // Drive charging LED.
        unwrap!(charge_led::spawn());

        defmt::debug!("init done");
        (
            Shared {
                i2c_req: I2cReqHandling::Idle,
                start_bootloader: false,
                irq,
                bkp,
                watchman,
                rtc,
                adc_buf: None,
                board,
                i2c2,
                stusb4500,
                max14636,
                bq25713,
                power_supply: None,
                battery: None,
                power_mode,
            },
            Local { i2c_reg_slave, adc, adc_inp, ugpio, pwm_timers },
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
    #[task(shared = [watchman, bkp], priority = 1)]
    fn watchdog_petter(mut cx: watchdog_petter::Context) {
        if !cx.shared.watchman.lock(|watchman| watchman.pet_hardware_watchdog()) {
            cx.shared.bkp.lock(|bkp| BootReason::WatchdogTimeout.set(bkp));
        }
        unwrap!(watchdog_petter::spawn_after(Watchman::pet_interval()));
    }

    /// Power off system.
    #[task(shared = [bkp, i2c2, bq25713])]
    fn power_off(mut cx: power_off::Context) {
        // Shut down charger, if we are running from battery.
        (cx.shared.i2c2, cx.shared.bq25713).lock(|i2c2, bq25713| {
            if let (Some(i2c2), Some(bq25713)) = (i2c2.as_mut(), bq25713.take()) {
                if !bq25713.status().map(|s| s.ac_stat).unwrap_or(true) {
                    defmt::info!("Shutting down BQ25713");
                    if let Err(err) = bq25713.shutdown(i2c2) {
                        defmt::error!("Cannot shutdown BQ25713: {}", err);
                    }
                }
            }
        });

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
    #[task(shared = [rtc, bkp], local = [reset_prescaler: bool = false])]
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
                *cx.local.reset_prescaler = true;
            }

            match rtc.clock_src() {
                Some(clock_src) if *cx.local.reset_prescaler => {
                    let prescaler = unwrap!(clock_src.prescaler()) - Rtc::PRESCALER_NEG_OFFSET;
                    match rtc.set_prescalar(prescaler) {
                        Ok(()) => {
                            rtc.set_slowdown(bkp, 64);
                            *cx.local.reset_prescaler = false;
                        }
                        Err(_) => {
                            defmt::warn!("cannot set RTC prescaler");
                        }
                    }
                }
                _ => (),
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

    /// Handles STUSB4500 alert.
    #[task(shared = [i2c2, stusb4500, board])]
    fn stusb4500_alert(cx: stusb4500_alert::Context) {
        (cx.shared.i2c2, cx.shared.stusb4500, cx.shared.board).lock(|i2c2, stusb4500, board| {
            if let Some(stusb4500) = stusb4500.as_mut() {
                stusb4500.alert(defmt::unwrap!(i2c2.as_mut()), board.check_stusb4500_attached());
                board.set_stusb4500_reset_pin(stusb4500.reset_pin_level());
                let _ = power_supply_update::spawn();
            }
        });
    }

    /// STUSB4500 periodic task.
    #[task(shared = [i2c2, stusb4500, board])]
    fn stusb4500_periodic(cx: stusb4500_periodic::Context) {
        (cx.shared.i2c2, cx.shared.stusb4500, cx.shared.board).lock(|i2c2, stusb4500, board| {
            if let Some(stusb4500) = stusb4500.as_mut() {
                stusb4500.periodic(defmt::unwrap!(i2c2.as_mut()), board.check_stusb4500_attached());
                board.set_stusb4500_reset_pin(stusb4500.reset_pin_level());
            }
        });

        defmt::unwrap!(stusb4500_periodic::spawn_after(500u64.millis()));
    }

    /// Updates the power supply status.
    #[task(
        shared = [i2c2, stusb4500, max14636, bq25713, power_supply, irq, board, &power_mode],
        local = [first: Option<Instant> = None],
        capacity = 3
    )]
    fn power_supply_update(cx: power_supply_update::Context) {
        let grace_period: Duration = 10u64.secs();
        let first = cx.local.first.get_or_insert_with(monotonics::now);

        (
            cx.shared.i2c2,
            cx.shared.stusb4500,
            cx.shared.max14636,
            cx.shared.bq25713,
            cx.shared.power_supply,
            cx.shared.irq,
            cx.shared.board,
        )
            .lock(|i2c2, stusb4500, max14636, bq25713, power_supply, irq, board| {
                // Merge power supply reports.
                let mut report = PowerSupply::default();
                if let Some(stusb4500) = stusb4500 {
                    defmt::trace!("STUSB4500 power supply report: {:?}", stusb4500.report());
                    report = report.merge(stusb4500.report());
                }
                if let Some(max14636) = max14636 {
                    let max14636_report = max14636.report();
                    defmt::trace!("MAX14636 power supply report: {:?}", max14636_report);
                    report = report.merge(&max14636_report);
                }

                if Some(&report) != power_supply.as_ref() {
                    defmt::info!("Power supply: {:?}", report);

                    // Configure battery charger.
                    match (i2c2, bq25713) {
                        (Some(i2c2), Some(bq25713)) if !report.is_unknown() => {
                            let max_current = report.max_current_ma();
                            defmt::info!("Setting BQ25713 maximum input current to {} mA", max_current);
                            if let Err(err) = bq25713.set_max_input_current(i2c2, max_current).and_then(|_| {
                                if max_current > 0 && !bq25713.is_charge_enabled() {
                                    bq25713.set_charge_enable(i2c2, true)?;
                                } else if max_current == 0 && bq25713.is_charge_enabled() {
                                    bq25713.set_charge_enable(i2c2, false)?;
                                }
                                Ok(())
                            }) {
                                defmt::error!("Cannot configure BQ25713 charging: {}", err);
                            }
                        }
                        _ => (),
                    }

                    // Check for power on and shutdown in charging mode.
                    if *cx.shared.power_mode == PowerMode::Charging {
                        if board.check_power_on_requested() {
                            defmt::info!("Power on requested");
                            let _ = power_restart::spawn();
                        }

                        if !report.is_connected() && monotonics::now() - *first > grace_period {
                            defmt::info!("Shutdown because power supply disconnected");
                            let _ = power_off::spawn();
                        }
                    }

                    // Store report and notify host.
                    irq.pend_soft(IrqState::SUPPLY | IrqState::BATTERY);
                    *power_supply = Some(report);
                }
            });

        defmt::unwrap!(power_supply_update::spawn_after(500u64.millis()));
    }

    /// BQ25713 periodic task.
    #[task(shared = [i2c2, bq25713, battery, irq, &power_mode], local = [first: Option<Instant> = None])]
    fn bq25713_periodic(cx: bq25713_periodic::Context) {
        let grace_period = 10u64.secs();
        let first = cx.local.first.get_or_insert_with(monotonics::now);

        (cx.shared.i2c2, cx.shared.bq25713, cx.shared.battery, cx.shared.irq).lock(
            |i2c2, bq25713, battery, irq| {
                if let Some(bq25713) = bq25713.as_mut() {
                    let i2c2 = defmt::unwrap!(i2c2.as_mut());
                    bq25713.periodic(i2c2);

                    let mut ac = false;
                    if let Some(status) = bq25713.status().cloned() {
                        if status.has_any_fault() {
                            defmt::warn!("BQ25713 reported a fault: {:?}", bq25713.status());
                        }

                        if status.has_clearable_fault() {
                            defmt::info!("Clearing BQ25713 fault");
                            if let Err(err) = bq25713.clear_faults(i2c2) {
                                defmt::error!("Clearing BQ25713 faults failed: {}", err);
                            }
                        }

                        ac = status.ac_stat;
                    }

                    match (bq25713.measurement(), ThisBoard::CRITICAL_LOW_BATTERY_VOLTAGE, cx.shared.power_mode) {
                        (Some(m), Some(min_mv), PowerMode::Full)
                            if !ac
                                && m.v_bat_mv < min_mv
                                && monotonics::now().checked_duration_since(*first) >= Some(grace_period) =>
                        {
                            if undervoltage_power_off::spawn().is_ok() {
                                defmt::warn!(
                                    "Battery voltage of {} mV is below critical voltage of {} mV, shutting down",
                                    m.v_bat_mv,
                                    min_mv
                                );
                            }
                        }
                        _ => (),
                    }

                    let report = bq25713.battery();
                    let changed = match (&battery, &report) {
                        (Some(battery), Some(report)) => report.changed_significantly(battery),
                        (None, None) => false,
                        _ => true,
                    };
                    if changed {
                        defmt::info!("Battery report: {:?}", &report);
                        *battery = report;
                        irq.pend_soft(IrqState::SUPPLY | IrqState::BATTERY);
                    }
                }
            },
        );

        defmt::unwrap!(bq25713_periodic::spawn_after(1u64.secs()));
    }

    /// Performs power off when battery has reached lower voltage limit.
    #[task(shared = [board], local = [blink_state: bool = false, blink_count: u8 = 0], capacity = 1)]
    fn undervoltage_power_off(mut cx: undervoltage_power_off::Context) {
        if *cx.local.blink_count < 100 {
            *cx.local.blink_state = !*cx.local.blink_state;
            *cx.local.blink_count += 1;
            cx.shared.board.lock(|board| board.set_power_led(*cx.local.blink_state));
            defmt::unwrap!(undervoltage_power_off::spawn_after(100u64.millis()));
        } else {
            defmt::warn!("Undervoltage power off");
            defmt::unwrap!(power_off::spawn());
        }
    }

    /// Controls the charging LED.
    #[task(
        shared = [battery, power_supply, board],
        local = [blink_state: u8 = 0, supplying_since: Option<Instant> = None],
    )]
    fn charge_led(cx: charge_led::Context) {
        (cx.shared.battery, cx.shared.power_supply, cx.shared.board).lock(|battery, power_supply, board| {
            let supplying = power_supply.as_ref().map(|s| s.is_connected()).unwrap_or_default();
            if supplying {
                if cx.local.supplying_since.is_none() {
                    *cx.local.supplying_since = Some(monotonics::now());
                }
            } else {
                *cx.local.supplying_since = None;
            }

            let grace_period: Duration = 5u64.secs();
            let grace = cx
                .local
                .supplying_since
                .map(|since| monotonics::now() - since <= grace_period)
                .unwrap_or_default();

            let (should_charge, charging_error) = battery
                .as_ref()
                .map(|b| {
                    (
                        b.voltage_mv < ThisBoard::CHARGING_LED_END_VOLTAGE,
                        !b.charging.is_charging() || b.current_ma < ThisBoard::CHARGING_LED_MIN_CURRENT,
                    )
                })
                .unwrap_or_default();

            let led = match (supplying, should_charge, charging_error) {
                (false, _, _) => false,
                (true, true, false) => *cx.local.blink_state / 10 == 1,
                (true, true, true) if grace => true,
                (true, true, true) => *cx.local.blink_state % 2 == 1,
                (true, false, _) => true,
            };
            board.set_charging_led(led);

            *cx.local.blink_state = (*cx.local.blink_state + 1) % 20;
        });

        defmt::unwrap!(charge_led::spawn_after(100u64.millis()));
    }

    /// Log data avilable.
    #[task(binds = USART3, shared = [irq])]
    fn log_available(mut cx: log_available::Context) {
        cx.shared.irq.lock(|irq| irq.pend_soft(IrqState::LOG))
    }

    /// Simulates interrupts on PD0 and PD1.
    #[task(shared = [irq])]
    fn simulate_pd_irqs(mut cx: simulate_pd_irqs::Context) {
        cx.shared.irq.lock(|irq| irq.simulate_gpio_pd_exti());
        defmt::unwrap!(simulate_pd_irqs::spawn_after(defmt::unwrap!(ThisBoard::SIMULATE_PD_IRQS)));
    }

    /// External interrupt handler 0, handling EXTI0 - EXTI15.
    #[task(binds = EXTI0, shared = [irq, board, i2c2, stusb4500, &power_mode])]
    fn exti0(mut cx: exti0::Context) {
        if cx.shared.board.lock(|board| board.check_stusb4500_alerting()) {
            defmt::trace!("STUSB4500 alert interrupt!");
            unwrap!(stusb4500_alert::spawn());
        } else if *cx.shared.power_mode == PowerMode::Charging
            && cx.shared.board.lock(|board| board.check_power_on_requested())
        {
            if power_restart::spawn().is_ok() {
                defmt::info!("Power on requested");
            }
        } else {
            cx.shared.irq.lock(|irq| {
                irq.pend_gpio_exti();
            });
        }
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
    #[task(binds = I2C1_ER, priority = 3)]
    fn i2c1_er(_cx: i2c1_er::Context) {
        NVIC::pend(Interrupt::I2C1_EV);
    }

    /// I2C event interrupt handler.
    #[task(binds = I2C1_EV, local = [i2c_reg_slave], shared = [i2c_req], priority = 3)]
    fn i2c1_ev(mut cx: i2c1_ev::Context) {
        let Some(i2c_reg_slave) = cx.local.i2c_reg_slave.as_mut() else { return };

        cx.shared.i2c_req.lock(|i2c_req| loop {
            match i2c_req {
                I2cReqHandling::Idle => match i2c_reg_slave.event() {
                    Ok(evt) => {
                        match &evt {
                            Event::Read { reg, .. } => {
                                defmt::debug!("I2C read  0x{:02x} pending", reg);
                                *i2c_req = I2cReqHandling::Read(*reg);
                            }
                            Event::Write { reg, value } => {
                                defmt::debug!("I2C write 0x{:02x}: {=[u8]:#x}", reg, value);
                                *i2c_req = I2cReqHandling::Write(*reg);
                            }
                        }
                        NVIC::mask(Interrupt::I2C1_EV);
                        NVIC::mask(Interrupt::I2C1_ER);
                        if i2c_slave_req::spawn(evt).is_err() {
                            defmt::panic!("I2C request handler busy");
                        }
                        break;
                    }
                    Err(nb::Error::WouldBlock) => {
                        defmt::trace!("I2C event internal processing");
                        break;
                    }
                    Err(nb::Error::Other(_err)) => defmt::error!("I2C error on OpenEMC control bus"),
                },
                I2cReqHandling::ReadResponse { reg, response } => {
                    defmt::debug!("I2C read  0x{:02x}: {=[u8]:#x}", reg, &response);
                    let I2cReqHandling::ReadResponse { response, ..} = replace(i2c_req, I2cReqHandling::Idle)
                        else { defmt::unreachable!() };
                    i2c_reg_slave.respond(response);
                }
                I2cReqHandling::Read(reg) => {
                    defmt::warn!("I2C event while processing read request for register 0x{:02x}", reg);
                    break;
                }
                I2cReqHandling::Write(reg) => {
                    defmt::warn!("I2C event while processing write request for register 0x{:02x}", reg);
                    break;
                }
            }
        })
    }

    /// I2C slave request handler.
    #[task(
        local = [
            copyright_offset: usize = 0,
            mfd_cell_index: usize = 0,
            ugpio,
            pwm_timers,
            pwm_timer_index: u8 = 0,
            pwm_channel_index: u8 = 0,
            echo: [u8; I2C_BUFFER_SIZE] = [0; I2C_BUFFER_SIZE],
        ],
        shared = [i2c_req, start_bootloader, irq, bkp, watchman, rtc, adc_buf, board, power_supply, battery],
        priority = 2,
    )]
    fn i2c_slave_req(mut cx: i2c_slave_req::Context, evt: i2c_reg_slave::Event<I2C_BUFFER_SIZE>) {
        let is_read = matches!(&evt, Event::Read { .. });
        let i2c_slave_response = Cell::new(None);
        let respond = |response: i2c_reg_slave::Response<I2C_BUFFER_SIZE>| {
            if is_read {
                i2c_slave_response.set(Some(response));
            } else {
                defmt::panic!("Response to I2C write event");
            }
        };
        let respond_slice = |response: &[u8]| respond(i2c_reg_slave::Response::set(response));
        let respond_clipped = |response: &[u8]| respond(i2c_reg_slave::Response::set_clipped(response));
        let respond_u8 = |response: u8| respond(i2c_reg_slave::Response::set_u8(response));
        let respond_u16 = |response: u16| respond(i2c_reg_slave::Response::set_u16(response));
        let respond_u32 = |response: u32| respond(i2c_reg_slave::Response::set_u32(response));
        let respond_u64 = |response: u64| respond(i2c_reg_slave::Response::set_u64(response));

        match evt {
            Event::Read { reg: reg::ID } => {
                respond_u8(if BootInfo::is_from_bootloader() { ID_WITH_BOOTLOADER } else { ID_STANDALONE })
            }
            Event::Read { reg: reg::VERSION } => respond_slice(VERSION),
            Event::Read { reg: reg::EMC_MODEL } => respond_u8(BootInfo::get().emc_model),
            Event::Read { reg: reg::BOARD_MODEL } => respond_slice(BootInfo::get().board_model()),
            Event::Read { reg: reg::BOOTLOADER_VERSION } => match BootInfo::get().bootloader_version() {
                Some(v) => respond_slice(v),
                None => respond_slice(&[0]),
            },
            Event::Read { reg: reg::COPYRIGHT } => {
                respond_clipped(COPYRIGHT.get(*cx.local.copyright_offset..).unwrap_or_default())
            }
            Event::Write { reg: reg::COPYRIGHT, value } => {
                *cx.local.copyright_offset = value.as_u8() as usize;
            }
            Event::Read { reg: reg::MFD_CELL } => match MFD_CELLS.get(*cx.local.mfd_cell_index) {
                Some(cell) => respond_slice(cell),
                None => respond_slice(&[0]),
            },
            Event::Write { reg: reg::MFD_CELL, value } => {
                *cx.local.mfd_cell_index = value.as_u8() as usize;
            }
            Event::Read { reg: reg::BOOT_REASON } => respond_u16(BootInfo::get().boot_reason),
            Event::Read { reg: reg::RESET_STATUS } => respond_u8(BootInfo::get().reset_status.0),
            Event::Read { reg: reg::START_REASON } => respond_u8(BootInfo::get().start_reason),
            Event::Read { reg: reg::PROGRAM_ID } => respond_u32(BootInfo::get().id),
            Event::Read { reg: reg::IRQ_MASK } => respond_u32(cx.shared.irq.lock(|irq| irq.get_mask())),
            Event::Write { reg: reg::IRQ_MASK, value } => cx.shared.irq.lock(|irq| irq.set_mask(value.as_u32())),
            Event::Read { reg: reg::IRQ_PENDING } => {
                respond_u32(cx.shared.irq.lock(|irq| irq.get_pending_and_clear()))
            }
            Event::Read { reg: reg::IRQ_EXTI_TRIGGER_RISING_EDGE } => {
                respond_u32(cx.shared.irq.lock(|irq| irq.get_exti_trigger_raising_edge()))
            }
            Event::Write { reg: reg::IRQ_EXTI_TRIGGER_RISING_EDGE, value } => {
                cx.shared.irq.lock(|irq| irq.set_exti_trigger_raising_edge(value.as_u32()))
            }
            Event::Read { reg: reg::IRQ_EXTI_TRIGGER_FALLING_EDGE } => {
                respond_u32(cx.shared.irq.lock(|irq| irq.get_exti_trigger_falling_edge()))
            }
            Event::Write { reg: reg::IRQ_EXTI_TRIGGER_FALLING_EDGE, value } => {
                cx.shared.irq.lock(|irq| irq.set_exti_trigger_falling_edge(value.as_u32()))
            }
            Event::Read { reg: reg::IRQ_EXTI_TRIGGER_HIGH_LEVEL } => {
                respond_u16(cx.shared.irq.lock(|irq| irq.get_exti_gpio_trigger_high_level()))
            }
            Event::Write { reg: reg::IRQ_EXTI_TRIGGER_HIGH_LEVEL, value } => {
                cx.shared.irq.lock(|irq| irq.set_exti_gpio_trigger_high_level(value.as_u16()))
            }
            Event::Read { reg: reg::IRQ_EXTI_TRIGGER_LOW_LEVEL } => {
                respond_u16(cx.shared.irq.lock(|irq| irq.get_exti_gpio_trigger_low_level()))
            }
            Event::Write { reg: reg::IRQ_EXTI_TRIGGER_LOW_LEVEL, value } => {
                cx.shared.irq.lock(|irq| irq.set_exti_gpio_trigger_low_level(value.as_u16()))
            }
            Event::Read { reg: reg::IRQ_EXTI_GPIO_SRC } => {
                respond_u64(cx.shared.irq.lock(|irq| irq.get_exti_gpio_source()))
            }
            Event::Write { reg: reg::IRQ_EXTI_GPIO_SRC, value } => {
                cx.shared.irq.lock(|irq| irq.set_exti_gpio_source(value.as_u64()))
            }
            Event::Write { reg: reg::IRQ_EXTI_DO_TRIGGER_LEVEL, .. } => {
                cx.shared.irq.lock(|irq| irq.do_trigger_exti(&cx.local.ugpio.get_in()))
            }
            Event::Read { reg: reg::WDG_UNLOCK } => {
                respond_u8(cx.shared.watchman.lock(|watchman| u8::from(watchman.unlocked())))
            }
            Event::Write { reg: reg::WDG_UNLOCK, value } => {
                cx.shared.watchman.lock(|watchman| watchman.unlock(value.as_u64()))
            }
            Event::Read { reg: reg::WDG_INTERVAL } => {
                respond_u32(cx.shared.watchman.lock(|watchman| watchman.interval().to_millis() as u32))
            }
            Event::Write { reg: reg::WDG_INTERVAL, value } => {
                cx.shared.watchman.lock(|watchman| watchman.set_interval((value.as_u32() as u64).millis()))
            }
            Event::Read { reg: reg::WDG_ACTIVE } => {
                respond_u8(cx.shared.watchman.lock(|watchman| u8::from(watchman.active())))
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
            Event::Read { reg: reg::RTC_READY } => respond_u8(cx.shared.rtc.lock(|rtc| u8::from(rtc.is_ready()))),
            Event::Read { reg: reg::RTC_SRC } => {
                respond_u8(cx.shared.rtc.lock(|rtc| rtc.clock_src().map(|src| src as u8).unwrap_or_default()))
            }
            Event::Read { reg: reg::RTC_PRESCALER } => {
                respond_u32(cx.shared.rtc.lock(|rtc| rtc.prescaler().unwrap_or_default()))
            }
            Event::Read { reg: reg::RTC_SLOWDOWN } => {
                respond_u8((cx.shared.rtc, cx.shared.bkp).lock(|rtc, bkp| rtc.slowdown(bkp)))
            }
            Event::Write { reg: reg::RTC_SLOWDOWN, value } => {
                (cx.shared.rtc, cx.shared.bkp).lock(|rtc, bkp| rtc.set_slowdown(bkp, value.as_u8()))
            }
            Event::Read { reg: reg::RTC_CLOCK } => {
                cx.shared.rtc.lock(|rtc| {
                    if let Ok(clock) = rtc.clock() {
                        respond_u64((clock.secs() as u64) << 32 | clock.millis() as u64);
                    }
                });
            }
            Event::Write { reg: reg::RTC_CLOCK, value } => {
                if cx.shared.rtc.lock(|rtc| rtc.set_clock(value.as_u32())).is_err() {
                    defmt::error!("setting RTC clock failed");
                }
            }
            Event::Read { reg: reg::RTC_ALARM } => {
                respond_u32((cx.shared.rtc, cx.shared.bkp).lock(|rtc, bkp| rtc.alarm(bkp)))
            }
            Event::Write { reg: reg::RTC_ALARM, value } => {
                if (cx.shared.rtc, cx.shared.bkp).lock(|rtc, bkp| rtc.set_alarm(value.as_u32(), bkp)).is_err() {
                    defmt::error!("setting RTC alarm failed");
                }
            }
            Event::Read { reg: reg::RTC_ALARM_ARMED } => {
                respond_u8(cx.shared.rtc.lock(|rtc| u8::from(rtc.is_alarm_listened())))
            }
            Event::Write { reg: reg::RTC_ALARM_ARMED, value } => {
                cx.shared.rtc.lock(|rtc| {
                    let res = if value.as_u8() != 0 { rtc.listen_alarm() } else { rtc.unlisten_alarm() };
                    if res.is_err() {
                        defmt::error!("setting RTC alarm armed failed");
                    }
                });
            }
            Event::Read { reg: reg::RTC_ALARM_OCCURRED } => {
                respond_u8(cx.shared.rtc.lock(|rtc| u8::from(rtc.is_alarming().unwrap_or_default())));
            }
            Event::Write { reg: reg::RTC_ALARM_OCCURRED, .. } => {
                cx.shared.rtc.lock(|rtc| {
                    if rtc.silence_alarm().is_err() {
                        defmt::error!("silencing RTC alarm failed");
                    }
                });
            }
            Event::Read { reg: reg::RTC_ALARM_AT_BOOT } => {
                respond_u8(cx.shared.rtc.lock(|rtc| match rtc.was_alarming_at_init() {
                    Ok(true) => 0x01,
                    Ok(false) => 0x00,
                    Err(_) => 0xff,
                }))
            }
            Event::Read { reg: reg::GPIO_COUNT } => respond_u8(cx.local.ugpio.gpios() as u8),
            Event::Read { reg: reg::GPIO_CFG } => {
                let v: [u8; board::PORTS * size_of::<u64>()] = array_from_u64(&cx.local.ugpio.get_cfg());
                respond_slice(&v);
            }
            Event::Write { reg: reg::GPIO_CFG, value } if value.len() == board::PORTS * size_of::<u64>() => {
                let v: [u64; board::PORTS] = array_to_u64(&value);
                cx.local.ugpio.set_cfg(&v);
            }
            Event::Read { reg: reg::GPIO_OUT } => {
                let v: [u8; board::PORTS * size_of::<u16>()] = array_from_u16(&cx.local.ugpio.get_out());
                respond_slice(&v);
            }
            Event::Write { reg: reg::GPIO_OUT, value } if value.len() == board::PORTS * size_of::<u16>() => {
                let v: [u16; board::PORTS] = array_to_u16(&value);
                cx.local.ugpio.set_out(&v);
            }
            Event::Read { reg: reg::GPIO_IN } => {
                let v: [u8; board::PORTS * size_of::<u16>()] = array_from_u16(&cx.local.ugpio.get_in());
                respond_slice(&v);
            }
            Event::Read { reg: reg::GPIO_USABLE } => {
                let v: [u8; board::PORTS * size_of::<u16>()] = array_from_u16(cx.local.ugpio.usable());
                respond_slice(&v);
            }
            Event::Write { reg: reg::ADC_CONVERT, .. } => {
                cx.shared.adc_buf.lock(|adc_buf| match adc_convert::spawn() {
                    Ok(()) => *adc_buf = None,
                    Err(()) => defmt::warn!("ADC conversion already in progress"),
                });
            }
            Event::Read { reg: reg::ADC_READY } => {
                respond_u8(cx.shared.adc_buf.lock(|adc_buf| u8::from(adc_buf.is_some())))
            }
            Event::Read { reg: reg::ADC_VREF } => respond_u16(
                cx.shared.adc_buf.lock(|adc_buf| adc_buf.as_ref().map(|b| b.vref).unwrap_or_default()),
            ),
            Event::Read { reg: reg::ADC_VALUES } => cx.shared.adc_buf.lock(|adc_buf| {
                if let Some(adc_buf) = &adc_buf {
                    let v: [u8; AdcInputs::CHANNELS * size_of::<u16>()] = array_from_u16(&adc_buf.voltages());
                    respond_slice(&v);
                }
            }),
            Event::Read { reg: reg::ADC_TEMPERATURE } => respond_u32(
                cx.shared.adc_buf.lock(|adc_buf| adc_buf.as_ref().map(|b| b.temp as u32).unwrap_or_default()),
            ),
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
            Event::Read { reg: reg::PWM_TIMERS } => respond_u8(cx.local.pwm_timers.len() as u8),
            Event::Read { reg: reg::PWM_TIMER } => respond_u8(*cx.local.pwm_timer_index),
            Event::Write { reg: reg::PWM_TIMER, value } => {
                *cx.local.pwm_timer_index = value.as_u8();
            }
            Event::Read { reg: reg::PWM_TIMER_CHANNELS } => {
                if let Some(pwm_timer) = cx.local.pwm_timers.get_mut(*cx.local.pwm_timer_index as usize) {
                    respond_u8(pwm_timer.channel_count());
                }
            }
            Event::Write { reg: reg::PWM_TIMER_REMAP, value } => {
                if let Some(pwm_timer) = cx.local.pwm_timers.get_mut(*cx.local.pwm_timer_index as usize) {
                    pwm_timer.set_remap(value.as_u8());
                }
            }
            Event::Write { reg: reg::PWM_TIMER_FREQUENCY, value } => {
                if let Some(pwm_timer) = cx.local.pwm_timers.get_mut(*cx.local.pwm_timer_index as usize) {
                    pwm_timer.set_frequency(value.as_u32().Hz());
                }
            }
            Event::Read { reg: reg::PWM_CHANNEL } => respond_u8(*cx.local.pwm_channel_index),
            Event::Write { reg: reg::PWM_CHANNEL, value } => {
                let channel = value.as_u8();
                if let Some(pwm_timer) = cx.local.pwm_timers.get_mut(*cx.local.pwm_timer_index as usize) {
                    if channel < pwm_timer.channel_count() {
                        *cx.local.pwm_channel_index = channel;
                    }
                }
            }
            Event::Write { reg: reg::PWM_CHANNEL_DUTY_CYCLE, value } => {
                if let Some(pwm_timer) = cx.local.pwm_timers.get_mut(*cx.local.pwm_timer_index as usize) {
                    pwm_timer.set_duty_cycle(*cx.local.pwm_channel_index, value.as_u16());
                }
            }
            Event::Write { reg: reg::PWM_CHANNEL_POLARITY, value } => {
                if let Some(pwm_timer) = cx.local.pwm_timers.get_mut(*cx.local.pwm_timer_index as usize) {
                    pwm_timer.set_polarity(*cx.local.pwm_channel_index, value.as_u8() != 0);
                }
            }
            Event::Write { reg: reg::PWM_CHANNEL_OUTPUT, value } => {
                if let Some(pwm_timer) = cx.local.pwm_timers.get_mut(*cx.local.pwm_timer_index as usize) {
                    pwm_timer.set_output(*cx.local.pwm_channel_index, value.as_u8() != 0);
                }
            }
            Event::Read { reg: reg::BATTERY_VOLTAGE } => {
                cx.shared.battery.lock(|battery| {
                    if let Some(battery) = battery {
                        respond_u32(battery.voltage_mv);
                    }
                });
            }
            Event::Read { reg: reg::BATTERY_MIN_VOLTAGE } => {
                respond_u32(ThisBoard::CRITICAL_LOW_BATTERY_VOLTAGE.unwrap_or_default())
            }
            Event::Read { reg: reg::BATTERY_MAX_VOLTAGE } => {
                cx.shared.battery.lock(|battery| {
                    if let Some(battery) = battery {
                        respond_u32(battery.max_voltage_mv);
                    }
                });
            }
            Event::Read { reg: reg::BATTERY_CHARGING } => {
                cx.shared.battery.lock(|battery| {
                    if let Some(battery) = battery {
                        respond_u8(battery.charging.into());
                    }
                });
            }
            Event::Read { reg: reg::BATTERY_CONSTANT_CHARGE_VOLTAGE } => {
                cx.shared.battery.lock(|battery| {
                    if let Some(battery) = battery {
                        respond_u32(battery.max_voltage_mv);
                    }
                });
            }
            Event::Read { reg: reg::BATTERY_CONSTANT_CHARGE_CURRENT } => {
                cx.shared.battery.lock(|battery| {
                    if let Some(battery) = battery {
                        respond_u32(battery.max_charge_current_ma);
                    }
                });
            }
            Event::Read { reg: reg::BATTERY_CURRENT } => {
                cx.shared.battery.lock(|battery| {
                    if let Some(battery) = battery {
                        respond_u32(battery.current_ma as u32);
                    }
                });
            }
            Event::Read { reg: reg::BATTERY_SYSTEM_VOLTAGE } => {
                cx.shared.battery.lock(|battery| {
                    if let Some(battery) = battery {
                        respond_u32(battery.system_voltage_mv);
                    }
                });
            }
            Event::Read { reg: reg::BATTERY_STATUS } => {
                cx.shared.battery.lock(|battery| match battery {
                    Some(battery) => respond_u8(if battery.present { 2 } else { 1 }),
                    None => respond_u8(0),
                });
            }
            Event::Read { reg: reg::SUPPLY_TYPE } => {
                cx.shared.power_supply.lock(|supply| {
                    if let Some(supply) = supply {
                        respond_u8(u8::from(&*supply));
                    }
                });
            }
            Event::Read { reg: reg::SUPPLY_REQUESTED_VOLTAGE } => {
                cx.shared.power_supply.lock(|supply| {
                    if let Some(supply) = supply {
                        respond_u32(supply.voltage_mv());
                    }
                });
            }
            Event::Read { reg: reg::SUPPLY_MAX_CURRENT } => {
                cx.shared.power_supply.lock(|supply| {
                    if let Some(supply) = supply {
                        respond_u32(supply.max_current_ma());
                    }
                });
            }
            Event::Read { reg: reg::SUPPLY_USB_COMMUNICATION } => {
                cx.shared.power_supply.lock(|supply| {
                    if let Some(supply) = supply {
                        respond_u8(supply.communication().into());
                    }
                });
            }
            Event::Read { reg: reg::SUPPLY_VOLTAGE } => {
                cx.shared.battery.lock(|battery| {
                    if let Some(battery) = battery {
                        respond_u32(battery.input_voltage_mv);
                    }
                });
            }
            Event::Read { reg: reg::SUPPLY_CURRENT } => {
                cx.shared.battery.lock(|battery| {
                    if let Some(battery) = battery {
                        respond_u32(battery.input_current_ma);
                    }
                });
            }
            Event::Write { reg: reg::RESET, .. } => {
                defmt::info!("reset");
                cx.shared.bkp.lock(|bkp| boot::reset(bkp));
            }
            Event::Read { reg: reg::LOG_READ } => {
                #[cfg(feature = "defmt-ringbuf")]
                {
                    let mut buf = [0; I2C_BUFFER_SIZE];
                    let (len, lost) = defmt_ringbuf::read(&mut buf[1..]);
                    buf[0] = len as u8;
                    if lost {
                        buf[0] |= 1 << 7;
                    }
                    respond_slice(&buf);
                }
                #[cfg(not(feature = "defmt-ringbuf"))]
                respond_slice(&[0]);
            }
            Event::Read { reg: reg::BOOTLOADER_LOG_READ } => {
                #[cfg(feature = "defmt-ringbuf")]
                {
                    use defmt_ringbuf::RingBuf;
                    let mut buf = [0; I2C_BUFFER_SIZE];
                    let (len, _lost) = unsafe { unwrap!(BOOTLOADER_LOG_REF.as_mut()).read(&mut buf[1..]) };
                    buf[0] = len as u8;
                    respond_slice(&buf);
                }
                #[cfg(not(feature = "defmt-ringbuf"))]
                respond_slice(&[0]);
            }
            Event::Read { reg: reg::ECHO } => {
                respond_slice(cx.local.echo);
            }
            Event::Write { reg: reg::ECHO, value } => {
                cx.local.echo[..value.len()].copy_from_slice(&value);
                cx.local.echo[value.len()..].fill(0);
            }
            Event::Write { reg: reg::START_BOOTLOADER, .. } => {
                defmt::info!("requesting start of bootloader");
                (cx.shared.start_bootloader, cx.shared.bkp).lock(|start_bootloader, bkp| {
                    BootReason::StartBootloader.set(bkp);
                    *start_bootloader = true;
                });
                irq::mask_exti();
            }

            // Board-specific registers.
            Event::Read { reg } => match cx.shared.board.lock(|board| board.i2c_read(reg)) {
                Ok(response) => respond(response),
                Err(board::UnknownI2cRegister) => {
                    defmt::warn!("I2C read from unknown register 0x{:02x}", reg);
                    respond_slice(&[]);
                }
            },
            Event::Write { reg, value } => {
                if let Err(board::UnknownI2cRegister) = cx.shared.board.lock(|board| board.i2c_write(reg, value))
                {
                    defmt::warn!("I2C write to unknown register 0x{:02x}", reg);
                }
            }
        }

        // Notify I2C event handler that request has been processed.
        cx.shared.i2c_req.lock(move |i2c_req| match (&i2c_req, i2c_slave_response.into_inner()) {
            (I2cReqHandling::Read(reg), Some(response)) => {
                defmt::debug!("Response set");
                *i2c_req = I2cReqHandling::ReadResponse { reg: *reg, response }
            }
            (I2cReqHandling::Read(reg), None) => {
                defmt::warn!("No response to I2C read from register 0x{:02x}", reg);
                *i2c_req =
                    I2cReqHandling::ReadResponse { reg: *reg, response: i2c_reg_slave::Response::set(&[]) };
            }
            (I2cReqHandling::Write(_), Some(_)) => defmt::panic!("Response to I2C write"),
            (I2cReqHandling::Write(_), None) => *i2c_req = I2cReqHandling::Idle,
            (I2cReqHandling::Idle, _) => defmt::panic!("I2C handling with no active request"),
            (I2cReqHandling::ReadResponse { .. }, _) => defmt::panic!("I2C read response already set"),
        });

        unsafe {
            NVIC::unmask(Interrupt::I2C1_EV);
            NVIC::unmask(Interrupt::I2C1_ER);
        }
        NVIC::pend(Interrupt::I2C1_EV);
    }
}
