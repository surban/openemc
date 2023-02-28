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
    boot::{BootInfoExt, BootReason},
    bq25713::{Battery, Bq25713},
    i2c_reg_slave::{Event, I2CRegSlave},
    i2c_slave::I2cSlave,
    pio::MaskedGpio,
    pwm::PwmTimer,
    rtc::{ClockSrc, Rtc},
    supply::{stusb4500::StUsb4500, PowerSupply},
    util::{array_from_u16, array_from_u64, array_to_u16, array_to_u64},
    watchman::Watchman,
};
use openemc_shared::BootInfo;

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
        /// If true, idle task starts bootloader.
        start_bootloader: bool,
        /// User GPIO.
        ugpio: MaskedGpio<{ board::PORTS }>,
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
        /// PWM timers.
        pwm_timers: [PwmTimer; 4],
        /// I2C 2 master.
        i2c2: Option<I2c2Master>,
        /// STUSB4500 USB PD controller.
        stusb4500: Option<StUsb4500<I2c2Master>>,
        /// BQ25713 batter charge controller.
        bq25713: Option<Bq25713<I2c2Master>>,
        /// Latest power supply report.
        power_supply: Option<PowerSupply>,
        /// Latest battery report.
        battery: Option<Battery>,
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
        /// Reset the RTC prescaler.
        rtc_reset_prescaler: bool,
        /// Undervoltage blink LED state.
        undervoltage_blink_state: bool,
        /// Undervoltage blink count.
        undervoltage_blink_count: usize,
        /// Time when BQ25713 periodic handler first ran.
        bq25713_first_periodic: Option<Instant>,
        /// Charge LED blink state.
        charge_blink_state: u8,
        /// Time when supply was attached, for charge LED.
        supplying_since: Option<Instant>,
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
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        let mono = Systick::new(cx.core.SYST, clocks.sysclk().to_Hz());
        let adc = Adc::adc1(cx.device.ADC1, clocks);
        let adc_inp = AdcInputs::new();
        let mut delay = cx.device.TIM4.delay_ms(&clocks);
        let gpioa = cx.device.GPIOA.split();
        let mut gpiob = cx.device.GPIOB.split();
        let _gpioc = cx.device.GPIOC.split();
        let mut _gpiod = cx.device.GPIOD.split();

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
        let mut board = ThisBoard::new(
            if BootInfo::is_from_bootloader() { Some(&bi.board_data) } else { None },
            &mut afio,
            &mut delay,
        );
        defmt::info!("board new done");

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
            board.set_power_led(false);
            BootReason::PowerOn.set(&mut bkp);
            board.shutdown();
        }

        // Start watchdog and its manager.
        let dog = IndependentWatchdog::new(cx.device.IWDG);
        let watchman = Watchman::new(dog, 120u64.secs(), option_env!("DISABLE_WATCHDOG").is_none());
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
        defmt::info!("board init");
        board.set_power_led(true);
        board.power_on(&mut delay);
        defmt::info!("board init done");

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

        // Initialize I2C bus 2 master.
        let mut i2c2 = ThisBoard::I2C2_MODE.map(|mode| {
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
            Some(cfg) => match Bq25713::new(defmt::unwrap!(i2c2.as_mut()), cfg) {
                Ok(bq) => {
                    defmt::unwrap!(bq25713_periodic::spawn_after(1u64.secs()));
                    Some(bq)
                }
                Err(err) => {
                    defmt::warn!("Cannot initialize BQ25713: {:?}", err);
                    None
                }
            },
            None => None,
        };

        // Initialize STUSB4500.
        let stusb4500 = match ThisBoard::STUSB4500_I2C_ADDR {
            Some(addr) => {
                defmt::unwrap!(stusb4500_periodic::spawn_after(1u64.secs()));
                Some(StUsb4500::new(addr, &ThisBoard::USB_INITIAL_PDO, ThisBoard::USB_MAXIMUM_VOLTAGE))
            }
            None => None,
        };

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
        let irq = IrqState::new(bi.irq_pin, bi.irq_pin_cfg, usable_exti);
        unsafe { irq::unmask_exti() };

        // Simulate PD IRQs.
        if ThisBoard::SIMULATE_PD_IRQS.is_some() {
            defmt::unwrap!(simulate_pd_irqs::spawn());
        }

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

        // Drive charging LED.
        unwrap!(charge_led::spawn());

        defmt::debug!("init done");
        (
            Shared {
                start_bootloader: false,
                irq,
                ugpio,
                bkp,
                watchman,
                rtc,
                adc_buf: None,
                pwm_timers,
                board,
                i2c2,
                stusb4500,
                bq25713,
                power_supply: None,
                battery: None,
            },
            Local {
                copyright_offset: 0,
                mfd_cell_index: 0,
                i2c_reg_slave,
                adc,
                adc_inp,
                pwm_timer_index: 0,
                pwm_channel_index: 0,
                rtc_reset_prescaler: false,
                undervoltage_blink_state: false,
                undervoltage_blink_count: 0,
                bq25713_first_periodic: None,
                charge_blink_state: 0,
                supplying_since: None,
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
    #[task(shared = [bkp, i2c2, bq25713])]
    fn power_off(mut cx: power_off::Context) {
        // Shut down charger, if we are running from battery.
        (cx.shared.i2c2, cx.shared.bq25713).lock(|i2c2, bq25713| {
            if let (Some(i2c2), Some(bq25713)) = (i2c2.as_mut(), bq25713.take()) {
                if !bq25713.status().ac_stat {
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
    #[task(shared = [rtc, bkp], local = [rtc_reset_prescaler])]
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
                *cx.local.rtc_reset_prescaler = true;
            }

            match rtc.clock_src() {
                Some(clock_src) if *cx.local.rtc_reset_prescaler => {
                    let prescaler = unwrap!(clock_src.prescaler()) - Rtc::PRESCALER_NEG_OFFSET;
                    match rtc.set_prescalar(prescaler) {
                        Ok(()) => {
                            rtc.set_slowdown(bkp, 64);
                            *cx.local.rtc_reset_prescaler = false;
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

                if stusb4500.new_report_available() {
                    defmt::unwrap!(stusb4500_handle_report::spawn());
                }

                board.set_stusb4500_reset_pin(stusb4500.reset_pin_level());
            }
        });
    }

    /// STUSB4500 periodic task.
    #[task(shared = [i2c2, stusb4500, board])]
    fn stusb4500_periodic(cx: stusb4500_periodic::Context) {
        (cx.shared.i2c2, cx.shared.stusb4500, cx.shared.board).lock(|i2c2, stusb4500, board| {
            if let Some(stusb4500) = stusb4500.as_mut() {
                stusb4500.periodic(defmt::unwrap!(i2c2.as_mut()), board.check_stusb4500_attached());

                if stusb4500.new_report_available() {
                    defmt::unwrap!(stusb4500_handle_report::spawn());
                }

                board.set_stusb4500_reset_pin(stusb4500.reset_pin_level());
            }
        });

        defmt::unwrap!(stusb4500_periodic::spawn_after(500u64.millis()));
    }

    /// Handles a new power supply report from the STUSB4500.
    #[task(shared = [i2c2, stusb4500, bq25713, power_supply, irq])]
    fn stusb4500_handle_report(cx: stusb4500_handle_report::Context) {
        (cx.shared.i2c2, cx.shared.stusb4500, cx.shared.bq25713, cx.shared.power_supply, cx.shared.irq).lock(
            |i2c2, stusb4500, bq25713, power_supply, irq| {
                let i2c2 = defmt::unwrap!(i2c2.as_mut());

                let report = defmt::unwrap!(stusb4500.as_mut()).report();
                defmt::info!("STUSB4500 power supply report: {:?}", &report);

                if let Some(bq25713) = bq25713.as_mut() {
                    defmt::info!("Setting BQ25713 maximum input current to {} mA", report.max_current_ma());
                    if let Err(err) =
                        bq25713.set_max_input_current(i2c2, report.max_current_ma()).and_then(|_| {
                            if report.max_current_ma() > 0 && !bq25713.is_charge_enabled() {
                                bq25713.set_charge_enable(i2c2, true)?;
                            } else if report.max_current_ma() == 0 && bq25713.is_charge_enabled() {
                                bq25713.set_charge_enable(i2c2, false)?;
                            }
                            Ok(())
                        })
                    {
                        defmt::error!("Cannot configure BQ25713 charging: {}", err);
                    }
                }

                irq.pend_soft(IrqState::SUPPLY | IrqState::BATTERY);

                *power_supply = Some(report);
            },
        );
    }

    /// BQ25713 periodic task.
    #[task(shared = [i2c2, bq25713, battery, irq], local = [bq25713_first_periodic])]
    fn bq25713_periodic(cx: bq25713_periodic::Context) {
        let grace_period = 10u64.secs();
        let first = cx.local.bq25713_first_periodic.get_or_insert_with(monotonics::now);

        (cx.shared.i2c2, cx.shared.bq25713, cx.shared.battery, cx.shared.irq).lock(
            |i2c2, bq25713, battery, irq| {
                if let Some(bq25713) = bq25713.as_mut() {
                    let i2c2 = defmt::unwrap!(i2c2.as_mut());

                    if let Err(err) = bq25713.periodic(i2c2) {
                        defmt::error!("BQ25713 periodic handling failed: {:?}", err);
                    }

                    let status = bq25713.status();
                    if status.has_any_fault() {
                        defmt::warn!("BQ25713 reported a fault: {:?}", bq25713.status());
                    }

                    match (bq25713.measurement(), ThisBoard::CRITICAL_LOW_BATTERY_VOLTAGE) {
                        (Some(m), Some(min_mv))
                            if !status.ac_stat
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

                    if status.has_clearable_fault() {
                        defmt::info!("Clearing BQ25713 fault");
                        if let Err(err) = bq25713.clear_faults(i2c2) {
                            defmt::error!("Clearing BQ25713 faults failed: {}", err);
                        }
                    }

                    let report = bq25713.battery();
                    if battery.as_ref().map(|b| report.changed_significantly(b)).unwrap_or(true) {
                        defmt::info!("Battery report: {:?}", &report);
                        *battery = Some(report);

                        irq.pend_soft(IrqState::SUPPLY | IrqState::BATTERY);
                    }
                }
            },
        );

        defmt::unwrap!(bq25713_periodic::spawn_after(1u64.secs()));
    }

    /// Performs power off when battery has reached lower voltage limit.
    #[task(shared = [board], local = [undervoltage_blink_state, undervoltage_blink_count], capacity = 1)]
    fn undervoltage_power_off(mut cx: undervoltage_power_off::Context) {
        if *cx.local.undervoltage_blink_count < 100 {
            *cx.local.undervoltage_blink_state = !*cx.local.undervoltage_blink_state;
            *cx.local.undervoltage_blink_count += 1;
            cx.shared.board.lock(|board| board.set_power_led(*cx.local.undervoltage_blink_state));
            defmt::unwrap!(undervoltage_power_off::spawn_after(100u64.millis()));
        } else {
            defmt::warn!("Undervoltage power off");
            defmt::unwrap!(power_off::spawn());
        }
    }

    /// Controls the charging LED.
    #[task(shared = [battery, power_supply, board], local = [charge_blink_state, supplying_since])]
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
                (true, true, false) => *cx.local.charge_blink_state / 10 == 1,
                (true, true, true) if grace => true,
                (true, true, true) => *cx.local.charge_blink_state % 2 == 1,
                (true, false, _) => true,
            };
            board.set_charging_led(led);

            *cx.local.charge_blink_state = (*cx.local.charge_blink_state + 1) % 20;
        });
        defmt::unwrap!(charge_led::spawn_after(100u64.millis()));
    }

    /// Simulates interrupts on PD0 and PD1.
    #[task(shared = [irq])]
    fn simulate_pd_irqs(mut cx: simulate_pd_irqs::Context) {
        cx.shared.irq.lock(|irq| irq.simulate_gpio_pd_exti());
        defmt::unwrap!(simulate_pd_irqs::spawn_after(defmt::unwrap!(ThisBoard::SIMULATE_PD_IRQS)));
    }

    /// External interrupt handler 0, handling EXTI0 - EXTI15.
    #[task(binds = EXTI0, shared = [irq, board, i2c2, stusb4500])]
    fn exti0(mut cx: exti0::Context) {
        if cx.shared.board.lock(|board| board.check_stusb4500_alerting()) {
            defmt::trace!("STUSB4500 alert interrupt!");
            unwrap!(stusb4500_alert::spawn());
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
    #[task(binds = I2C1_ER, priority = 2)]
    fn i2c1_er(_cx: i2c1_er::Context) {
        NVIC::pend(Interrupt::I2C1_EV);
    }

    /// I2C event interrupt handler.
    #[task(binds = I2C1_EV,
        local = [i2c_reg_slave, copyright_offset, mfd_cell_index, pwm_timer_index, pwm_channel_index],
        shared = [start_bootloader, irq, ugpio, bkp, watchman, rtc, adc_buf, pwm_timers, board, power_supply, battery],
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
                cx.shared.irq.lock(|irq| value.set_u32(irq.get_exti_trigger_raising_edge()))
            }
            Event::Write { reg: reg::IRQ_EXTI_TRIGGER_RISING_EDGE, value } => {
                cx.shared.irq.lock(|irq| irq.set_exti_trigger_raising_edge(value.as_u32()))
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
                cx.shared.adc_buf.lock(|adc_buf| match adc_convert::spawn() {
                    Ok(()) => *adc_buf = None,
                    Err(()) => defmt::warn!("ADC conversion already in progress"),
                });
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
            Event::Read { reg: reg::PWM_TIMER_CHANNELS, value } => cx.shared.pwm_timers.lock(|pwm_timers| {
                if let Some(pwm_timer) = pwm_timers.get_mut(*cx.local.pwm_timer_index as usize) {
                    value.set_u8(pwm_timer.channel_count());
                }
            }),
            Event::Write { reg: reg::PWM_TIMER_REMAP, value } => cx.shared.pwm_timers.lock(|pwm_timers| {
                if let Some(pwm_timer) = pwm_timers.get_mut(*cx.local.pwm_timer_index as usize) {
                    pwm_timer.set_remap(value.as_u8());
                }
            }),
            Event::Write { reg: reg::PWM_TIMER_FREQUENCY, value } => cx.shared.pwm_timers.lock(|pwm_timers| {
                if let Some(pwm_timer) = pwm_timers.get_mut(*cx.local.pwm_timer_index as usize) {
                    pwm_timer.set_frequency(value.as_u32().Hz());
                }
            }),
            Event::Read { reg: reg::PWM_CHANNEL, value } => value.set_u8(*cx.local.pwm_channel_index),
            Event::Write { reg: reg::PWM_CHANNEL, value } => cx.shared.pwm_timers.lock(|pwm_timers| {
                let channel = value.as_u8();
                if let Some(pwm_timer) = pwm_timers.get_mut(*cx.local.pwm_timer_index as usize) {
                    if channel < pwm_timer.channel_count() {
                        *cx.local.pwm_channel_index = channel;
                    }
                }
            }),
            Event::Write { reg: reg::PWM_CHANNEL_DUTY_CYCLE, value } => {
                cx.shared.pwm_timers.lock(|pwm_timers| {
                    if let Some(pwm_timer) = pwm_timers.get_mut(*cx.local.pwm_timer_index as usize) {
                        pwm_timer.set_duty_cycle(*cx.local.pwm_channel_index, value.as_u16());
                    }
                });
            }
            Event::Write { reg: reg::PWM_CHANNEL_POLARITY, value } => {
                cx.shared.pwm_timers.lock(|pwm_timers| {
                    if let Some(pwm_timer) = pwm_timers.get_mut(*cx.local.pwm_timer_index as usize) {
                        pwm_timer.set_polarity(*cx.local.pwm_channel_index, value.as_u8() != 0);
                    }
                });
            }
            Event::Write { reg: reg::PWM_CHANNEL_OUTPUT, value } => {
                cx.shared.pwm_timers.lock(|pwm_timers| {
                    if let Some(pwm_timer) = pwm_timers.get_mut(*cx.local.pwm_timer_index as usize) {
                        pwm_timer.set_output(*cx.local.pwm_channel_index, value.as_u8() != 0);
                    }
                });
            }
            Event::Read { reg: reg::BATTERY_VOLTAGE, value } => {
                cx.shared.battery.lock(|battery| {
                    if let Some(battery) = battery {
                        value.set_u32(battery.voltage_mv);
                    }
                });
            }
            Event::Read { reg: reg::BATTERY_MIN_VOLTAGE, value } => {
                value.set_u32(ThisBoard::CRITICAL_LOW_BATTERY_VOLTAGE.unwrap_or_default());
            }
            Event::Read { reg: reg::BATTERY_MAX_VOLTAGE, value } => {
                cx.shared.battery.lock(|battery| {
                    if let Some(battery) = battery {
                        value.set_u32(battery.max_voltage_mv);
                    }
                });
            }
            Event::Read { reg: reg::BATTERY_CHARGING, value } => {
                cx.shared.battery.lock(|battery| {
                    if let Some(battery) = battery {
                        value.set_u8(battery.charging.into());
                    }
                });
            }
            Event::Read { reg: reg::BATTERY_CONSTANT_CHARGE_VOLTAGE, value } => {
                cx.shared.battery.lock(|battery| {
                    if let Some(battery) = battery {
                        value.set_u32(battery.max_voltage_mv);
                    }
                });
            }
            Event::Read { reg: reg::BATTERY_CONSTANT_CHARGE_CURRENT, value } => {
                cx.shared.battery.lock(|battery| {
                    if let Some(battery) = battery {
                        value.set_u32(battery.max_charge_current_ma);
                    }
                });
            }
            Event::Read { reg: reg::BATTERY_CURRENT, value } => {
                cx.shared.battery.lock(|battery| {
                    if let Some(battery) = battery {
                        value.set_u32(battery.current_ma as u32);
                    }
                });
            }
            Event::Read { reg: reg::BATTERY_SYSTEM_VOLTAGE, value } => {
                cx.shared.battery.lock(|battery| {
                    if let Some(battery) = battery {
                        value.set_u32(battery.system_voltage_mv);
                    }
                });
            }
            Event::Read { reg: reg::SUPPLY_TYPE, value } => {
                cx.shared.power_supply.lock(|supply| {
                    if let Some(supply) = supply {
                        value.set_u8(u8::from(&*supply));
                    }
                });
            }
            Event::Read { reg: reg::SUPPLY_REQUESTED_VOLTAGE, value } => {
                cx.shared.power_supply.lock(|supply| {
                    if let Some(supply) = supply {
                        value.set_u32(supply.voltage_mv());
                    }
                });
            }
            Event::Read { reg: reg::SUPPLY_MAX_CURRENT, value } => {
                cx.shared.power_supply.lock(|supply| {
                    if let Some(supply) = supply {
                        value.set_u32(supply.max_current_ma());
                    }
                });
            }
            Event::Read { reg: reg::SUPPLY_USB_COMMUNICATION, value } => {
                cx.shared.power_supply.lock(|supply| {
                    if let Some(supply) = supply {
                        value.set_u8(supply.communication().into());
                    }
                });
            }
            Event::Read { reg: reg::SUPPLY_VOLTAGE, value } => {
                cx.shared.battery.lock(|battery| {
                    if let Some(battery) = battery {
                        value.set_u32(battery.input_voltage_mv);
                    }
                });
            }
            Event::Read { reg: reg::SUPPLY_CURRENT, value } => {
                cx.shared.battery.lock(|battery| {
                    if let Some(battery) = battery {
                        value.set_u32(battery.input_current_ma);
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
