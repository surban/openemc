//! Real-time clock (RTC).

use core::cell::Cell;
use defmt::Format;
use stm32f1::stm32f103::{rcc::bdcr::RTCSEL, Peripherals, RTC};
use stm32f1xx_hal::{backup_domain::BackupDomain, prelude::*};

use crate::{backup::BackupReg, Delay};

/// RTC clock source.
#[derive(Clone, Copy, PartialEq, Eq, Format)]
#[repr(u8)]
pub enum ClockSrc {
    /// External low-speed oscillator.
    Lse = 1,
    /// Internal low-speed osciallator (40 kHz ± 30%)
    Lsi = 2,
    /// HSE clock divided by 128.
    HseDiv128 = 3,
}

impl ClockSrc {
    /// Prescaler value for clock source.
    pub const fn prescaler(&self) -> Option<u32> {
        match self {
            ClockSrc::Lse => Some(32_768),
            ClockSrc::Lsi => Some(40_000),
            ClockSrc::HseDiv128 => None,
        }
    }

    /// Checks whether the clock source is ready.
    pub fn is_ready(&self) -> bool {
        let dp = unsafe { Peripherals::steal() };

        match self {
            Self::Lse => dp.RCC.bdcr().read().lserdy().is_ready(),
            Self::Lsi | Self::HseDiv128 => true,
        }
    }
}

/// Clock value.
#[derive(Clone, Copy, PartialEq, Eq, Format)]
pub struct Clock {
    /// Number of clock ticks. (32 bits)
    pub clock: u32,
    /// Number of prescaler ticks. (20 bits)
    pub prescaler: u32,
    /// Maximum value of prescaler. (20 bits)
    pub prescaler_max: u32,
}

impl Clock {
    /// Number of seconds.
    pub fn secs(&self) -> u32 {
        self.clock
    }

    /// Number of milliseconds since last whole second.
    pub fn millis(&self) -> u32 {
        // Accurate calculation is possible using u32 because prescaler and prescaler_max
        // are 20 bit values and 1000 < 1024 = 1 << 10.
        1000 * (self.prescaler_max - self.prescaler) / self.prescaler_max
    }
}

/// Real-time clock (RTC).
pub struct Rtc {
    rtc: RTC,
    prescaler: Option<u32>,
    /// Alarm time, mirrored in the backup registers.
    alarm: u32,
    /// Whether the alarm is armed, mirrored in the backup registers.
    alarm_armed: bool,
    alarm_at_init: Cell<Option<bool>>,
    alarm_at_init_silenced: bool,
}

impl Rtc {
    /// Negative prescaler offset for calibration abilty in both fast and slow direction.
    pub const PRESCALER_NEG_OFFSET: u32 = 2;

    /// Creates a new interface to the real-time clock and enables it.
    ///
    /// Waits for the RTC registers to synchronize, which only completes if the RTC is clocked.
    pub fn new(rtc: RTC, bkp: &mut BackupDomain, delay: &mut Delay) -> Self {
        let dp = unsafe { Peripherals::steal() };

        dp.RCC.bdcr().modify(|_, w| w.rtcen().enabled().lseon().on());
        rtc.crl().modify(|_, w| w.rsf().clear());

        let mut this = Self {
            rtc,
            prescaler: None,
            alarm: (BackupReg::RtcAlarmHigh.get(bkp) as u32) << 16 | BackupReg::RtcAlarmLow.get(bkp) as u32,
            alarm_armed: BackupReg::RtcAlarmArmed.get(bkp) != 0,
            alarm_at_init: Cell::new(None),
            alarm_at_init_silenced: false,
        };

        // Wait for RTC sync.
        delay.delay(10u32.millis());
        for _ in 0..100 {
            if this.is_synced() {
                break;
            }
            delay.delay(1u32.millis());
        }
        delay.delay(10u32.millis());
        for _ in 0..100 {
            this.read_counter();
        }
        if !this.is_synced() {
            defmt::warn!("RTC registers are not synchronized");
        }

        // Restore RTC alarm settings that might have been cleared by reset from backup registers.
        if this.restore_alarm(bkp).is_err() {
            defmt::warn!("cannot restore RTC alarm");
        }

        this
    }

    /// Returns whether the RTC is fully ready.
    pub fn is_ready(&self) -> bool {
        self.check_regs_synced().is_ok() && self.prescaler.is_some()
    }

    /// Restore RTC alarm settings from backup registers.
    pub fn restore_alarm(&mut self, bkp: &mut BackupDomain) -> Result<(), RtcNotReady> {
        self.set_alarm(self.alarm, bkp)?;
        if self.alarm_armed {
            self.listen_alarm(bkp)?;
        } else {
            self.unlisten_alarm(bkp)?;
        }
        Ok(())
    }

    /// Backups the backup domain, resets it and then restores its values,
    /// except for the RTC clock source, prescaler and alarm.
    fn reset_backup_domain(&mut self, bkp: &mut BackupDomain) {
        let dp = unsafe { Peripherals::steal() };

        defmt::info!("RTC resets backup domain");

        let slowdown = self.slowdown(bkp);
        let clock = self.clock();

        let mut regs = [0; 10];
        for (reg, v) in regs.iter_mut().enumerate() {
            *v = bkp.read_data_register_low(reg);
        }

        dp.RCC.bdcr().modify(|_, w| w.bdrst().enabled());
        dp.RCC.bdcr().modify(|_, w| w.bdrst().disabled());
        dp.RCC.bdcr().modify(|_, w| w.rtcen().enabled().lseon().on());

        for (reg, v) in regs.into_iter().enumerate() {
            bkp.write_data_register_low(reg, v);
        }

        // The alarm has been lost by the backup domain reset.
        BackupReg::RtcAlarmHigh.set(bkp, 0);
        BackupReg::RtcAlarmLow.set(bkp, 0);
        BackupReg::RtcAlarmArmed.set(bkp, 0);
        self.alarm = 0;
        self.alarm_armed = false;

        let _ = self.set_clock(clock.map(|c| c.clock).unwrap_or_default());
        self.set_slowdown(bkp, slowdown);
    }

    /// Gets the clock source of the RTC.
    pub fn clock_src(&self) -> Option<ClockSrc> {
        let dp = unsafe { Peripherals::steal() };

        match dp.RCC.bdcr().read().rtcsel().variant() {
            RTCSEL::NoClock => None,
            RTCSEL::Lse => Some(ClockSrc::Lse),
            RTCSEL::Lsi => Some(ClockSrc::Lsi),
            RTCSEL::Hse => Some(ClockSrc::HseDiv128),
        }
    }

    /// Sets the clock source of the RTC.
    ///
    /// If the clock source changes, prescaler and alarm must be set again afterwards.
    pub fn set_clock_src(&mut self, bkp: &mut BackupDomain, clock_src: ClockSrc) {
        let dp = unsafe { Peripherals::steal() };

        match self.clock_src() {
            Some(prev) if prev == clock_src => return,
            Some(_) => {
                defmt::info!("resetting backup domain for RTC clock source change");
                self.reset_backup_domain(bkp);
            }
            None => (),
        }

        defmt::info!("setting RTC clock source to {}", clock_src);

        if let ClockSrc::Lse = clock_src {
            dp.RCC.bdcr().modify(|_, w| w.lseon().on());
        } else {
            dp.RCC.bdcr().modify(|_, w| w.lseon().off());
        }

        dp.RCC.bdcr().modify(|_, w| {
            w.rtcsel().variant(match clock_src {
                ClockSrc::Lse => RTCSEL::Lse,
                ClockSrc::Lsi => RTCSEL::Lsi,
                ClockSrc::HseDiv128 => RTCSEL::Hse,
            })
        });
    }

    /// Gets the calibration slowdown value in clock pulses that will
    /// be ignored every 2^20 clock pulses.
    ///
    /// One unit causes a slowdown of 1_000_000/2^20 PPM ≈ 0.954 PPM.
    pub fn slowdown(&self, _bkp: &mut BackupDomain) -> u8 {
        let dp = unsafe { Peripherals::steal() };
        dp.BKP.rtccr().read().cal().bits()
    }

    /// Sets the calibration slowdown value in clock pulses that will
    /// be ignored every 2^20 clock pulses.
    ///
    /// One unit causes a slowdown of 1_000_000/2^20 PPM ≈ 0.954 PPM.
    pub fn set_slowdown(&mut self, _bkp: &mut BackupDomain, slowdown: u8) {
        defmt::info!("setting RTC slowdown to {}", slowdown);
        let dp = unsafe { Peripherals::steal() };
        dp.BKP.rtccr().modify(|_, w| unsafe { w.cal().bits(slowdown) });
    }

    /// Ensures that the RTC registers are synchronized and ready for modification.
    fn check_regs_synced(&self) -> Result<(), RtcNotReady> {
        let crl = self.rtc.crl().read();
        if crl.rsf().is_synchronized() && crl.rtoff().bit_is_set() {
            if self.alarm_at_init.get().is_none() {
                self.alarm_at_init.set(Some(self.is_alarm_pending_within(self.read_counter(), 0)));
            }
            Ok(())
        } else {
            Err(RtcNotReady)
        }
    }

    /// Modify RTC registers CNT, PRL or ALR.
    fn modify<T>(&mut self, modify_fn: impl FnOnce(&mut RTC) -> T) -> Result<T, RtcNotReady> {
        self.check_regs_synced()?;

        // Enable write access.
        self.rtc.crl().modify(|_, w| w.cnf().enter());

        // Perform modifications.
        let ret = modify_fn(&mut self.rtc);

        // Commit changes.
        self.rtc.crl().modify(|_, w| w.cnf().exit());

        Ok(ret)
    }

    /// Arms the alarm by enabling the RTC alarm interrupt.
    pub fn listen_alarm(&mut self, bkp: &mut BackupDomain) -> Result<(), RtcNotReady> {
        defmt::info!("enabling RTC alarm");
        self.alarm_armed = true;
        BackupReg::RtcAlarmArmed.set(bkp, 1);
        self.modify(|rtc| rtc.crh().modify(|_, w| w.alrie().enabled()))?;
        Ok(())
    }

    /// Disarms the alarm by disabling the RTC alarm interrupt.
    pub fn unlisten_alarm(&mut self, bkp: &mut BackupDomain) -> Result<(), RtcNotReady> {
        defmt::info!("disabling RTC alarm");
        self.alarm_armed = false;
        BackupReg::RtcAlarmArmed.set(bkp, 0);
        self.modify(|rtc| rtc.crh().modify(|_, w| w.alrie().disabled()))?;
        Ok(())
    }

    /// Returns whether the alarm is armed.
    pub fn is_alarm_listened(&self) -> bool {
        self.alarm_armed
    }

    /// Returns whether an alarm has occured and has not been silenced.
    pub fn is_alarming(&self) -> Result<bool, RtcNotReady> {
        self.check_regs_synced()?;
        Ok(self.rtc.crl().read().alrf().is_alarm()
            || (self.alarm_at_init.get() == Some(true) && !self.alarm_at_init_silenced))
    }

    /// Returns whether the alarm was pending when the RTC was initialized.
    pub fn was_alarming_at_init(&self) -> Result<bool, RtcNotReady> {
        self.check_regs_synced()?;
        Ok(self.alarm_at_init.get() == Some(true))
    }

    /// Returns whether the RTC registers are synchronized and can be accessed.
    pub fn is_synced(&self) -> bool {
        self.check_regs_synced().is_ok()
    }

    /// Returns whether the alarm is armed and its time has been reached.
    ///
    /// This does not depend on the alarm flag, since it is cleared by a system reset.
    pub fn is_alarm_pending(&self) -> Result<bool, RtcNotReady> {
        Ok(self.is_alarm_pending_within(self.counter()?, 0))
    }

    /// Returns whether the alarm is armed and its time has been reached or
    /// will be reached within the next two seconds.
    pub fn is_alarm_pending_soon(&self) -> Result<bool, RtcNotReady> {
        Ok(self.is_alarm_pending_within(self.counter()?, 2))
    }

    /// Returns whether the alarm is armed and its time will be reached within
    /// the specified number of seconds after the specified counter value.
    fn is_alarm_pending_within(&self, counter: u32, secs: u32) -> bool {
        self.alarm_armed && self.alarm != 0 && counter.saturating_add(secs) >= self.alarm
    }

    /// Clears the alarm.
    pub fn silence_alarm(&mut self) -> Result<(), RtcNotReady> {
        defmt::info!("silencing RTC alarm");
        self.modify(|rtc| rtc.crl().modify(|_, w| w.alrf().clear()))?;
        self.alarm_at_init_silenced = true;
        Ok(())
    }

    /// Gets the alarm time.
    pub fn alarm(&self) -> u32 {
        self.alarm
    }

    /// Gets the counter value.
    fn counter(&self) -> Result<u32, RtcNotReady> {
        self.check_regs_synced()?;
        Ok(self.read_counter())
    }

    /// Reads the counter value without checking for synchronization.
    fn read_counter(&self) -> u32 {
        (self.rtc.cnth().read().cnth().bits() as u32) << 16 | (self.rtc.cntl().read().cntl().bits() as u32)
    }

    /// Sets the alarm time.
    pub fn set_alarm(&mut self, alarm: u32, bkp: &mut BackupDomain) -> Result<(), RtcNotReady> {
        defmt::info!("setting RTC alarm to {}", alarm);

        self.modify(|rtc| {
            rtc.alrh().write(|w| w.alrh().set((alarm >> 16) as u16));
            rtc.alrl().write(|w| w.alrl().set(alarm as u16));
        })?;

        self.alarm = alarm;
        BackupReg::RtcAlarmHigh.set(bkp, (alarm >> 16) as u16);
        BackupReg::RtcAlarmLow.set(bkp, alarm as u16);

        Ok(())
    }

    /// Gets the clock value.
    pub fn clock(&self) -> Result<Clock, RtcNotReady> {
        self.check_regs_synced()?;

        match self.prescaler {
            Some(prescaler_max) => Ok(Clock {
                clock: self.read_counter(),
                prescaler: (self.rtc.divh().read().divh().bits() as u32) << 16
                    | (self.rtc.divl().read().divl().bits() as u32),
                prescaler_max,
            }),
            None => Err(RtcNotReady),
        }
    }

    /// Sets the clock value.
    pub fn set_clock(&mut self, clock: u32) -> Result<(), RtcNotReady> {
        defmt::info!("setting RTC to {}", clock);
        self.modify(|rtc| {
            rtc.cnth().write(|w| w.cnth().set((clock >> 16) as u16));
            rtc.cntl().write(|w| w.cntl().set(clock as u16));
        })
    }

    /// Sets the prescalar value.
    pub fn set_prescalar(&mut self, prescaler: u32) -> Result<(), RtcNotReady> {
        defmt::assert!(prescaler >= 2);

        defmt::info!("setting RTC prescaler to {}", prescaler);
        self.prescaler = Some(prescaler);

        let p = prescaler - 1;
        self.modify(|rtc| {
            rtc.prlh().write(|w| w.prlh().set((p >> 16) as u8));
            rtc.prll().write(|w| w.prll().set(p as u16));
        })
    }

    /// Gets the prescaler value.
    pub fn prescaler(&self) -> Option<u32> {
        self.prescaler
    }
}

/// RTC registers have not been synchronized.
#[derive(Copy, Clone, Format)]
pub struct RtcNotReady;
