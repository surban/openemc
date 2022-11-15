//! Pulse-width modulation (PWM).

use stm32f1::stm32f103::Peripherals;
use stm32f1xx_hal::{rcc::Clocks, time::Hertz};

/// Timer instance.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Timer {
    /// Advanced-control timer 1
    Timer1,
    /// General-purpose timer 2
    Timer2,
    /// General-purpose timer 3
    Timer3,
    /// General-purpose timer 4
    Timer4,
}

/// Timer channel.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    /// Channel 1
    Ch1,
    /// Channel 2
    Ch2,
    /// Channel 3
    Ch3,
    /// Channel 4
    Ch4,
}

impl TryFrom<u8> for Channel {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Ch1),
            1 => Ok(Self::Ch2),
            2 => Ok(Self::Ch3),
            3 => Ok(Self::Ch4),
            _ => Err(()),
        }
    }
}

impl From<Channel> for u8 {
    fn from(channel: Channel) -> Self {
        match channel {
            Channel::Ch1 => 0,
            Channel::Ch2 => 1,
            Channel::Ch3 => 2,
            Channel::Ch4 => 3,
        }
    }
}

impl Channel {
    /// Channel count.
    pub const fn count() -> usize {
        4
    }
}

impl Timer {
    fn set_remap(&self, value: u8) {
        let dp = unsafe { Peripherals::steal() };

        dp.RCC.apb2enr.modify(|_, w| w.afioen().enabled());

        match self {
            Self::Timer1 => dp.AFIO.mapr.modify(|_, w| unsafe { w.tim1_remap().bits(value) }),
            Self::Timer2 => dp.AFIO.mapr.modify(|_, w| unsafe { w.tim2_remap().bits(value) }),
            Self::Timer3 => dp.AFIO.mapr.modify(|_, w| unsafe { w.tim3_remap().bits(value) }),
            Self::Timer4 => dp.AFIO.mapr.modify(|_, w| w.tim4_remap().bit(value & 0b1 != 0)),
        }
    }

    fn set_enable(&self, enable: bool) {
        let dp = unsafe { Peripherals::steal() };

        if enable {
            dp.RCC.apb2enr.modify(|_, w| w.afioen().enabled());
        }

        match self {
            Self::Timer1 => {
                dp.RCC.apb2enr.modify(|_, w| w.tim1en().bit(enable));
                dp.TIM1.bdtr.modify(|_, w| w.moe().bit(enable));
                dp.TIM1.cr1.modify(|_, w| w.cen().bit(enable).arpe().enabled());
            }
            Self::Timer2 => {
                dp.RCC.apb1enr.modify(|_, w| w.tim2en().bit(enable));
                dp.TIM2.cr1.modify(|_, w| w.cen().bit(enable).arpe().enabled());
            }
            Self::Timer3 => {
                dp.RCC.apb1enr.modify(|_, w| w.tim3en().bit(enable));
                dp.TIM3.cr1.modify(|_, w| w.cen().bit(enable).arpe().enabled());
            }
            Self::Timer4 => {
                dp.RCC.apb1enr.modify(|_, w| w.tim4en().bit(enable));
                dp.TIM4.cr1.modify(|_, w| w.cen().bit(enable).arpe().enabled());
            }
        }
    }

    fn clock(&self, clocks: &Clocks) -> Hertz {
        match self {
            Self::Timer1 => clocks.pclk2_tim(),
            Self::Timer2 => clocks.pclk1_tim(),
            Self::Timer3 => clocks.pclk1_tim(),
            Self::Timer4 => clocks.pclk1_tim(),
        }
    }

    fn set_prescaler(&self, prescaler: u16) {
        let dp = unsafe { Peripherals::steal() };
        match self {
            Self::Timer1 => dp.TIM1.psc.modify(|_, w| w.psc().bits(prescaler)),
            Self::Timer2 => dp.TIM2.psc.modify(|_, w| w.psc().bits(prescaler)),
            Self::Timer3 => dp.TIM3.psc.modify(|_, w| w.psc().bits(prescaler)),
            Self::Timer4 => dp.TIM4.psc.modify(|_, w| w.psc().bits(prescaler)),
        }
    }

    fn set_auto_reload(&self, auto_reload: u16) {
        let dp = unsafe { Peripherals::steal() };
        match self {
            Self::Timer1 => dp.TIM1.arr.modify(|_, w| w.arr().bits(auto_reload)),
            Self::Timer2 => dp.TIM2.arr.modify(|_, w| w.arr().bits(auto_reload)),
            Self::Timer3 => dp.TIM3.arr.modify(|_, w| w.arr().bits(auto_reload)),
            Self::Timer4 => dp.TIM4.arr.modify(|_, w| w.arr().bits(auto_reload)),
        }
    }

    fn update(&self) {
        let dp = unsafe { Peripherals::steal() };
        match self {
            Self::Timer1 => dp.TIM1.egr.write(|w| w.ug().update()),
            Self::Timer2 => dp.TIM2.egr.write(|w| w.ug().update()),
            Self::Timer3 => dp.TIM3.egr.write(|w| w.ug().update()),
            Self::Timer4 => dp.TIM4.egr.write(|w| w.ug().update()),
        }
    }

    fn set_pwm(&self, channel: &Channel, mode: bool) {
        let dp = unsafe { Peripherals::steal() };
        match (self, channel) {
            (Self::Timer1, Channel::Ch1) => dp
                .TIM1
                .ccmr1_output()
                .modify(|_, w| if mode { w.oc1m().pwm_mode1() } else { w.oc1m().pwm_mode2() }.oc1pe().enabled()),
            (Self::Timer1, Channel::Ch2) => dp
                .TIM1
                .ccmr1_output()
                .modify(|_, w| if mode { w.oc2m().pwm_mode1() } else { w.oc2m().pwm_mode2() }.oc2pe().enabled()),
            (Self::Timer1, Channel::Ch3) => dp
                .TIM1
                .ccmr2_output()
                .modify(|_, w| if mode { w.oc3m().pwm_mode1() } else { w.oc3m().pwm_mode2() }.oc3pe().enabled()),
            (Self::Timer1, Channel::Ch4) => dp
                .TIM1
                .ccmr2_output()
                .modify(|_, w| if mode { w.oc4m().pwm_mode1() } else { w.oc4m().pwm_mode2() }.oc4pe().enabled()),
            (Self::Timer2, Channel::Ch1) => dp
                .TIM2
                .ccmr1_output()
                .modify(|_, w| if mode { w.oc1m().pwm_mode1() } else { w.oc1m().pwm_mode2() }.oc1pe().enabled()),
            (Self::Timer2, Channel::Ch2) => dp
                .TIM2
                .ccmr1_output()
                .modify(|_, w| if mode { w.oc2m().pwm_mode1() } else { w.oc2m().pwm_mode2() }.oc2pe().enabled()),
            (Self::Timer2, Channel::Ch3) => dp
                .TIM2
                .ccmr2_output()
                .modify(|_, w| if mode { w.oc3m().pwm_mode1() } else { w.oc3m().pwm_mode2() }.oc3pe().enabled()),
            (Self::Timer2, Channel::Ch4) => dp
                .TIM2
                .ccmr2_output()
                .modify(|_, w| if mode { w.oc4m().pwm_mode1() } else { w.oc4m().pwm_mode2() }.oc4pe().enabled()),
            (Self::Timer3, Channel::Ch1) => dp
                .TIM3
                .ccmr1_output()
                .modify(|_, w| if mode { w.oc1m().pwm_mode1() } else { w.oc1m().pwm_mode2() }.oc1pe().enabled()),
            (Self::Timer3, Channel::Ch2) => dp
                .TIM3
                .ccmr1_output()
                .modify(|_, w| if mode { w.oc2m().pwm_mode1() } else { w.oc2m().pwm_mode2() }.oc2pe().enabled()),
            (Self::Timer3, Channel::Ch3) => dp
                .TIM3
                .ccmr2_output()
                .modify(|_, w| if mode { w.oc3m().pwm_mode1() } else { w.oc3m().pwm_mode2() }.oc3pe().enabled()),
            (Self::Timer3, Channel::Ch4) => dp
                .TIM3
                .ccmr2_output()
                .modify(|_, w| if mode { w.oc4m().pwm_mode1() } else { w.oc4m().pwm_mode2() }.oc4pe().enabled()),
            (Self::Timer4, Channel::Ch1) => dp
                .TIM4
                .ccmr1_output()
                .modify(|_, w| if mode { w.oc1m().pwm_mode1() } else { w.oc1m().pwm_mode2() }.oc1pe().enabled()),
            (Self::Timer4, Channel::Ch2) => dp
                .TIM4
                .ccmr1_output()
                .modify(|_, w| if mode { w.oc2m().pwm_mode1() } else { w.oc2m().pwm_mode2() }.oc2pe().enabled()),
            (Self::Timer4, Channel::Ch3) => dp
                .TIM4
                .ccmr2_output()
                .modify(|_, w| if mode { w.oc3m().pwm_mode1() } else { w.oc3m().pwm_mode2() }.oc3pe().enabled()),
            (Self::Timer4, Channel::Ch4) => dp
                .TIM4
                .ccmr2_output()
                .modify(|_, w| if mode { w.oc4m().pwm_mode1() } else { w.oc4m().pwm_mode2() }.oc4pe().enabled()),
        }
    }

    fn set_compare(&self, channel: &Channel, value: u16) {
        let dp = unsafe { Peripherals::steal() };
        match (self, channel) {
            (Self::Timer1, Channel::Ch1) => dp.TIM1.ccr1.modify(|_, w| w.ccr().bits(value)),
            (Self::Timer1, Channel::Ch2) => dp.TIM1.ccr2.modify(|_, w| w.ccr().bits(value)),
            (Self::Timer1, Channel::Ch3) => dp.TIM1.ccr3.modify(|_, w| w.ccr().bits(value)),
            (Self::Timer1, Channel::Ch4) => dp.TIM1.ccr4.modify(|_, w| w.ccr().bits(value)),
            (Self::Timer2, Channel::Ch1) => dp.TIM2.ccr1.modify(|_, w| w.ccr().bits(value)),
            (Self::Timer2, Channel::Ch2) => dp.TIM2.ccr2.modify(|_, w| w.ccr().bits(value)),
            (Self::Timer2, Channel::Ch3) => dp.TIM2.ccr3.modify(|_, w| w.ccr().bits(value)),
            (Self::Timer2, Channel::Ch4) => dp.TIM2.ccr4.modify(|_, w| w.ccr().bits(value)),
            (Self::Timer3, Channel::Ch1) => dp.TIM3.ccr1.modify(|_, w| w.ccr().bits(value)),
            (Self::Timer3, Channel::Ch2) => dp.TIM3.ccr2.modify(|_, w| w.ccr().bits(value)),
            (Self::Timer3, Channel::Ch3) => dp.TIM3.ccr3.modify(|_, w| w.ccr().bits(value)),
            (Self::Timer3, Channel::Ch4) => dp.TIM3.ccr4.modify(|_, w| w.ccr().bits(value)),
            (Self::Timer4, Channel::Ch1) => dp.TIM4.ccr1.modify(|_, w| w.ccr().bits(value)),
            (Self::Timer4, Channel::Ch2) => dp.TIM4.ccr2.modify(|_, w| w.ccr().bits(value)),
            (Self::Timer4, Channel::Ch3) => dp.TIM4.ccr3.modify(|_, w| w.ccr().bits(value)),
            (Self::Timer4, Channel::Ch4) => dp.TIM4.ccr4.modify(|_, w| w.ccr().bits(value)),
        }
    }

    fn set_output_enable(&self, channel: &Channel, enable: bool) {
        let dp = unsafe { Peripherals::steal() };
        match (self, channel) {
            (Self::Timer1, Channel::Ch1) => dp.TIM1.ccer.modify(|_, w| w.cc1e().bit(enable).cc1ne().bit(enable)),
            (Self::Timer1, Channel::Ch2) => dp.TIM1.ccer.modify(|_, w| w.cc2e().bit(enable).cc2ne().bit(enable)),
            (Self::Timer1, Channel::Ch3) => dp.TIM1.ccer.modify(|_, w| w.cc3e().bit(enable).cc3ne().bit(enable)),
            (Self::Timer1, Channel::Ch4) => dp.TIM1.ccer.modify(|_, w| w.cc4e().bit(enable)),
            (Self::Timer2, Channel::Ch1) => dp.TIM2.ccer.modify(|_, w| w.cc1e().bit(enable)),
            (Self::Timer2, Channel::Ch2) => dp.TIM2.ccer.modify(|_, w| w.cc2e().bit(enable)),
            (Self::Timer2, Channel::Ch3) => dp.TIM2.ccer.modify(|_, w| w.cc3e().bit(enable)),
            (Self::Timer2, Channel::Ch4) => dp.TIM2.ccer.modify(|_, w| w.cc4e().bit(enable)),
            (Self::Timer3, Channel::Ch1) => dp.TIM3.ccer.modify(|_, w| w.cc1e().bit(enable)),
            (Self::Timer3, Channel::Ch2) => dp.TIM3.ccer.modify(|_, w| w.cc2e().bit(enable)),
            (Self::Timer3, Channel::Ch3) => dp.TIM3.ccer.modify(|_, w| w.cc3e().bit(enable)),
            (Self::Timer3, Channel::Ch4) => dp.TIM3.ccer.modify(|_, w| w.cc4e().bit(enable)),
            (Self::Timer4, Channel::Ch1) => dp.TIM4.ccer.modify(|_, w| w.cc1e().bit(enable)),
            (Self::Timer4, Channel::Ch2) => dp.TIM4.ccer.modify(|_, w| w.cc2e().bit(enable)),
            (Self::Timer4, Channel::Ch3) => dp.TIM4.ccer.modify(|_, w| w.cc3e().bit(enable)),
            (Self::Timer4, Channel::Ch4) => dp.TIM4.ccer.modify(|_, w| w.cc4e().bit(enable)),
        }
    }

    fn set_output_polarity(&self, channel: &Channel, invert: bool) {
        let dp = unsafe { Peripherals::steal() };
        match (self, channel) {
            (Self::Timer1, Channel::Ch1) => dp.TIM1.ccer.modify(|_, w| w.cc1p().bit(invert).cc1np().bit(!invert)),
            (Self::Timer1, Channel::Ch2) => dp.TIM1.ccer.modify(|_, w| w.cc2p().bit(invert).cc2np().bit(!invert)),
            (Self::Timer1, Channel::Ch3) => dp.TIM1.ccer.modify(|_, w| w.cc3p().bit(invert).cc3np().bit(!invert)),
            (Self::Timer1, Channel::Ch4) => dp.TIM1.ccer.modify(|_, w| w.cc4p().bit(invert)),
            (Self::Timer2, Channel::Ch1) => dp.TIM2.ccer.modify(|_, w| w.cc1p().bit(invert)),
            (Self::Timer2, Channel::Ch2) => dp.TIM2.ccer.modify(|_, w| w.cc2p().bit(invert)),
            (Self::Timer2, Channel::Ch3) => dp.TIM2.ccer.modify(|_, w| w.cc3p().bit(invert)),
            (Self::Timer2, Channel::Ch4) => dp.TIM2.ccer.modify(|_, w| w.cc4p().bit(invert)),
            (Self::Timer3, Channel::Ch1) => dp.TIM3.ccer.modify(|_, w| w.cc1p().bit(invert)),
            (Self::Timer3, Channel::Ch2) => dp.TIM3.ccer.modify(|_, w| w.cc2p().bit(invert)),
            (Self::Timer3, Channel::Ch3) => dp.TIM3.ccer.modify(|_, w| w.cc3p().bit(invert)),
            (Self::Timer3, Channel::Ch4) => dp.TIM3.ccer.modify(|_, w| w.cc4p().bit(invert)),
            (Self::Timer4, Channel::Ch1) => dp.TIM4.ccer.modify(|_, w| w.cc1p().bit(invert)),
            (Self::Timer4, Channel::Ch2) => dp.TIM4.ccer.modify(|_, w| w.cc2p().bit(invert)),
            (Self::Timer4, Channel::Ch3) => dp.TIM4.ccer.modify(|_, w| w.cc3p().bit(invert)),
            (Self::Timer4, Channel::Ch4) => dp.TIM4.ccer.modify(|_, w| w.cc4p().bit(invert)),
        }
    }
}

/// PWM timer interface.
pub struct PwmTimer {
    timer: Timer,
    clocks: Clocks,
    arr: u32,
}

impl PwmTimer {
    /// Creates a new PMW timer interface and enables the timer.
    pub fn new(timer: Timer, clocks: &Clocks) -> Self {
        timer.set_enable(true);

        Self { timer, clocks: *clocks, arr: 0 }
    }

    /// Sets the pin remap configuration.
    ///
    /// 0 = no remap
    /// 1 = partial remap 1
    /// 2 = partial remap 2
    /// 3 = full remap
    pub fn set_remap(&mut self, remap: u8) {
        self.timer.set_remap(remap)
    }

    /// Sets the PWM frequency.
    ///
    /// The duty cycle must be re-set afterwards.
    pub fn set_frequency(&mut self, freq: Hertz) {
        let ticks = self.timer.clock(&self.clocks) / freq;
        let psc = ticks / (u16::MAX as u32 + 1);
        let arr = ticks / (psc + 1);

        self.arr = arr;

        self.timer.set_prescaler(psc as u16);
        self.timer.set_auto_reload(arr as u16);
        self.timer.update();
    }

    /// Sets the duty cycle.
    pub fn set_duty_cycle(&mut self, channel: &Channel, duty_cycle: u16) {
        let cmp = (self.arr * duty_cycle as u32 + u16::MAX as u32 / 2) / (u16::MAX as u32);

        self.timer.set_compare(channel, cmp as u16);
        self.timer.update();
    }

    /// Sets the output polarity.
    pub fn set_polarity(&mut self, channel: &Channel, invert: bool) {
        self.timer.set_output_polarity(channel, invert)
    }

    /// Enables or disables the PWM output.
    pub fn set_output(&mut self, channel: &Channel, enable: bool) {
        if enable {
            self.timer.set_pwm(channel, false);
        }
        self.timer.set_output_enable(channel, enable);
    }
}
