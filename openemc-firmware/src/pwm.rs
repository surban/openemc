//! Pulse-width modulation (PWM).

use defmt::Format;
use stm32f1::stm32f103::Peripherals;
use stm32f1xx_hal::{rcc::Clocks, time::Hertz};

/// Channels per timer.
const CHANNEL_COUNT: usize = 4;

/// Timer instance.
#[derive(Clone, Copy, PartialEq, Eq, Format)]
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

impl Timer {
    fn set_remap(&self, value: u8) {
        let dp = unsafe { Peripherals::steal() };

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

                if enable {
                    dp.RCC.apb2rstr.modify(|_, w| w.tim1rst().reset());
                    dp.RCC.apb2rstr.modify(|_, w| w.tim1rst().clear_bit());
                }

                dp.TIM1.bdtr.modify(|_, w| w.moe().bit(enable).ossr().idle_level().ossi().idle_level());
                dp.TIM1
                    .cr1
                    .modify(|_, w| w.cen().bit(enable).arpe().enabled().urs().counter_only().udis().enabled());
            }
            Self::Timer2 => {
                dp.RCC.apb1enr.modify(|_, w| w.tim2en().bit(enable));

                if enable {
                    dp.RCC.apb1rstr.modify(|_, w| w.tim2rst().reset());
                    dp.RCC.apb1rstr.modify(|_, w| w.tim2rst().clear_bit());
                }

                dp.TIM2
                    .cr1
                    .modify(|_, w| w.arpe().enabled().urs().counter_only().udis().enabled().cen().bit(enable));
            }
            Self::Timer3 => {
                dp.RCC.apb1enr.modify(|_, w| w.tim3en().bit(enable));

                if enable {
                    dp.RCC.apb1rstr.modify(|_, w| w.tim3rst().reset());
                    dp.RCC.apb1rstr.modify(|_, w| w.tim3rst().clear_bit());
                }

                dp.TIM3
                    .cr1
                    .modify(|_, w| w.arpe().enabled().urs().counter_only().udis().enabled().cen().bit(enable));
            }
            Self::Timer4 => {
                dp.RCC.apb1enr.modify(|_, w| w.tim4en().bit(enable));

                if enable {
                    dp.RCC.apb1rstr.modify(|_, w| w.tim4rst().reset());
                    dp.RCC.apb1rstr.modify(|_, w| w.tim4rst().clear_bit());
                }

                dp.TIM4
                    .cr1
                    .modify(|_, w| w.arpe().enabled().urs().counter_only().udis().enabled().cen().bit(enable));
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
            Self::Timer1 => dp.TIM1.psc.write(|w| w.psc().bits(prescaler)),
            Self::Timer2 => dp.TIM2.psc.write(|w| w.psc().bits(prescaler)),
            Self::Timer3 => dp.TIM3.psc.write(|w| w.psc().bits(prescaler)),
            Self::Timer4 => dp.TIM4.psc.write(|w| w.psc().bits(prescaler)),
        }
    }

    fn set_auto_reload(&self, auto_reload: u16) {
        let dp = unsafe { Peripherals::steal() };
        match self {
            Self::Timer1 => dp.TIM1.arr.write(|w| w.arr().bits(auto_reload)),
            Self::Timer2 => dp.TIM2.arr.write(|w| w.arr().bits(auto_reload)),
            Self::Timer3 => dp.TIM3.arr.write(|w| w.arr().bits(auto_reload)),
            Self::Timer4 => dp.TIM4.arr.write(|w| w.arr().bits(auto_reload)),
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

    fn set_pwm(&self, channel: u8, mode: bool) {
        let dp = unsafe { Peripherals::steal() };
        match (self, channel) {
            (Self::Timer1, 0) => dp
                .TIM1
                .ccmr1_output()
                .modify(|_, w| if mode { w.oc1m().pwm_mode1() } else { w.oc1m().pwm_mode2() }.oc1pe().enabled()),
            (Self::Timer1, 1) => dp
                .TIM1
                .ccmr1_output()
                .modify(|_, w| if mode { w.oc2m().pwm_mode1() } else { w.oc2m().pwm_mode2() }.oc2pe().enabled()),
            (Self::Timer1, 2) => dp
                .TIM1
                .ccmr2_output()
                .modify(|_, w| if mode { w.oc3m().pwm_mode1() } else { w.oc3m().pwm_mode2() }.oc3pe().enabled()),
            (Self::Timer1, 3) => dp
                .TIM1
                .ccmr2_output()
                .modify(|_, w| if mode { w.oc4m().pwm_mode1() } else { w.oc4m().pwm_mode2() }.oc4pe().enabled()),
            (Self::Timer2, 0) => dp
                .TIM2
                .ccmr1_output()
                .modify(|_, w| if mode { w.oc1m().pwm_mode1() } else { w.oc1m().pwm_mode2() }.oc1pe().enabled()),
            (Self::Timer2, 1) => dp
                .TIM2
                .ccmr1_output()
                .modify(|_, w| if mode { w.oc2m().pwm_mode1() } else { w.oc2m().pwm_mode2() }.oc2pe().enabled()),
            (Self::Timer2, 2) => dp
                .TIM2
                .ccmr2_output()
                .modify(|_, w| if mode { w.oc3m().pwm_mode1() } else { w.oc3m().pwm_mode2() }.oc3pe().enabled()),
            (Self::Timer2, 3) => dp
                .TIM2
                .ccmr2_output()
                .modify(|_, w| if mode { w.oc4m().pwm_mode1() } else { w.oc4m().pwm_mode2() }.oc4pe().enabled()),
            (Self::Timer3, 0) => dp
                .TIM3
                .ccmr1_output()
                .modify(|_, w| if mode { w.oc1m().pwm_mode1() } else { w.oc1m().pwm_mode2() }.oc1pe().enabled()),
            (Self::Timer3, 1) => dp
                .TIM3
                .ccmr1_output()
                .modify(|_, w| if mode { w.oc2m().pwm_mode1() } else { w.oc2m().pwm_mode2() }.oc2pe().enabled()),
            (Self::Timer3, 2) => dp
                .TIM3
                .ccmr2_output()
                .modify(|_, w| if mode { w.oc3m().pwm_mode1() } else { w.oc3m().pwm_mode2() }.oc3pe().enabled()),
            (Self::Timer3, 3) => dp
                .TIM3
                .ccmr2_output()
                .modify(|_, w| if mode { w.oc4m().pwm_mode1() } else { w.oc4m().pwm_mode2() }.oc4pe().enabled()),
            (Self::Timer4, 0) => dp
                .TIM4
                .ccmr1_output()
                .modify(|_, w| if mode { w.oc1m().pwm_mode1() } else { w.oc1m().pwm_mode2() }.oc1pe().enabled()),
            (Self::Timer4, 1) => dp
                .TIM4
                .ccmr1_output()
                .modify(|_, w| if mode { w.oc2m().pwm_mode1() } else { w.oc2m().pwm_mode2() }.oc2pe().enabled()),
            (Self::Timer4, 2) => dp
                .TIM4
                .ccmr2_output()
                .modify(|_, w| if mode { w.oc3m().pwm_mode1() } else { w.oc3m().pwm_mode2() }.oc3pe().enabled()),
            (Self::Timer4, 3) => dp
                .TIM4
                .ccmr2_output()
                .modify(|_, w| if mode { w.oc4m().pwm_mode1() } else { w.oc4m().pwm_mode2() }.oc4pe().enabled()),
            _ => defmt::panic!("invalid channel"),
        }
    }

    fn set_compare(&self, channel: u8, value: u16) {
        let dp = unsafe { Peripherals::steal() };
        match (self, channel) {
            (Self::Timer1, 0) => dp.TIM1.ccr1().write(|w| w.ccr().bits(value)),
            (Self::Timer1, 1) => dp.TIM1.ccr2().write(|w| w.ccr().bits(value)),
            (Self::Timer1, 2) => dp.TIM1.ccr3().write(|w| w.ccr().bits(value)),
            (Self::Timer1, 3) => dp.TIM1.ccr4().write(|w| w.ccr().bits(value)),
            (Self::Timer2, 0) => dp.TIM2.ccr1().write(|w| w.ccr().bits(value)),
            (Self::Timer2, 1) => dp.TIM2.ccr2().write(|w| w.ccr().bits(value)),
            (Self::Timer2, 2) => dp.TIM2.ccr3().write(|w| w.ccr().bits(value)),
            (Self::Timer2, 3) => dp.TIM2.ccr4().write(|w| w.ccr().bits(value)),
            (Self::Timer3, 0) => dp.TIM3.ccr1().write(|w| w.ccr().bits(value)),
            (Self::Timer3, 1) => dp.TIM3.ccr2().write(|w| w.ccr().bits(value)),
            (Self::Timer3, 2) => dp.TIM3.ccr3().write(|w| w.ccr().bits(value)),
            (Self::Timer3, 3) => dp.TIM3.ccr4().write(|w| w.ccr().bits(value)),
            (Self::Timer4, 0) => dp.TIM4.ccr1().write(|w| w.ccr().bits(value)),
            (Self::Timer4, 1) => dp.TIM4.ccr2().write(|w| w.ccr().bits(value)),
            (Self::Timer4, 2) => dp.TIM4.ccr3().write(|w| w.ccr().bits(value)),
            (Self::Timer4, 3) => dp.TIM4.ccr4().write(|w| w.ccr().bits(value)),
            _ => defmt::panic!("invalid channel"),
        }
    }

    fn set_output_enable(&self, channel: u8, enable: bool) {
        let dp = unsafe { Peripherals::steal() };
        match (self, channel) {
            (Self::Timer1, 0) => dp.TIM1.ccer.modify(|_, w| w.cc1e().bit(enable).cc1ne().bit(enable)),
            (Self::Timer1, 1) => dp.TIM1.ccer.modify(|_, w| w.cc2e().bit(enable).cc2ne().bit(enable)),
            (Self::Timer1, 2) => dp.TIM1.ccer.modify(|_, w| w.cc3e().bit(enable).cc3ne().bit(enable)),
            (Self::Timer1, 3) => dp.TIM1.ccer.modify(|_, w| w.cc4e().bit(enable)),
            (Self::Timer2, 0) => dp.TIM2.ccer.modify(|_, w| w.cc1e().bit(enable)),
            (Self::Timer2, 1) => dp.TIM2.ccer.modify(|_, w| w.cc2e().bit(enable)),
            (Self::Timer2, 2) => dp.TIM2.ccer.modify(|_, w| w.cc3e().bit(enable)),
            (Self::Timer2, 3) => dp.TIM2.ccer.modify(|_, w| w.cc4e().bit(enable)),
            (Self::Timer3, 0) => dp.TIM3.ccer.modify(|_, w| w.cc1e().bit(enable)),
            (Self::Timer3, 1) => dp.TIM3.ccer.modify(|_, w| w.cc2e().bit(enable)),
            (Self::Timer3, 2) => dp.TIM3.ccer.modify(|_, w| w.cc3e().bit(enable)),
            (Self::Timer3, 3) => dp.TIM3.ccer.modify(|_, w| w.cc4e().bit(enable)),
            (Self::Timer4, 0) => dp.TIM4.ccer.modify(|_, w| w.cc1e().bit(enable)),
            (Self::Timer4, 1) => dp.TIM4.ccer.modify(|_, w| w.cc2e().bit(enable)),
            (Self::Timer4, 2) => dp.TIM4.ccer.modify(|_, w| w.cc3e().bit(enable)),
            (Self::Timer4, 3) => dp.TIM4.ccer.modify(|_, w| w.cc4e().bit(enable)),
            _ => defmt::panic!("invalid channel"),
        }
    }

    fn set_output_polarity(&self, channel: u8, invert: bool) {
        let dp = unsafe { Peripherals::steal() };
        match (self, channel) {
            (Self::Timer1, 0) => dp.TIM1.ccer.modify(|_, w| w.cc1p().bit(invert).cc1np().bit(invert)),
            (Self::Timer1, 1) => dp.TIM1.ccer.modify(|_, w| w.cc2p().bit(invert).cc2np().bit(invert)),
            (Self::Timer1, 2) => dp.TIM1.ccer.modify(|_, w| w.cc3p().bit(invert).cc3np().bit(invert)),
            (Self::Timer1, 3) => dp.TIM1.ccer.modify(|_, w| w.cc4p().bit(invert)),
            (Self::Timer2, 0) => dp.TIM2.ccer.modify(|_, w| w.cc1p().bit(invert)),
            (Self::Timer2, 1) => dp.TIM2.ccer.modify(|_, w| w.cc2p().bit(invert)),
            (Self::Timer2, 2) => dp.TIM2.ccer.modify(|_, w| w.cc3p().bit(invert)),
            (Self::Timer2, 3) => dp.TIM2.ccer.modify(|_, w| w.cc4p().bit(invert)),
            (Self::Timer3, 0) => dp.TIM3.ccer.modify(|_, w| w.cc1p().bit(invert)),
            (Self::Timer3, 1) => dp.TIM3.ccer.modify(|_, w| w.cc2p().bit(invert)),
            (Self::Timer3, 2) => dp.TIM3.ccer.modify(|_, w| w.cc3p().bit(invert)),
            (Self::Timer3, 3) => dp.TIM3.ccer.modify(|_, w| w.cc4p().bit(invert)),
            (Self::Timer4, 0) => dp.TIM4.ccer.modify(|_, w| w.cc1p().bit(invert)),
            (Self::Timer4, 1) => dp.TIM4.ccer.modify(|_, w| w.cc2p().bit(invert)),
            (Self::Timer4, 2) => dp.TIM4.ccer.modify(|_, w| w.cc3p().bit(invert)),
            (Self::Timer4, 3) => dp.TIM4.ccer.modify(|_, w| w.cc4p().bit(invert)),
            _ => defmt::panic!("invalid channel"),
        }
    }
}

/// PWM timer interface.
pub struct PwmTimer {
    timer: Timer,
    clocks: Clocks,
    arr: u32,
    freq: Hertz,
    duty_cycles: [u16; CHANNEL_COUNT],
}

impl PwmTimer {
    /// Creates a new PMW timer interface and enables the timer.
    pub fn new(timer: Timer, clocks: &Clocks) -> Self {
        timer.set_enable(true);

        for ch in 0..CHANNEL_COUNT {
            timer.set_pwm(ch as u8, true);
        }

        Self { timer, clocks: *clocks, arr: 0, freq: Hertz::Hz(0), duty_cycles: [0; CHANNEL_COUNT] }
    }

    /// Channel count.
    pub fn channel_count(&self) -> u8 {
        self.duty_cycles.len() as u8
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
    pub fn set_frequency(&mut self, freq: Hertz) {
        if self.freq == freq {
            return;
        }

        let ticks = self.timer.clock(&self.clocks) / freq;
        let psc = ticks / (u16::MAX as u32 + 1);
        self.arr = ticks / (psc + 1);
        self.freq = freq;

        defmt::debug!("setting {:?} prescaler={} reload={}", &self.timer, psc, self.arr);

        self.timer.set_prescaler(psc as u16);
        self.timer.set_auto_reload(self.arr as u16);
        self.timer.update();

        for (channel, &duty_cycle) in self.duty_cycles.clone().iter().enumerate() {
            self.set_duty_cycle(channel as u8, duty_cycle);
        }
    }

    /// Sets the duty cycle.
    pub fn set_duty_cycle(&mut self, channel: u8, duty_cycle: u16) {
        self.duty_cycles[channel as usize] = duty_cycle;

        let cmp = if duty_cycle >= u16::MAX - 2 {
            u16::MAX as u32
        } else {
            (self.arr * duty_cycle as u32 + u16::MAX as u32 / 2) / (u16::MAX as u32)
        };

        defmt::debug!("setting {:?} channel {} arr={} compare={}", &self.timer, channel, self.arr, cmp);
        self.timer.set_compare(channel, cmp as u16);
        self.timer.update();
    }

    /// Sets the output polarity.
    pub fn set_polarity(&mut self, channel: u8, invert: bool) {
        defmt::debug!("setting {:?} channel {} invert={:?}", &self.timer, channel, invert);
        self.timer.set_output_polarity(channel, invert)
    }

    /// Enables or disables the PWM output.
    pub fn set_output(&mut self, channel: u8, enable: bool) {
        defmt::debug!("setting {:?} channel {} enable={:?}", &self.timer, channel, enable);
        self.timer.set_output_enable(channel, enable);
    }
}
