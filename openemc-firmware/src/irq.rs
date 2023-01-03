//! Interrupt handling.

use cortex_m::{interrupt::free, peripheral::NVIC};
use stm32f1::stm32f103::{Interrupt, Peripherals};

use crate::pio::MaskedGpio;

/// IRQ state.
///
/// Interrupt is signalled by low level on IRQ pin.
///
/// Interrupt bit assignment:
///    * Bits 0 - 15 correspond to the EXTI GPIO lines
///    * Bit 17 is the RTC alarm interrupt
///    * Bit 20 is the battery interrupt
///    * Bit 21 is the supply interrupt
pub struct IrqState<const PORTS: usize> {
    /// Gpio for signalling any IRQ to I2C master.
    irq_pin_io: MaskedGpio<PORTS>,
    /// Current IRQ pin level.
    irq_pin_level: bool,
    /// High level trigger mask for EXTI lines.
    trig_high_level: u16,
    /// Low level trigger mask for EXTI lines.
    trig_low_level: u16,
    /// Mask of EXTI lines controlled by this.
    controlled: u32,
    /// PD GPIO values for interrupt simulation.
    pd: u16,
    /// Interrupt mask.
    mask: u32,
    /// Pending software interrupts.
    soft_pending: u32,
}

impl<const PORTS: usize> IrqState<PORTS> {
    /// Mask corresponding to EXTI lines 0 - 15 used for GPIO interrupts.
    const EXTI_GPIO_MASK: u32 = 0xffff;

    /// RTC interrupt flag.
    pub const RTC_ALARM: u32 = 1 << 17;

    /// Battery interrupt flag.
    pub const BATTERY: u32 = 1 << 20;

    /// Supply interrupt flag.
    pub const SUPPLY: u32 = 1 << 21;

    /// Initializes the IRQ state.
    pub fn new(irq_pin: u8, irq_pin_cfg: u8, controlled: u32) -> Self {
        // Configure IRQ pin.
        let mut irq_mask = [0u16; PORTS];
        irq_mask[irq_pin as usize / 16] = 1 << (irq_pin % 16);
        let mut irq_pin_io = MaskedGpio::new(irq_mask);

        let mut irq_cfg = [0u64; PORTS];
        irq_cfg[irq_pin as usize * 4 / 64] = (irq_pin_cfg as u64 & 0b1111) << (irq_pin * 4 % 64);
        irq_pin_io.set_cfg(&irq_cfg);

        let mut this = Self {
            irq_pin_io,
            irq_pin_level: false,
            trig_high_level: 0,
            trig_low_level: 0,
            controlled,
            pd: 0,
            mask: 0,
            soft_pending: 0,
        };

        this.set_irq_pin_level(true);
        this.set_mask(0);
        this.set_exti_trigger_falling_edge(0);
        this.set_exti_trigger_raising_edge(0);
        this.set_exti_gpio_source(0);
        this.get_pending_and_clear();

        this
    }

    /// Sets the level of the IRQ pin.
    fn set_irq_pin_level(&mut self, level: bool) {
        if level != self.irq_pin_level {
            self.irq_pin_io.set_out(if level { &[0xffff; PORTS] } else { &[0; PORTS] });
            self.irq_pin_level = level;

            defmt::debug!("IRQ pin becomes {}", if level { "high" } else { "low" });
        }
    }

    /// Gets the set of pending IRQs, clears them and resets the IRQ line.
    pub fn get_pending_and_clear(&mut self) -> u32 {
        let dp = unsafe { Peripherals::steal() };

        // Get and clear EXTI GPIO pending interrupts.
        let mut pending = dp.EXTI.pr.read().bits() & self.controlled;
        dp.EXTI.pr.write(|w| unsafe { w.bits(pending) });

        // Add pending software interrupts.
        pending |= self.soft_pending & self.mask;
        self.soft_pending &= !pending;

        self.set_irq_pin_level(true);

        pending
    }

    fn apply_controlled(&self, current: u32, desired: u32) -> u32 {
        current & !self.controlled | (desired & self.controlled)
    }

    /// Sets the IRQ mask (0=disabled, 1=enabled).
    pub fn set_mask(&mut self, mask: u32) {
        defmt::info!("IRQ mask: {:b}", mask);
        self.mask = mask;

        let dp = unsafe { Peripherals::steal() };
        free(|_| dp.EXTI.imr.modify(|r, w| unsafe { w.bits(self.apply_controlled(r.bits(), mask)) }));

        // Trigger software interrupts that may have become unmasked.
        self.pend_soft(0);
    }

    /// Gets the IRQ mask (0=disabled, 1=enabled).
    pub fn get_mask(&self) -> u32 {
        self.mask
    }

    /// Sets the EXTI rising edge trigger mask (0=disabled, 1=enabled).
    pub fn set_exti_trigger_raising_edge(&mut self, mask: u32) {
        let dp = unsafe { Peripherals::steal() };
        defmt::info!("IRQ trigger raising edge: {:b}", mask);
        free(|_| dp.EXTI.rtsr.modify(|r, w| unsafe { w.bits(self.apply_controlled(r.bits(), mask)) }));
    }

    /// Gets the EXTI rising edge trigger mask (0=disabled, 1=enabled).
    pub fn get_exti_trigger_raising_edge(&self) -> u32 {
        let dp = unsafe { Peripherals::steal() };
        dp.EXTI.rtsr.read().bits()
    }

    /// Sets the EXTI falling edge trigger mask (0=disabled, 1=enabled).
    pub fn set_exti_trigger_falling_edge(&mut self, mask: u32) {
        let dp = unsafe { Peripherals::steal() };
        defmt::info!("IRQ trigger falling edge: {:b}", mask);
        free(|_| dp.EXTI.ftsr.modify(|r, w| unsafe { w.bits(self.apply_controlled(r.bits(), mask)) }));
    }

    /// Gets the EXTI falling edge trigger mask (0=disabled, 1=enabled).
    pub fn get_exti_trigger_falling_edge(&self) -> u32 {
        let dp = unsafe { Peripherals::steal() };
        dp.EXTI.ftsr.read().bits()
    }

    /// Sets the EXTI GPIO high level trigger mask (0=disabled, 1=enabled).
    pub fn set_exti_gpio_trigger_high_level(&mut self, mask: u16) {
        self.trig_high_level = mask;
    }

    /// Gets the EXTI GPIO high level trigger mask (0=disabled, 1=enabled).
    pub fn get_exti_gpio_trigger_high_level(&self) -> u16 {
        self.trig_high_level
    }

    /// Sets the EXTI GPIO low level trigger mask (0=disabled, 1=enabled).
    pub fn set_exti_gpio_trigger_low_level(&mut self, mask: u16) {
        self.trig_low_level = mask;
    }

    /// Gets the EXTI GPIO low level trigger mask (0=disabled, 1=enabled).
    pub fn get_exti_gpio_trigger_low_level(&self) -> u16 {
        self.trig_low_level
    }

    /// Sets the EXTI line sources.
    ///
    /// 0000 - PA
    /// 0001 - PB
    /// 0010 - PC
    /// 0011 - PD
    pub fn set_exti_gpio_source(&mut self, src: u64) {
        let dp = unsafe { Peripherals::steal() };

        let mut controlled = 0;
        for n in 0..u32::BITS {
            if self.controlled & (1 << n) != 0 {
                controlled |= 0b1111 << (4 * n);
            }
        }

        free(|_| {
            let current = self.get_exti_gpio_source();
            let new = current & !controlled | (src & controlled);

            defmt::info!("EXTI GPIO source: {:b}", new);

            dp.AFIO.exticr1.write(|w| unsafe { w.bits(new as u32 & Self::EXTI_GPIO_MASK) });
            dp.AFIO.exticr2.write(|w| unsafe { w.bits((new >> 16) as u32 & Self::EXTI_GPIO_MASK) });
            dp.AFIO.exticr3.write(|w| unsafe { w.bits((new >> 32) as u32 & Self::EXTI_GPIO_MASK) });
            dp.AFIO.exticr4.write(|w| unsafe { w.bits((new >> 48) as u32 & Self::EXTI_GPIO_MASK) });
        });
    }

    /// Gets the EXTI line sources.
    ///
    /// 0000 - PA
    /// 0001 - PB
    /// 0010 - PC
    /// 0011 - PD
    pub fn get_exti_gpio_source(&self) -> u64 {
        let dp = unsafe { Peripherals::steal() };

        let a = dp.AFIO.exticr1.read().bits() & Self::EXTI_GPIO_MASK;
        let b = dp.AFIO.exticr2.read().bits() & Self::EXTI_GPIO_MASK;
        let c = dp.AFIO.exticr3.read().bits() & Self::EXTI_GPIO_MASK;
        let d = dp.AFIO.exticr4.read().bits() & Self::EXTI_GPIO_MASK;

        a as u64 | (b as u64) << 16 | (c as u64) << 32 | (d as u64) << 48
    }

    /// Triggers interrupts on EXTI lines with the specified levels.
    pub fn do_trigger_exti(&self, port_in: &[u16]) {
        let dp = unsafe { Peripherals::steal() };

        let srcs = self.get_exti_gpio_source();
        let mask = self.get_mask();
        let mut pend = 0;

        for n in 0..16 {
            if (mask >> n) & 1 != 0 {
                let src = (srcs >> (n * 4)) & 0b1111;
                let port = port_in.get(src as usize).cloned().unwrap_or_default();
                let level = (port >> n) & 1 != 0;

                if ((self.trig_high_level >> n) & 1 != 0 && level)
                    || ((self.trig_low_level >> n) & 1 != 0 && !level)
                {
                    pend |= 1 << n;
                }
            }
        }

        defmt::debug!("level trigger: {:b}", pend);

        dp.EXTI.swier.reset();
        dp.EXTI.swier.write(|w| unsafe { w.bits(pend) });
        dp.EXTI.swier.reset();
    }

    /// Pends an interrupt request from an EXTI line.
    pub fn pend_gpio_exti(&mut self) {
        defmt::trace!("EXTI IRQ request");
        self.set_irq_pin_level(false);
    }

    /// Simulates interrupts on PD0 and PD1.
    pub fn simulate_gpio_pd_exti(&mut self) {
        let dp = unsafe { Peripherals::steal() };
        let pd = dp.GPIOD.idr.read().bits() as u16;

        let mask = self.get_mask();
        let srcs = self.get_exti_gpio_source();
        let falling = self.get_exti_trigger_falling_edge();
        let raising = self.get_exti_trigger_raising_edge();

        let mut pend = 0;
        for n in 0..=1 {
            if mask & (1 << n) != 0 && srcs >> (n * 4) & 0b1111 == 0b0011 {
                if falling & (1 << n) != 0 && pd & (1 << n) == 0 && self.pd & (1 << n) != 0 {
                    pend |= 1 << n;
                }

                if raising & (1 << n) != 0 && pd & (1 << n) != 0 && self.pd & (1 << n) == 0 {
                    pend |= 1 << n;
                }
            }
        }

        self.pd = pd;

        if pend != 0 {
            defmt::debug!("simulated trigger: {:b}", pend);
        }

        dp.EXTI.swier.reset();
        dp.EXTI.swier.write(|w| unsafe { w.bits(pend) });
        dp.EXTI.swier.reset();
    }

    /// Pends software interrupts.
    pub fn pend_soft(&mut self, irqs: u32) {
        self.soft_pending |= irqs;

        if self.soft_pending & self.mask != 0 {
            defmt::trace!("software IRQ request: {:b}", self.soft_pending);
            self.set_irq_pin_level(false);
        }
    }
}

/// Masks all EXTI interrupt lines in the NVIC.
pub fn mask_exti() {
    NVIC::mask(Interrupt::EXTI0);
    NVIC::mask(Interrupt::EXTI1);
    NVIC::mask(Interrupt::EXTI2);
    NVIC::mask(Interrupt::EXTI3);
    NVIC::mask(Interrupt::EXTI4);
    NVIC::mask(Interrupt::EXTI9_5);
    NVIC::mask(Interrupt::EXTI15_10);
    NVIC::mask(Interrupt::RTCALARM);
}

/// Unmasks all EXTI interrupt lines in the NVIC.
pub unsafe fn unmask_exti() {
    NVIC::unmask(Interrupt::EXTI0);
    NVIC::unmask(Interrupt::EXTI1);
    NVIC::unmask(Interrupt::EXTI2);
    NVIC::unmask(Interrupt::EXTI3);
    NVIC::unmask(Interrupt::EXTI4);
    NVIC::unmask(Interrupt::EXTI9_5);
    NVIC::unmask(Interrupt::EXTI15_10);
    NVIC::unmask(Interrupt::RTCALARM);
}
