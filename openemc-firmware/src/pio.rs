//! GPIO.

use stm32f1::{
    stm32f103::{
        gpioa::{crh::CRH_SPEC, crl::CRL_SPEC, idr::IDR_SPEC, odr::ODR_SPEC},
        Peripherals,
    },
    Reg,
};

/// Maximum number of ports.
#[allow(dead_code)]
pub const MAX_PORTS: usize = 7;

/// Masked GPIO.
pub struct MaskedGpio<const PORTS: usize> {
    dp: Peripherals,
    usable: [u16; PORTS],
}

impl<const PORTS: usize> MaskedGpio<PORTS> {
    /// Creates a new masked GPIO interface.
    pub fn new(usable: [u16; PORTS]) -> Self {
        Self { dp: unsafe { Peripherals::steal() }, usable }
    }

    /// Number of ports.
    pub const fn ports(&self) -> usize {
        PORTS
    }

    /// Number of GPIOs.
    pub const fn gpios(&self) -> usize {
        PORTS * 16
    }

    fn idr(&self, port: usize) -> &Reg<IDR_SPEC> {
        match port {
            0 => &self.dp.GPIOA.idr,
            1 => &self.dp.GPIOB.idr,
            2 => &self.dp.GPIOC.idr,
            3 => &self.dp.GPIOD.idr,
            4 => &self.dp.GPIOE.idr,
            5 => &self.dp.GPIOF.idr,
            6 => &self.dp.GPIOG.idr,
            _ => defmt::unreachable!(),
        }
    }

    fn odr(&self, port: usize) -> &Reg<ODR_SPEC> {
        match port {
            0 => &self.dp.GPIOA.odr,
            1 => &self.dp.GPIOB.odr,
            2 => &self.dp.GPIOC.odr,
            3 => &self.dp.GPIOD.odr,
            4 => &self.dp.GPIOE.odr,
            5 => &self.dp.GPIOF.odr,
            6 => &self.dp.GPIOG.odr,
            _ => defmt::unreachable!(),
        }
    }

    fn crh(&self, port: usize) -> &Reg<CRH_SPEC> {
        match port {
            0 => &self.dp.GPIOA.crh,
            1 => &self.dp.GPIOB.crh,
            2 => &self.dp.GPIOC.crh,
            3 => &self.dp.GPIOD.crh,
            4 => &self.dp.GPIOE.crh,
            5 => &self.dp.GPIOF.crh,
            6 => &self.dp.GPIOG.crh,
            _ => defmt::unreachable!(),
        }
    }

    fn crl(&self, port: usize) -> &Reg<CRL_SPEC> {
        match port {
            0 => &self.dp.GPIOA.crl,
            1 => &self.dp.GPIOB.crl,
            2 => &self.dp.GPIOC.crl,
            3 => &self.dp.GPIOD.crl,
            4 => &self.dp.GPIOE.crl,
            5 => &self.dp.GPIOF.crl,
            6 => &self.dp.GPIOG.crl,
            _ => defmt::unreachable!(),
        }
    }

    /// Reads the input values of all GPIOs.
    pub fn get_in(&self) -> [u16; PORTS] {
        let mut port_in = [0; PORTS];
        for (p, pi) in port_in.iter_mut().enumerate() {
            *pi = self.idr(p).read().bits() as u16;
        }
        port_in
    }

    /// Sets the output values of all GPIOs.
    pub fn set_out(&mut self, port_out: &[u16; PORTS]) {
        for (p, po) in port_out.iter().enumerate() {
            let usable = self.usable[p];
            let old = self.odr(p).read().bits() as u16;
            let new = (!usable & old) | (usable & po);
            self.odr(p).write(|w| unsafe { w.bits(new as u32) });
        }
    }

    /// Gets the output values of all GPIOs.
    pub fn get_out(&self) -> [u16; PORTS] {
        let mut port_out = [0; PORTS];
        for (p, po) in port_out.iter_mut().enumerate() {
            *po = self.odr(p).read().bits() as u16;
        }
        port_out
    }

    /// Gets the GPIO configuration.
    pub fn get_cfg(&self) -> [u64; PORTS] {
        let mut port_cfg = [0; PORTS];
        for (p, pc) in port_cfg.iter_mut().enumerate() {
            *pc = (self.crh(p).read().bits() as u64) << 32 | (self.crl(p).read().bits() as u64);
        }
        port_cfg
    }

    /// Sets the GPIO configuration.
    pub fn set_cfg(&mut self, port_cfg: &[u64; PORTS]) {
        for (p, pc) in port_cfg.iter().enumerate() {
            let usable = self.usable[p];

            let mut mask: u64 = 0;
            for b in 0..16 {
                if usable & (1 << b) != 0 {
                    mask |= 0b1111 << (b * 4);
                }
            }

            let old = (self.crh(p).read().bits() as u64) << 32 | (self.crl(p).read().bits() as u64);
            let new = (!mask & old) | (mask & pc);

            self.crh(p).write(|w| unsafe { w.bits((new >> 32) as u32) });
            self.crl(p).write(|w| unsafe { w.bits(new as u32) });
        }
    }

    /// Returns a bitmap of usable GPIO ports.
    pub fn usable(&self) -> &[u16; PORTS] {
        &self.usable
    }
}
