//! Analog to digital converter.

use stm32f1::stm32f103::{Peripherals, ADC1};
use stm32f1xx_hal::{
    adc::Adc,
    gpio::{Analog, Pin, CRL},
    prelude::*,
};

/// ADC inputs.
pub struct AdcInputs {
    ch0: Pin<Analog, CRL, 'A', 0>,
    ch1: Pin<Analog, CRL, 'A', 1>,
    ch2: Pin<Analog, CRL, 'A', 2>,
    ch3: Pin<Analog, CRL, 'A', 3>,
    ch4: Pin<Analog, CRL, 'A', 4>,
    ch5: Pin<Analog, CRL, 'A', 5>,
    ch6: Pin<Analog, CRL, 'A', 6>,
    ch7: Pin<Analog, CRL, 'A', 7>,
    ch8: Pin<Analog, CRL, 'B', 0>,
    ch9: Pin<Analog, CRL, 'B', 1>,
    ch10: Pin<Analog, CRL, 'C', 0>,
    ch11: Pin<Analog, CRL, 'C', 1>,
    ch12: Pin<Analog, CRL, 'C', 2>,
    ch13: Pin<Analog, CRL, 'C', 3>,
    ch14: Pin<Analog, CRL, 'C', 4>,
    ch15: Pin<Analog, CRL, 'C', 5>,
}

impl Default for AdcInputs {
    fn default() -> Self {
        Self::new()
    }
}

impl AdcInputs {
    /// Number of ADC channels.
    pub const CHANNELS: usize = 16;

    /// Initializes ADC inputs.
    ///
    /// GPIO settings might be disturbed briefly.
    pub fn new() -> Self {
        let dp = unsafe { Peripherals::steal() };

        // Save GPIO config.
        let crl_a = dp.GPIOA.crl.read().bits();
        let crl_b = dp.GPIOB.crl.read().bits();
        let crl_c = dp.GPIOC.crl.read().bits();

        // Acquire analog pins.
        let mut gpioa = dp.GPIOA.split();
        let mut gpiob = dp.GPIOB.split();
        let mut gpioc = dp.GPIOC.split();
        let this = Self {
            ch0: gpioa.pa0.into_analog(&mut gpioa.crl),
            ch1: gpioa.pa1.into_analog(&mut gpioa.crl),
            ch2: gpioa.pa2.into_analog(&mut gpioa.crl),
            ch3: gpioa.pa3.into_analog(&mut gpioa.crl),
            ch4: gpioa.pa4.into_analog(&mut gpioa.crl),
            ch5: gpioa.pa5.into_analog(&mut gpioa.crl),
            ch6: gpioa.pa6.into_analog(&mut gpioa.crl),
            ch7: gpioa.pa7.into_analog(&mut gpioa.crl),
            ch8: gpiob.pb0.into_analog(&mut gpiob.crl),
            ch9: gpiob.pb1.into_analog(&mut gpiob.crl),
            ch10: gpioc.pc0.into_analog(&mut gpioc.crl),
            ch11: gpioc.pc1.into_analog(&mut gpioc.crl),
            ch12: gpioc.pc2.into_analog(&mut gpioc.crl),
            ch13: gpioc.pc3.into_analog(&mut gpioc.crl),
            ch14: gpioc.pc4.into_analog(&mut gpioc.crl),
            ch15: gpioc.pc5.into_analog(&mut gpioc.crl),
        };

        // Restore GPIO config.
        unsafe {
            let dp = Peripherals::steal();
            dp.GPIOA.crl.write(|w| w.bits(crl_a));
            dp.GPIOB.crl.write(|w| w.bits(crl_b));
            dp.GPIOC.crl.write(|w| w.bits(crl_c));
        }

        this
    }

    /// Reads the specified ADC channel.
    pub fn read(&mut self, adc: &mut Adc<ADC1>, ch: usize) -> u16 {
        let res = match ch {
            0 => adc.read(&mut self.ch0),
            1 => adc.read(&mut self.ch1),
            2 => adc.read(&mut self.ch2),
            3 => adc.read(&mut self.ch3),
            4 => adc.read(&mut self.ch4),
            5 => adc.read(&mut self.ch5),
            6 => adc.read(&mut self.ch6),
            7 => adc.read(&mut self.ch7),
            8 => adc.read(&mut self.ch8),
            9 => adc.read(&mut self.ch9),
            10 => adc.read(&mut self.ch10),
            11 => adc.read(&mut self.ch11),
            12 => adc.read(&mut self.ch12),
            13 => adc.read(&mut self.ch13),
            14 => adc.read(&mut self.ch14),
            15 => adc.read(&mut self.ch15),
            _ => defmt::panic!("invalid ADC channel"),
        };

        match res {
            Ok(value) => value,
            Err(_) => defmt::panic!("ADC conversion failed"),
        }
    }
}

/// ADC conversion buffer.
#[derive(Clone, Default)]
pub struct AdcBuf {
    /// Reference voltage.
    pub vref: u16,
    /// Temperature.
    pub temp: i32,
    /// Channel measurements.
    pub channel: [u16; AdcInputs::CHANNELS],
}

impl AdcBuf {
    /// Voltage of specified channel.
    pub const fn voltage(&self, channel: usize) -> u16 {
        ((self.channel[channel] as u32) * 1200 / (self.vref as u32)) as u16
    }

    /// Voltages of all channels.
    pub fn voltages(&self) -> [u16; AdcInputs::CHANNELS] {
        let mut v = [0; AdcInputs::CHANNELS];
        for (ch, vch) in v.iter_mut().enumerate() {
            *vch = self.voltage(ch);
        }
        v
    }

    /// Acquires the values.
    pub fn acquire(adc: &mut Adc<ADC1>, inputs: &mut AdcInputs) -> Self {
        let mut this = Self { vref: adc.read_vref(), temp: adc.read_temp(), ..Default::default() };

        for ch in 0..this.channel.len() {
            this.channel[ch] = inputs.read(adc, ch);
        }

        this
    }
}
