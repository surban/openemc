//! Board.

use core::{
    any::TypeId,
    cell::RefCell,
    mem,
    mem::{take, transmute},
    sync::atomic::{AtomicBool, Ordering},
};
use cortex_m::interrupt::{self, Mutex};
use defmt::Format;
use heapless::{Deque, Vec};
use stm32f1xx_hal::{afio, i2c, rcc::Clocks};

use crate::{
    boot,
    bq25713::{Bq25713Cfg, InputCurrentLimit},
    cfg::Cfg,
    i2c_reg_slave,
    supply::{max14636::Max14636, FixedSinkPdo, PowerSupply},
    Delay, Duration, PowerMode, I2C_BUFFER_SIZE,
};
use openemc_shared::boot::BootInfo;

/// Number of IO ports (PA, PB, PC, etc.).
pub const PORTS: usize = 4;

/// Board-specific functionality.
pub trait Board {
    /// I2C slave address.
    const I2C_ADDR: u8 = 0x10;

    /// I2C pins remapped?
    const I2C_REMAP: bool = true;

    /// IRQ pin number (PA0, PA1, .., PA15, PB0, PB1, ...)
    const IRQ_PIN: u8 = 6; // PA6

    /// IRQ pin configuration.
    const IRQ_PIN_CFG: u8 = 0b0010; // 2 MHz push-pull output

    /// Simulates interrupts on PD0 and PD1 with the specified polling interval.
    ///
    /// This is necessary because the STM32 does not support them as interrupt
    /// triggers.
    const SIMULATE_PD_IRQS: Option<Duration> = None;

    /// Whether to use a timer for PWM.
    const PWM_TIMERS: [bool; 4] = [false; 4];

    /// Mode for I2C 2 master bus.
    const I2C2_MODE: Option<i2c::Mode> = None;

    /// I2C address on I2C 2 master bus of STUSB4500 USB PD controller, if present.
    const STUSB4500_I2C_ADDR: Option<u8> = None;

    /// NVM contents to program into STUSB4500 USB PD controller.
    #[rustfmt::skip]
    const STUSB4500_NVM: Option<[u8; 40]> = Some([
        0x00, 0x00, 0xB0, 0xAB, 0x00, 0x45, 0x00, 0x00,
        0x20, 0x40, 0x9C, 0x1C, 0xFF, 0x01, 0x3C, 0xDF,
        0x02, 0x40, 0x0F, 0x00, 0x32, 0x00, 0xFC, 0xF1,
        0x00, 0x19, 0x11, 0xAF, 0xF5, 0x35, 0x5F, 0x00,
        0x00, 0x4B, 0x90, 0x21, 0x43, 0x00, 0x40, 0xFB,
    ]);

    /// BQ25713 battery charge controller configuration, if present.
    const BQ25713_CFG: Option<Bq25713Cfg> = None;

    /// Initial USB PD sink PDO.
    ///
    /// Must have voltage of 5 V.
    const USB_INITIAL_PDO: FixedSinkPdo = FixedSinkPdo {
        operating_current_ma: 100,
        voltage_mv: 5000,
        fast_role_req_current_ma: 0,
        dual_role_data: false,
        dual_role_power: false,
        communication: true,
        unconstrained_power: false,
        higher_capability: false,
    };

    /// Maximum USB input voltage in mV to request via USB PD.
    const USB_MAXIMUM_VOLTAGE: u32 = 5000;

    /// Battery voltage in mV that triggers immediate low voltage shutdown.
    const CRITICAL_LOW_BATTERY_VOLTAGE: Option<u32> = None;

    /// Minimum battery voltage in mV that is required for system power on.
    const MIN_POWER_ON_BATTERY_VOLTAGE: Option<u32> = None;

    /// Time to charge battery before attempting to power on, if it is below
    /// [`MIN_POWER_ON_BATTERY_VOLTAGE`](Self::MIN_POWER_ON_BATTERY_VOLTAGE).
    const LOW_BATTERY_CHARGING_TIME: Duration = Duration::minutes(5);

    /// Battery voltage in mV that is used for switching charging LED from blinking to steady.
    const CHARGING_LED_END_VOLTAGE: u32 = 10000;

    /// Battery voltage in mV that is used for switching charging LED from blinking to steady,
    /// when charging current is below [`CHARGING_LED_MIN_CURRENT`](Self::CHARGING_LED_MIN_CURRENT).
    const CHARGING_LED_NOT_CHARGING_ACCEPTABLE_VOLTAGE: u32 = Self::CHARGING_LED_END_VOLTAGE;

    /// Battery current in mA that is used to indicate charging error if fallen below.
    const CHARGING_LED_MIN_CURRENT: i32 = 0;

    /// Task arguments.
    type TaskArgs: 'static;

    /// Task queue length.
    const TASK_QUEUE_LENGTH: usize = 16;

    /// Create a new instance.
    fn new(data: InitData, res: InitResources) -> Self;

    /// Returns whether the board id is supported.
    fn is_supported(&self, model: &[u8]) -> bool {
        model.starts_with(Self::model())
    }

    /// Board model.
    fn model() -> &'static [u8];

    /// The mode for powering up.
    fn power_mode(&mut self) -> PowerMode {
        PowerMode::Full { quiet: false }
    }

    /// Sets the mode for powering up.
    fn set_power_mode(&mut self, _power_mode: PowerMode) {}

    /// Powers on the system.
    fn power_on(&mut self, _afio: &mut afio::Parts, _delay: &mut Delay) {}

    /// Shutdown the system and go to sleep.
    fn shutdown(&mut self, _res: InitResources) -> ! {
        boot::enter_standby();
    }

    /// Limit usable GPIOs.
    fn limit_usable(&mut self, _usable: &mut [u16; PORTS]) {}

    /// Limit usable EXTI lines.
    fn limit_usable_exti(&mut self, _exti: &mut u32) {}

    /// Read from board-specific I2C register.
    fn i2c_read(&mut self, _reg: u8) -> Result<i2c_reg_slave::Response<I2C_BUFFER_SIZE>, UnknownI2cRegister> {
        Err(UnknownI2cRegister)
    }

    /// Write to board-specific I2C register.
    fn i2c_write(
        &mut self, _reg: u8, _value: i2c_reg_slave::Value<I2C_BUFFER_SIZE>,
    ) -> Result<(), UnknownI2cRegister> {
        Err(UnknownI2cRegister)
    }

    /// Checks whether power on during charging has been requested.
    fn check_power_on_requested(&mut self) -> bool {
        false
    }

    /// Checks whether STUSB4500 is alerting and clears the pending interrupt.
    fn check_stusb4500_alerting(&mut self) -> bool {
        false
    }

    /// Checks whether STUSB4500 is attached.
    fn check_stusb4500_attached(&mut self) -> bool {
        false
    }

    /// Reset STUSB4500 via reset pin.
    fn set_stusb4500_reset_pin(&mut self, _state: bool) {}

    /// MAX14636 USB charger detector.
    fn max14636(&mut self) -> Option<Max14636> {
        None
    }

    /// Gets the status of the BQ25713 CHRG_OK signal.
    fn check_bq25713_chrg_ok(&mut self) -> bool {
        false
    }

    /// Checks whether the state of the BQ25713 CHRG_OK signal changed and clears the pending interrupt.
    ///
    /// Returns `Some(state)` if the state changed and `None` if the state is unchanged.
    fn check_bq25713_chrg_ok_changed(&mut self) -> Option<bool> {
        None
    }

    /// Calculate the input current limit for the connected power supply.
    ///
    /// `active` specifies for how long the supplied power supply configuration has been unchanged.
    fn input_current_limit(&mut self, power_supply: &PowerSupply, active: Duration) -> InputCurrentLimit {
        let stable = active.to_secs() >= 3;

        let mut max_input_current_ma = power_supply.max_current_ma();
        if max_input_current_ma > 200 {
            max_input_current_ma -= 100;
        }
        if !stable {
            max_input_current_ma = max_input_current_ma.min(500);
        }

        InputCurrentLimit {
            max_input_current_ma,
            ico: matches!(power_supply, PowerSupply::UsbDcp | PowerSupply::UsbCdp) && stable,
        }
    }

    /// Sets the power LED to the specified state.
    fn set_power_led(&mut self, _state: bool) {}

    /// Sets the charging LED to the specified state.
    fn set_charging_led(&mut self, _state: bool) {}

    /// Custom board-specific function that is called periodically.
    ///
    /// Returns the delay until the function should be called the next time.
    fn periodic(&mut self) -> Duration {
        Duration::secs(60)
    }

    /// Board IO read.
    fn io_read(&mut self) -> Vec<u8, IO_READ_SIZE> {
        Vec::new()
    }

    /// Board IO write.
    fn io_write(&mut self, _data: Vec<u8, IO_WRITE_SIZE>) -> Result<(), WriteBlock> {
        Ok(())
    }

    /// Board ioctl request.
    fn ioctl(&mut self, _ioctl: Ioctl) {}

    /// Custom board task, that can be spawned using [`spawn_task`](Self::spawn_task).
    fn task(&mut self, _args: Self::TaskArgs) {}

    /// Queues the board task with the specified arguments for execution.
    fn spawn_task(args: Self::TaskArgs) -> Result<(), Self::TaskArgs> {
        interrupt::free(|cs| {
            let mut task_queue = TASK_QUEUE.borrow(cs).borrow_mut();

            // This is a no-op, since Self::TaskArgs and crate::ThisBoard as Board>::TaskArgs are the same type
            // for the chosen board. However, this is necessary to avoid compile errors for all non-chosen
            // boards.
            defmt::assert!(
                TypeId::of::<Self::TaskArgs>() == TypeId::of::<<crate::ThisBoard as Board>::TaskArgs>()
            );
            let task_queue: &mut Deque<Self::TaskArgs, { <crate::ThisBoard as Board>::TASK_QUEUE_LENGTH }> =
                unsafe { transmute(&mut *task_queue) };

            task_queue.push_back(args)
        })
    }
}

/// Board initialization data.
#[derive(Clone)]
#[non_exhaustive]
pub struct InitData {
    /// Boot information.
    pub boot_info: &'static BootInfo,
    /// Clocks.
    pub clocks: Clocks,
    /// Configuration.
    pub cfg: Cfg,
}

/// Board initialization resources.
///
/// To be used only during initialization.
#[non_exhaustive]
pub struct InitResources<'a> {
    /// Alternate function IO registers.
    pub afio: &'a mut afio::Parts,
    /// Delay provider.
    pub delay: &'a mut Delay,
}

/// Board task queue.
pub(crate) static TASK_QUEUE: Mutex<
    RefCell<Deque<<crate::ThisBoard as Board>::TaskArgs, { <crate::ThisBoard as Board>::TASK_QUEUE_LENGTH }>>,
> = Mutex::new(RefCell::new(Deque::new()));

/// Unknown I2C event.
#[derive(Format)]
pub struct UnknownI2cRegister;

/// IO write currently not accepted.
///
/// It will be tried again after [`io_write_available`] has been called.
#[derive(Format)]
pub struct WriteBlock;

/// A board ioctl request.
pub struct Ioctl {
    req: Vec<u8, { Self::BUFFER_SIZE }>,
    responded: bool,
}

impl defmt::Format for Ioctl {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "Ioctl ([");
        for v in &self.req {
            defmt::write!(fmt, "{:02x} ", *v);
        }
        defmt::write!(fmt, "])");
    }
}

/// Board ioctl is already active.
#[derive(Format)]
pub(crate) struct IoctlAlreadyActive;

/// Board ioctl status.
#[derive(Default)]
pub(crate) enum IoctlStatus {
    /// No ioctl active.
    #[default]
    None,
    /// Ioctl is being processed by board.
    Processing,
    /// Ioctl response is available.
    Done {
        /// The response data.
        response: Vec<u8, { Ioctl::BUFFER_SIZE }>,
        /// Whether the host was notified.
        notified: bool,
    },
    /// Ioctl failed.
    Failed {
        /// Error number.
        errno: u8,
        /// Whether the host was notified.
        notified: bool,
    },
}

/// Board ioctl status.
pub(crate) static IOCTL_STATUS: Mutex<RefCell<IoctlStatus>> = Mutex::new(RefCell::new(IoctlStatus::None));

impl Ioctl {
    /// Request and response buffer size.
    pub const BUFFER_SIZE: usize = I2C_BUFFER_SIZE;

    /// Start a board ioctl request.
    pub(crate) fn start(req: Vec<u8, { Self::BUFFER_SIZE }>) -> Result<Self, IoctlAlreadyActive> {
        interrupt::free(|cs| {
            let mut status = IOCTL_STATUS.borrow(cs).borrow_mut();
            if let IoctlStatus::Processing { .. } = *status {
                defmt::error!("attempt to make board ioctl request while ioctl is being processed");
                return Err(IoctlAlreadyActive);
            }
            *status = IoctlStatus::Processing;
            Ok(Self { req, responded: false })
        })
    }

    /// Return the response if done.
    pub(crate) fn take_response() -> Option<Vec<u8, { Self::BUFFER_SIZE }>> {
        interrupt::free(|cs| {
            let mut status = IOCTL_STATUS.borrow(cs).borrow_mut();
            match *status {
                IoctlStatus::Done { .. } => (),
                _ => return None,
            }
            match take(&mut *status) {
                IoctlStatus::Done { response, .. } => Some(response),
                _ => defmt::unreachable!(),
            }
        })
    }

    /// Returns whether an IRQ is requested.
    pub(crate) fn irq_required() -> bool {
        interrupt::free(|cs| {
            let mut status = IOCTL_STATUS.borrow(cs).borrow_mut();
            match &mut *status {
                IoctlStatus::Done { notified, .. } | IoctlStatus::Failed { notified, .. } if !*notified => {
                    *notified = true;
                    true
                }
                _ => false,
            }
        })
    }

    /// The request data.
    #[allow(dead_code)]
    pub fn request(&self) -> &Vec<u8, { Self::BUFFER_SIZE }> {
        &self.req
    }

    /// Take the request data, leaving empty request data behind.
    pub fn take_request(&mut self) -> Vec<u8, { Self::BUFFER_SIZE }> {
        mem::take(&mut self.req)
    }

    /// Respond successfully to the request.
    pub fn success(mut self, response: Vec<u8, { Self::BUFFER_SIZE }>) {
        interrupt::free(|cs| {
            let mut status = IOCTL_STATUS.borrow(cs).borrow_mut();
            *status = IoctlStatus::Done { response, notified: false };
        });
        self.responded = true;
    }

    /// Fail with specified error number.
    pub fn fail(mut self, errno: u8) {
        interrupt::free(|cs| {
            let mut status = IOCTL_STATUS.borrow(cs).borrow_mut();
            *status = IoctlStatus::Failed { errno, notified: false };
        });
        self.responded = true;
    }
}

impl Drop for Ioctl {
    fn drop(&mut self) {
        if !self.responded {
            defmt::warn!("Ioctl was dropped without response");
            interrupt::free(|cs| {
                let status = IOCTL_STATUS.borrow(cs);
                *status.borrow_mut() = IoctlStatus::Failed { errno: 0, notified: false };
            });
        }
    }
}

/// Whether board io data is available for reading.
pub(crate) static IO_READ_AVAILABLE: AtomicBool = AtomicBool::new(false);

/// Whether board io data is available for writing.
pub(crate) static IO_WRITE_AVAILABLE: AtomicBool = AtomicBool::new(false);

/// Board maximum IO read size.
pub const IO_READ_SIZE: usize = I2C_BUFFER_SIZE - 1;

/// Board maximum IO write size.
pub const IO_WRITE_SIZE: usize = I2C_BUFFER_SIZE;

/// Notifies system that board io data is available for reading read.
pub fn io_read_available() {
    IO_READ_AVAILABLE.store(true, Ordering::SeqCst);
}

/// Notifies system that board io data can be written.
pub fn io_write_available() {
    IO_WRITE_AVAILABLE.store(true, Ordering::SeqCst);
}
