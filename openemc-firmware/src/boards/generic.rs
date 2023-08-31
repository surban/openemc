//! Generic board.

use openemc_shared::BootInfo;
use stm32f1xx_hal::afio;

use crate::cfg::Cfg;
use crate::{board::Board, Delay};

/// Generic board.
pub struct BoardImpl;

impl Board for BoardImpl {
    fn new(_boot_info: &'static BootInfo, _afio: &mut afio::Parts, _delay: &mut Delay, _cfg: &Cfg) -> BoardImpl {
        Self
    }

    fn model() -> &'static [u8] {
        b"generic"
    }
}
