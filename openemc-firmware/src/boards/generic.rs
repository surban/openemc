//! Generic board.

use openemc_shared::BootInfo;
use stm32f1xx_hal::afio;

use crate::{board::Board, Delay};

/// Generic board.
pub struct BoardImpl;

impl Board for BoardImpl {
    fn new(
        _board_data: Option<&[u8; BootInfo::BOARD_DATA_SIZE]>, _afio: &mut afio::Parts, _delay: &mut Delay,
    ) -> BoardImpl {
        Self
    }

    fn model() -> &'static [u8] {
        b"generic"
    }
}
