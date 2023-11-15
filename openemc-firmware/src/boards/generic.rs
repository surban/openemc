//! Generic board.
//
// OPENEMC-BOARD-VERSION: 1
// OPENEMC-FLASH-SIZE: 65536
// OPENEMC-RAM-SIZE: 20480
//

use stm32f1xx_hal::afio;

use crate::{board::Board, cfg::Cfg, Delay};
use openemc_shared::boot::BootInfo;

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
