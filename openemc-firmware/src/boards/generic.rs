//! Generic board.

use openemc_shared::BootInfo;

use crate::board::Board;

/// Generic board.
pub struct BoardImpl;

impl Board for BoardImpl {
    fn new(_board_data: Option<&[u8; BootInfo::BOARD_DATA_SIZE]>) -> BoardImpl {
        Self
    }

    fn model() -> &'static [u8] {
        b"generic"
    }    
}
