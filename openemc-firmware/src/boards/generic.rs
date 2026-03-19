//! Generic board.
//
// OPENEMC-BOARD-VERSION: 1
// OPENEMC-FLASH-SIZE: 131072
// OPENEMC-RAM-SIZE: 20480
//

#![cfg_attr(not(board = "generic"), allow(dead_code))]

use crate::board::{Board, InitData, InitResources};

/// Generic board.
pub struct BoardImpl;

impl Board for BoardImpl {
    type TaskArgs = ();

    fn new(_data: InitData, _res: InitResources) -> BoardImpl {
        Self
    }

    fn model() -> &'static [u8] {
        b"generic"
    }
}
