//! Generic board.

use crate::board::Board;

/// Generic board.
pub struct BoardImpl;

impl Board for BoardImpl {
    fn new() -> Self {
        Self
    }

    fn model(&self) -> &'static [u8] {
        b"generic"
    }
}
