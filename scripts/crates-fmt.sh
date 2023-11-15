#!/bin/bash
#
# Formats all OpenEMC crates.
#

set -e

cd -- "$( dirname -- "${BASH_SOURCE[0]}" )"
cd ..

cargo +nightly fmt --manifest-path openemc-bootloader/Cargo.toml
cargo +nightly fmt --manifest-path openemc-firmware/Cargo.toml
cargo +nightly fmt --manifest-path openemc-shared/Cargo.toml
cargo +nightly fmt --manifest-path xtask/Cargo.toml

rustup run nightly rustfmt openemc-bootloader/src/boards/*.rs
rustup run nightly rustfmt openemc-firmware/src/boards/*.rs
