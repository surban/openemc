#!/bin/bash
#
# Updates the dependencies of all crates.
#

set -e

cd -- "$( dirname -- "${BASH_SOURCE[0]}" )"
cd ..

cargo +nightly update --manifest-path openemc-bootloader/Cargo.toml
cargo +nightly update --manifest-path openemc-firmware/Cargo.toml
cargo +nightly update --manifest-path openemc-shared/Cargo.toml
cargo +nightly update --manifest-path xtask/Cargo.toml
