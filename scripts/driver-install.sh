#!/bin/bash
#
# Build and install the OpenEMC driver for the active kernel.
#

set -e

cd -- "$( dirname -- "${BASH_SOURCE[0]}" )"
cd ../openemc-driver

../scripts/driver-build.sh clean
../scripts/driver-build.sh -j

sudo ../scripts/driver-unload.sh || true
sudo ../scripts/driver-build.sh modules_install
sudo depmod -a
sudo cp -fv ../image/*.emc /lib/firmware/
sudo ../scripts/driver-load.sh
