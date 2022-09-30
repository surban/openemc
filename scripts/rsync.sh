#!/bin/bash
#
# Rsync to a test host.
#

set -e

cd -- "$( dirname -- "${BASH_SOURCE[0]}" )"
cd ..

rsync -az --delete --exclude '*.o' --exclude '*.ko' --exclude '*.mod*' \
    --exclude 'target' --exclude '*.cmd' --exclude 'Module.symvers' --exclude 'modules.order' \
    . $(cat .test-host)
