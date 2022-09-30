#!/bin/bash
#
# Build the OpenEMC driver modules.
#

cd -- "$( dirname -- "${BASH_SOURCE[0]}" )"
cd ../openemc-driver

if [ "$LINUX_DIR" == "" ] ; then
    make -C /lib/modules/`uname -r`/build M=$PWD "$@"
else
    make "$@"
fi

