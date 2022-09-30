#!/bin/sh
#
# Loads the specified device tree overlay.
#

set -e

DIR="/sys/kernel/config/device-tree/overlays/openemc"

if [ -d $DIR ] ; then
    rmdir $DIR
fi

if ! [ -z "$1" ] ; then
    mkdir -p $DIR
    cat "$1" > $DIR/dtbo
fi

