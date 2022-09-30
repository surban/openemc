#!/bin/bash
#
# Builds the OpenEMC devicetree examples.
#

set -e

cd -- "$( dirname -- "${BASH_SOURCE[0]}" )"
cd ../devicetree

for SRC in *.dts ; do
    cpp -nostdinc -I include -I arch -undef -x assembler-with-cpp \
        -I /lib/modules/`uname -r`/build/include \
        "$SRC" "$SRC.preprocessed"

    dtc -o "$(basename $SRC .dts).dtbo" "$SRC.preprocessed"
    rm -f "$SRC.preprocessed"
done
