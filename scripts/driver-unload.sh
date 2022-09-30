#!/bin/sh
#
# Unloads all OpenEMC driver modules.
#

set -e

modprobe -r openemc_wdt || true
modprobe -r openemc_power || true
modprobe -r openemc_adc || true
modprobe -r openemc_gpio || true
modprobe -r openemc_pinctrl || true
modprobe -r openemc_rtc || true
modprobe -r openemc
