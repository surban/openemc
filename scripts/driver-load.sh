#!/bin/sh
#
# Loads all OpenEMC driver modules.
#

set -e

modprobe openemc
modprobe openemc_pinctrl
modprobe openemc_gpio
modprobe openemc_power
modprobe openemc_rtc
modprobe openemc_wdt
modprobe openemc_adc
