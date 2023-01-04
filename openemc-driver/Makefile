obj-m += openemc.o openemc_adc.o openemc_battery.o openemc_gpio.o \
	openemc_pinctrl.o openemc_power.o openemc_pwm.o openemc_rtc.o \
	openemc_supply.o openemc_wdt.o

all:
	make -C ${LINUX_DIR} M=$(PWD) modules

clean:
	make -C ${LINUX_DIR} M=$(PWD) clean
