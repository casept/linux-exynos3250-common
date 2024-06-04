#
# Makefile for the sensors drivers.
#

# Each configuration option enables a list of files.

ifndef CONFIG_SENSORS_SSP
obj-$(CONFIG_SENSORS)		+= sensors_core.o
endif

obj-$(CONFIG_SENSORS_TMD27723)	+= tmd27723.o
obj-$(CONFIG_SENSORS_AK09911C)	+= ak09911c.o
# 6-axis_sensor
obj-$(CONFIG_INPUT_MPU6500)     += mpu6500/
obj-$(CONFIG_SENSORS_K6DS3TR)	+= k6ds3.o
# barometer
obj-$(CONFIG_SENSORS)     += lps22hb/
