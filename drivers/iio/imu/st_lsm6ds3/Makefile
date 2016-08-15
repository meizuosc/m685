#
# Makefile for STMicroelectronics LSM6DS3 sensor.
#

#include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-$(CONFIG_ST_LSM6DS3_IIO) += st_lsm6ds3.o
st_lsm6ds3-objs := st_lsm6ds3_core.o st_lsm6ds3_self_test.o

obj-$(CONFIG_ST_LSM6DS3_IIO) += st_lsm6ds3_i2c.o
st_lsm6ds3-$(CONFIG_IIO_BUFFER) += st_lsm6ds3_buffer.o
st_lsm6ds3-$(CONFIG_IIO_TRIGGER) += st_lsm6ds3_trigger.o
st_lsm6ds3-$(CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT) += st_lsm6ds3_i2c_master.o
