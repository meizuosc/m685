#TFA98XX_ROOT_DIR := $(CURDIR) 
TFA98XX_ROOT_DIR := $(srctree)/sound/soc/tfa98xx

subdir-ccflags-y += -I$(TFA98XX_ROOT_DIR)/tfa/inc
subdir-ccflags-y += -I$(TFA98XX_ROOT_DIR)/tfa/linux-kernel-inc
subdir-ccflags-y += -I$(TFA98XX_ROOT_DIR)/tfa/src

obj-$(CONFIG_SND_SOC_TFA98XX) += tfa/
