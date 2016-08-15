#include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-$(CONFIG_MTK_LEDS) := leds.o

obj-$(CONFIG_BACKLIGHT_SUPPORT_LM3697) += lm3697_driver.o
obj-$(CONFIG_BACKLIGHT_SUPPORT_LM36272) += lm36272_driver.o

ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include
