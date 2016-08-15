
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/usb_c/inc

ifeq ($(CONFIG_USB_C_SWITCH_FUSB301A),y)
obj-y += usb_typec.o
endif
ifeq ($(CONFIG_USB_C_SWITCH_TUSB320),y)
obj-y += usb_typec.o
endif

obj-$(CONFIG_USB_C_SWITCH_FUSB301A) += fusb301a/
# edit by wpb@meizu.com START
obj-$(CONFIG_USB_C_SWITCH_FUSB302) += fusb302_mz/
# edit by wpb@meizu.com END
obj-$(CONFIG_USB_C_SWITCH_SII70XX) += sii7033/
obj-$(CONFIG_USB_C_SWITCH_ANX7418) += anx7418/
obj-$(CONFIG_USB_C_SWITCH_TUSB320) += tusb320/

