
# host notify driver
obj-$(CONFIG_USB_HOST_NOTIFY)		+= host_notify_class.o host_notifier.o

obj-$(CONFIG_USB_NOTIFY_LAYER)		+= usb_notify_layer.o
usb_notify_layer-y			:= usb_notify.o  usb_notify_sysfs.o
obj-$(CONFIG_USB)		+= dock_notify.o

obj-$(CONFIG_USB_NOTIFIER_QCOM)		+= usb_notifier_qcom.o
obj-$(CONFIG_USB_NOTIFIER_SLSI)		+= usb_notifier.o

obj-$(CONFIG_USB_HOST_NOTIFY_FEATURE)	+= host_notify_class.o
