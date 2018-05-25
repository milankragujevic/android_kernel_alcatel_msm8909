#[PLATFORM]-Add-BEGIN by mike.li for msg2238 driver code.[2015.03.06]
obj-$(CONFIG_TOUCHSCREEN_MSG2238_POP445C) += mstar_drv_common.o
obj-$(CONFIG_TOUCHSCREEN_MSG2238_POP445C) += mstar_drv_ic_fw_porting_layer.o
obj-$(CONFIG_TOUCHSCREEN_MSG2238_POP445C) += mstar_drv_main.o
obj-$(CONFIG_TOUCHSCREEN_MSG2238_POP445C) += mstar_drv_platform_interface.o
obj-$(CONFIG_TOUCHSCREEN_MSG2238_POP445C) += mstar_drv_platform_porting_layer.o
obj-$(CONFIG_TOUCHSCREEN_MSG2238_POP445C) += mstar_drv_qcom.o
obj-$(CONFIG_TOUCHSCREEN_MSG2238_POP445C) += mstar_drv_utility_adaption.o
obj-$(CONFIG_TOUCHSCREEN_MSG2238_POP445C) += mstar_drv_self_fw_control.o
obj-$(CONFIG_TOUCHSCREEN_MSG2238_POP445C) += mstar_drv_self_mp_test.o
obj-$(CONFIG_TOUCHSCREEN_MSG2238_POP445C) += mstar_drv_jni_interface.o

# add for forced upgrading of tp firmware in MINI SW.
ifeq ($(TARGET_BUILD_MMITEST), true)
KBUILD_CFLAGS += -DMINI_MODE_TO_CTP
endif
# add for forced upgrading of tp firmware in MINI SW.

#[PLATFORM]-Add-End by mike.li

