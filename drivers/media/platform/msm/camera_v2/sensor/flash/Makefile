ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/cci
ccflags-y += -Idrivers/media/platform/msm/camera_v2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/io
obj-$(CONFIG_MSMB_CAMERA) += msm_led_flash.o
obj-$(CONFIG_MSMB_CAMERA) += msm_led_trigger.o
obj-$(CONFIG_MSMB_CAMERA) += msm_led_i2c_trigger.o
obj-$(CONFIG_MSMB_CAMERA) += adp1660.o
obj-$(CONFIG_MSMB_CAMERA) += bd7710.o
obj-$(CONFIG_MSMB_CAMERA) += msm_led_torch.o
obj-$(CONFIG_MSMB_CAMERA) += msm_flash.o
obj-$(CONFIG_MSMB_CAMERA) += lm3642.o
#add by weicai.long@tcl.com, flashlight driver.
obj-$(CONFIG_MSMB_CAMERA) += tct_rear_front_led_trigger.o
obj-$(CONFIG_MSMB_CAMERA) += tct_rear_front_led_torch.o
#Begin add by zhaohong.chen@tcl.com for POP45 flashlight driver,20151218
ifeq ($(TARGET_PRODUCT),POP45)
obj-$(CONFIG_MSMB_CAMERA) += sy7806.o
endif
#End add
#Begin add by zhaohong.chen@tcl.com for GOPLAY2 flashlight driver,20151218
ifeq ($(TARGET_PRODUCT),GOPLAY2)
obj-$(CONFIG_MSMB_CAMERA) += sgm3784.o
endif
#End add
ifeq ($(TARGET_PRODUCT),Pixi464G)
#/*[BUGFIX]-Mod-BEGIN by TCTSZ.(gaoxiang.zou@tcl.com), PR 12/21/2015*/
obj-$(CONFIG_MSMB_CAMERA) += ktd2682.o
endif
ifeq ($(TARGET_PRODUCT),Pixi464GCricket)
#/*[BUGFIX]-Mod-BEGIN by TCTSZ.(gaoxiang.zou@tcl.com), PR 12/21/2015*/
obj-$(CONFIG_MSMB_CAMERA) += ktd2682.o
endif
