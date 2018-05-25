ccflags-y += -Idrivers/media/platform/msm/camera_v2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/msm_vb2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/camera
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/io
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/cci
obj-$(CONFIG_MSMB_CAMERA) += cci/ io/ csiphy/ csid/ actuator/ flash/ eeprom/ ois/
obj-$(CONFIG_MSM_CAMERA_SENSOR) += msm_sensor_init.o msm_sensor_driver.o msm_sensor.o
obj-$(CONFIG_MT9M114) += mt9m114.o
obj-$(CONFIG_OV5645) += ov5645.o
obj-$(CONFIG_OV5645) += ov7695.o
#Begin mod by (TCTSZ) jin.xia@tcl.com for engineer mode, 2015-11-24

obj-y += camera_tct_func.o
#End mod by 

obj-$(CONFIG_GC0310) += gc0310.o
obj-$(CONFIG_GC0310_YUV_PIXI455TF) += gc0310_yuv_pixi455tf.o
obj-$(CONFIG_GC0310_YUV_SHINETCH) += gc0310_yuv_shinetch.o

#add by weicai.long@tcl.com
obj-$(CONFIG_OV8858_OTP) += ov8858_otp.o
obj-$(CONFIG_OV5670_OTP) += ov5670_otp.o
obj-$(CONFIG_S5K4H8_OTP) += s5k4h8_otp.o
obj-$(CONFIG_S5K5E8_OTP) += s5k5e8_otp.o
obj-$(CONFIG_OV8865_OTP) += ov8865_otp.o
#add by jin.xia@tcl.com
obj-$(CONFIG_S5K4H8_TRULY_OTP) += s5k4h8_otp_truly.o
obj-$(CONFIG_OV8856_SUNNY_OTP) += ov8856_otp.o
obj-$(CONFIG_HI553_KINGCOME_OTP) += hi553_kingcome_otp.o
#end add
obj-$(CONFIG_GC0310_SUNRISE) += gc0310_sunrise.o
obj-$(CONFIG_SP0A20_SUNRISE) += sp0a20_sunrise.o
