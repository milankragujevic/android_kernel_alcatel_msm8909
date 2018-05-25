#
# Makefile for the input core drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_INPUT)		+= input-core.o
input-core-y := input.o input-compat.o input-mt.o ff-core.o

obj-$(CONFIG_INPUT_FF_MEMLESS)	+= ff-memless.o
obj-$(CONFIG_INPUT_POLLDEV)	+= input-polldev.o
obj-$(CONFIG_INPUT_SPARSEKMAP)	+= sparse-keymap.o
obj-$(CONFIG_INPUT_MATRIXKMAP)	+= matrix-keymap.o

obj-$(CONFIG_INPUT_MOUSEDEV)	+= mousedev.o
obj-$(CONFIG_INPUT_JOYDEV)	+= joydev.o
obj-$(CONFIG_INPUT_EVDEV)	+= evdev.o
obj-$(CONFIG_INPUT_EVBUG)	+= evbug.o

obj-$(CONFIG_INPUT_KEYBOARD)	+= keyboard/
obj-$(CONFIG_INPUT_MOUSE)	+= mouse/
obj-$(CONFIG_INPUT_JOYSTICK)	+= joystick/
obj-$(CONFIG_INPUT_TABLET)	+= tablet/
obj-$(CONFIG_INPUT_TOUCHSCREEN)	+= touchscreen/
obj-$(CONFIG_INPUT_MISC)	+= misc/

obj-$(CONFIG_INPUT_APMPOWER)	+= apm-power.o
obj-$(CONFIG_INPUT_KEYRESET)	+= keyreset.o
obj-$(CONFIG_INPUT_KEYCOMBO)	+= keycombo.o
obj-$(CONFIG_SENSORS_HALL)	+= hall_sensor.o

#[Feature]-BEGIN by TCTSZ.yongzhong.cheng@tcl.com,2015/8/26,for 538697:add gryoscope bmg160 driver
obj-$(CONFIG_GRYOSCOPE_BMG160)	+= bmg160/
#[Feature]-END by TCTSZ.yongzhong.cheng@tcl.com, 2015/8/26,for  538697

#yongzhong.cheng@tcl.com,task1401030,2016/1/12,enable pressure sensor
obj-$(CONFIG_PRESSURE_BMP280)  += bmp280/
#yongzhong.cheng@tcl.com,task1401030,2016/1/12,enable pressure sensor

#yongzhong.cheng@tcl.com,2016/4/12,defect 1938927,G-sensor and Gyro-sensor 2in1 bmi120
obj-$(CONFIG_ACC_GRYO_BMI120)	+= bmi120/
#yongzhong.cheng@tcl.com,2016/4/12,defect 1938927,G-sensor and Gyro-sensor 2in1 bmi120

