#
# Makefile for the input core drivers.
#

# Each configuration option enables a list of files.
#obj-$(CONFIG_TOUCHSCREEN_FTS) += fts_ts.o 
#fts_ts-y += focaltech_core.o focaltech_ctl.o focaltech_ex_fun.o focaltech_gesture.o focaltech_flash.o 
#extlibs:=$(srctree)/drivers/input/touchscreen/focaltech/ft_gesture_32bit_lib_v1.0_20140820.a
#ldflagobj-y += fts_ts.o
#obj-y s-y +=--whole-archive $(extlibs) --no-whole-archive
#ldflags-y +=--whole-archive $(extlibs) --no-whole-archive

obj-$(CONFIG_TOUCHSCREEN_FTS_FT5346I) += focaltech_core.o focaltech_ctl.o focaltech_ex_fun.o focaltech_gesture.o focaltech_flash.o
extlibs:=$(srctree)/$(src)/ft_gesture_32bit_lib_v1.0_20140820.a
ldflags-y +=--whole-archive $(extlibs) --no-whole-archive 

