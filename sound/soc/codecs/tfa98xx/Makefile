# make sure that the environment variables ARCH and CROSS_COMPILE 
# are set for your architecture and cross compiler
#
# e.g. for Beaglebone black:
# export ARCH := arm
# export CROSS_COMPILE := arm-linux-gnueabihf-
#
# e.g. Samsung Galaxy S6:
# export ARCH := arm64
# export CROSS_COMPILE := aarch64-linux-android-

#MODULE_NAME = snd-soc-tfa98xx

# additional debugging
#TFA_DEBUG=1

#TOPLEVEL=$(PWD)/..

# debugging support (also enables trace_printk)
EXTRA_CFLAGS += -DDEBUG

#EXTRA_CFLAGS += -I$(TOPLEVEL)/inc
#EXTRA_CFLAGS += -I$(TOPLEVEL)/linux-kernel-inc # config.h

#EXTRA_CFLAGS += -Iinc
#EXTRA_CFLAGS += -Ilinux-kernel-inc # config.h

#EXTRA_CFLAGS += -Werror

snd-soc-tfa98xx-objs += src/tfa98xx.o
snd-soc-tfa98xx-objs += src/tfa_container.o
snd-soc-tfa98xx-objs += src/tfa_dsp.o
snd-soc-tfa98xx-objs += src/tfa9888_init.o
#snd-soc-tfa98xx-objs += src/tfa9891_init.o
#snd-soc-tfa98xx-objs += src/tfa9897_init.o
#snd-soc-tfa98xx-objs += src/tfa9890_init.o
#snd-soc-tfa98xx-objs += src/tfa9887B_init.o
#snd-soc-tfa98xx-objs += src/tfa9887_init.o

obj-y += snd-soc-tfa98xx.o

ifdef TFA_DEBUG
#EXTRA_CFLAGS += -DTFA_DEBUG -DDEBUG
snd-soc-tfa98xx-objs += ../src/tfa_debug.o
endif

#MAKEARCH := $(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE)

#all:
#	$(MAKEARCH) -C $(KDIR) M=$(PWD) modules

#clean:
#	$(MAKEARCH) -C $(KDIR) M=$(PWD) clean
#	rm -f $(snd-soc-tfa98xx-objs)


