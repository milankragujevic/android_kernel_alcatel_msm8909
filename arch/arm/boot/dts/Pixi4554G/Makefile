ifeq ($(CONFIG_OF),y)

# [PLATFORM]-Mod-BEGIN by shengwang.luo, 2015/06/17, optimize dts file for msm8909
dtb-$(CONFIG_ARCH_MSM8909) += msm8909-1gb-mtp.dtb \
         msm8909-mtp-smb1360.dtb \
         msm8909-mtp.dtb
#       msm8909-1gb-mtp.dtb \
 
# [PLATFORM]-Mod-END by shengwang.luo


DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif

targets += dtbs
targets += $(addprefix ../, $(DTB_LIST))
endif

$(obj)/../%.dtb: $(src)/%.dts FORCE
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../,$(DTB_LIST))

clean-files := *.dtb
