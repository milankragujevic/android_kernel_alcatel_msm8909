
obj-$(CONFIG_TCT_DEBUG_BOOTPROF)		+= bootprof.o
obj-$(CONFIG_TCT_DEBUG_SAVE_BOOT_MSG)		+= bootmsg.o
obj-$(CONFIG_TCT_LOG_RAMOOPS)			+= ramoops.o
obj-$(CONFIG_TCT_WDOG_DUMP)			+= watchdog_dump.o
obj-$(CONFIG_TCT_DEBUG_BOOT_REASON)		+= poweron_reason.o
#add dirty file detector for defect 432029 by pingping.jiang 20150715 begin
obj-$(CONFIG_TCT_DIRTY_FILES_DETECTOR) += dirty_file_detector.o
#add dirty file detector for defect 432029 by pingping.jiang 20150715 end
# [Feature]-Add-BEGIN by TCTSZ. add android - kernel timestamp wenzhao.guo@tcl.com, 2015/07/17, for [Task-433879]
obj-$(CONFIG_TCT_DEBUG_BOOT_TIME)		+= logtime.o
# [Feature]-Add-END by TCTSZ. add android - kernel timestamp wenzhao.guo@tcl.com, 2015/07/17, for [Task-433879]

# [Feature]-Add-BEGIN by TCTSZ. add klog. xiaoju.liang@tcl.com, 2015/07/20, for [Task-403392]
obj-$(CONFIG_TCT_DEBUG_KLOG)			+= klog.o
# [Feature]-Add-END by TCTSZ. xiaoju.liang@tcl.com, 2015/07/20, for [Task-403392]
#[Feature]-Add-BEGIN by TCTSZ. add watchdog timeout test method wenzhao.guo@tcl.com, 2015/07/28, for [Task-403392]
obj-$(CONFIG_TCT_DEBUG_WATCHDOG_TEST)			+= watchdog_test.o
#[Feature]-Add-END by TCTSZ. add watchdog timeout test method wenzhao.guo@tcl.com, 2015/07/28, for [Task-403392]
# [Feature]-Add-BEGIN by TCTSZ. add mmc dumper. xiaoju.liang@tcl.com, 2015/08/06, for [Task-403392]
obj-$(CONFIG_TCT_DEBUG_MMC)			+= mmc_dump.o
# [Feature]-Add-END by TCTSZ. xiaoju.liang@tcl.com, 2015/08/06, for [Task-403392]
# [Feature]-Add-BEGIN by TCTSZ. add uart log output controller. wenzhao.guo@tcl.com, 2015/09/23, for [Task-662897]
obj-$(CONFIG_TCT_PRINTK_UART)			+= tct_printk_ctrl.o
# [Feature]-Add-END by TCTSZ. add uart log output controller. wenzhao.guo@tcl.com, 2015/09/23, for [Task-662897]
obj-$(CONFIG_TCT_DEBUG_MLOG)			+= mlog.o
#add by junfeng.zhou for add sbl1 log begin .
obj-$(CONFIG_TCT_DEBUG_MORELOG)			+= morelog.o
#add by junfeng.zhou for add sbl1 log end .
