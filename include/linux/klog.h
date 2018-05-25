#ifndef __KLOG_H__
#define __KLOG_H__

#include <linux/printk.h>

enum klog_genl_cmds {
	NL_KLOG_CMD_UNSPEC,
	NL_KLOG_CMD_EVENT,

	__NL_KLOG_CMD_LAST,
	NL_KLOG_CMD_MAX = __NL_KLOG_CMD_LAST - 1,
};

enum klog_genl_attrs {
	NL_KLOG_ATTR_UNSPEC,
	NL_KLOG_ATTR_FIRST,

	__NL_KLOG_ATTR_LAST,
	NL_KLOG_ATTR_MAX = __NL_KLOG_ATTR_LAST - 1,
};

struct klog_msg {
	int id;
	char label[16];
	unsigned long long ts_nsec;
	int level;
	char data[512];
};

#ifdef CONFIG_TCT_DEBUG_KLOG
extern int klog_print(const char *name, const char *fmt, ...);
#else
#define klog_print(tag, fmt, ...) printk(KERN_INFO fmt, ##__VA_ARGS__)
#endif

#define ssr_print(fmt, ...) klog_print("ssr", pr_fmt(fmt), ##__VA_ARGS__)
#define fb_print(fmt, ...) klog_print("fb", pr_fmt(fmt), ##__VA_ARGS__)
#define other_print(fmt, ...) klog_print("other", pr_fmt(fmt), ##__VA_ARGS__)
#define mmc_print(fmt, ...) klog_print("mmc", pr_fmt(fmt), ##__VA_ARGS__)

#endif
