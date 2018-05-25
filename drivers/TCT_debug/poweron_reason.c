#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/spmi.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/qpnp/power-on.h>
#include <linux/proc_fs.h>

#define RESTART_REASON_PANIC 0x55aa0001

struct pon {
	struct spmi_device *spmi;
	u16 base;
	int powerup_reason;
	void *restart_reason_addr;
	u32 restart_reason;
};

static struct pon *g_pon;

static const char * const powerup_reason[] = {
	[0] = "reboot",
	[1] = "SMPL",
	[2] = "rtc",
	[3] = "DC",
	[4] = "usb_chg",
	[5] = "PON1",
	[6] = "CBL",
	[7] = "keypad",
};

static int powerup_reason_panic(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	__raw_writel(RESTART_REASON_PANIC, g_pon->restart_reason_addr);
	return 0;
}

static struct notifier_block powerup_reason_blk = {
	.notifier_call  = powerup_reason_panic,
};

extern int check_watchdog_dump_magic(void);
static int boot_reason_show(struct seq_file *m, void *v)
{
	int index;

	if (!g_pon)
		return -1;

	if (g_pon->restart_reason == RESTART_REASON_PANIC) {
		/* panic */
		seq_printf(m, "%s", "panic");//[Bugfix]-Add by TCTSZ. wenzhao.guo@tcl.com, 2015/08/04, for [Task-403392]
#ifdef CONFIG_TCT_WDOG_DUMP
	} else if (check_watchdog_dump_magic()) {
		/* wdt */
		seq_printf(m, "%s", "wdt");//[Bugfix]-Add by TCTSZ. wenzhao.guo@tcl.com, 2015/08/04, for [Task-403392]
#endif
	} else {
		index = g_pon->powerup_reason - 1;
		if (index >= ARRAY_SIZE(powerup_reason) || index < 0)
			seq_printf(m, "%s", "others");//[Bugfix]-Add by TCTSZ. wenzhao.guo@tcl.com, 2015/08/04, for [Task-403392]
		else
			seq_printf(m, "%s", powerup_reason[index]);//[Bugfix]-Add by TCTSZ. wenzhao.guo@tcl.com, 2015/08/04, for [Task-403392]
	}

	return 0;
}

static int boot_reason_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, boot_reason_show, NULL);
}

static const struct file_operations boot_reason_proc_fops = {
	.open = boot_reason_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init pon_init(void)
{
	struct pon *pon;

	pon = kzalloc(sizeof(*pon), GFP_KERNEL);
	if (!pon)
		return -ENOMEM;

	g_pon = pon;

	pon->powerup_reason = boot_reason;

	/* TODO: reserve our own memblock */
	pon->restart_reason_addr = phys_to_virt(0x9fa00000+SZ_512K-4);
	pon->restart_reason = __raw_readl(pon->restart_reason_addr);
	__raw_writel(0, pon->restart_reason_addr);

	atomic_notifier_chain_register(&panic_notifier_list,
			&powerup_reason_blk);

	proc_create("boot_reason", 0444, NULL, &boot_reason_proc_fops);

	return 0;
}
late_initcall(pon_init);
