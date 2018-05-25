/*
 * Simple softlockup watchdog regression test module
 *
 * Copyright (C) chenghui.jia@tcl.com.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation.
 *
 * [Feature]-Add-BEGIN by TCTSZ. add watchdog timeout test method wenzhao.guo@tcl.com, 2015/07/28, for [Task-403392]
 */
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/preempt.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <mach/watchdogtest.h>
//add by yusen.ke.sz@tcl.com at 20150723 for checkspc
#include <linux/reboot.h>	
#include <linux/delay.h>
//add end

//add by yusen.ke.sz@tcl.com for fix checkspc ramdump begin
void do_msm_poweroff_freezefb(void);
//add end

struct completion wdt_timeout_complete;
static void wdt_timeout_work(struct work_struct *work)
{
	local_irq_disable();
	while (1) { }
	local_irq_enable();
	complete(&wdt_timeout_complete);
}
static DECLARE_WORK(wdt_timeout_work_struct, wdt_timeout_work);

static int fire_watchdog_reset_set(void *data, u64 val)
{
	init_completion(&wdt_timeout_complete);
	printk(KERN_WARNING "Fire hardware watchdog reset.\n");
	printk(KERN_WARNING "Please wait ...\n");
	schedule_work_on(0, &wdt_timeout_work_struct);
	wait_for_completion(&wdt_timeout_complete);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fire_watchdog_reset_fops,
	NULL, fire_watchdog_reset_set, "%llu\n");

//add by yusen.ke.sz@tcl.com at 20150723 for checkspc
static int fire_freezefb_reset_set(void *data, u64 val)
{
	//touch_hw_watchdog();

	printk(KERN_ERR "Fire freezefb reset.\n");
	printk(KERN_ERR "Please wait ...\n");
	local_irq_disable();
	preempt_disable();
	mdelay(1000UL*60);
	//modify by yusen.ke.sz@tcl.com for fix checkspc ramdump begin
	//preempt_enable();
	//local_irq_enable();//del by yusen.ke.sz@tcl.com for debug
	//kernel_power_off();
	do_msm_poweroff_freezefb();
	//modify end
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fire_freezefb_reset_fops,
	NULL, fire_freezefb_reset_set, "%llu\n");
//add end

static int fire_softlockup_reset_set(void *data, u64 val)
{
	touch_hw_watchdog();

	printk(KERN_WARNING "Fire softlockup watchdog reset.\n");
	printk(KERN_WARNING "Please wait ...\n");
	preempt_disable();
	while (1) { }

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fire_softlockup_reset_fops,
	NULL, fire_softlockup_reset_set, "%llu\n");

static int watchdog_test_init(void)
{
	debugfs_create_file("fire_softlockup_reset", 0200,
		NULL, NULL, &fire_softlockup_reset_fops);
	debugfs_create_file("fire_watchdog_reset", 0200,
		NULL, NULL, &fire_watchdog_reset_fops);
//add by yusen.ke.sz@tcl.com at 20150723 for checkspc
	debugfs_create_file("fire_freezefb_reset", 0200,
		NULL, NULL, &fire_freezefb_reset_fops);
//add end
	return 0;
}

static void watchdog_test_exit(void)
{
}

module_init(watchdog_test_init);
module_exit(watchdog_test_exit);
MODULE_LICENSE("GPL");
