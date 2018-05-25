/*
 *
 * Author:TCTSZ wenzhao.guo@tcl.com
 * Date:1.0 05-11-2015
 *      1.1 07-17-2015
 * Version:1.1
 * Task:433879
 * Description:Record the kernel boot time and provide it for Android.
 *             /proc/boot_time record the kernel boot time. 
 *             /proc/log_ts control the timestamp switch.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/aio.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#define LOG_TS_FILE    "log_ts"
#define LOG_BOOT_TIME_FILE    "boot_time"

int g_ts_switch = 0; // 0: android default timestamp; 1: kernel timestamp

static int ts_switch_proc_show(struct seq_file *m, void *v)
{

    seq_printf(m, "%d\n", g_ts_switch);
    return 0;
}

static ssize_t ts_switch_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	if (count) {
		char ts_switch;

		if (get_user(ts_switch, buf))
			return -EFAULT;
		switch(ts_switch) {
		case '0':
			g_ts_switch = 0;
			printk(KERN_INFO "logger: ts_switch_write g_ts_switch == 0\n");
			break;
		case '1':
			g_ts_switch = 1;
			printk(KERN_INFO "logger: ts_switch_write g_ts_switch == 1\n");
			break;
		default:
			printk(KERN_ERR "logger: ts_switch_write incorrect parameter\n");
			break;
		}
	}

	return count;
}

static int ts_switch_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ts_switch_proc_show, NULL);
}


static const struct file_operations ts_switch_proc_fops = {
	.open		= ts_switch_proc_open,
	.read		= seq_read,
	.write          = ts_switch_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int init_ts_switch_proc(void)
{
	g_ts_switch = 0; // 0=using android default log timestamp,1=kernel timestamp
	
	if (!proc_create(LOG_TS_FILE, S_IRWXU, NULL,&ts_switch_proc_fops)) { 	
		printk(KERN_ERR "JCH: init_log_proc create_proc_entry fails\n");
		return 1;
	}

	return 0;
}

ssize_t boot_time_proc_show(struct seq_file *m, void *v)
{
	unsigned long long t;
	unsigned long nanosec_rem;
	
	t = cpu_clock(UINT_MAX);
	nanosec_rem = do_div(t, 1000 * 1000 * 1000);
	
	if (g_ts_switch == 1)
		return seq_printf(m, "%5lu.%06lu", (unsigned long)t, nanosec_rem / 1000);
	else
		return 0;
}

static int boot_time_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, boot_time_proc_show, NULL);
}

static const struct file_operations boot_time_proc_fops = {
	.open		= boot_time_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int init_boot_time_proc(void)
{
	if (!proc_create(LOG_BOOT_TIME_FILE, S_IRWXU, NULL, &boot_time_proc_fops)) { 	
		printk(KERN_ERR "init_boot_time_proc create_proc_entry fails\n");
		return 1;
	}

	return 0;
}

static int __init boot_time_init(void)
{
	int ret = 0;
	ret = init_ts_switch_proc();
	if(ret) {
		printk(KERN_INFO "init_ts_switch_proc fail!\n");
		return ret;
	}
	ret = init_boot_time_proc();
	if(ret) {
		printk(KERN_INFO "init_boot_time_proc fail!\n");
		return ret;
	}
	return ret;
}

static void __exit boot_time_exit(void)
{
	
}

module_init(boot_time_init);
module_exit(boot_time_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cloud Guo, <wenzhao.guo@tcl.com>");
MODULE_DESCRIPTION("Kernel boot time");

