/*
 *
 * Author:TCTSZ wenzhao.guo@tcl.com
 * Date:1.0 09-23-2015
 * Version:1.0
 * Task:662897
 * Description:Add a node (/proc/tctprintk) to control the uart log outpurt.
 *             Write 1: Enable the uart console log.
 *             Write 0: Disable the uart console log.
 *
 */
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/kallsyms.h>
#include <linux/utsname.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/pid.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/stacktrace.h>
#include <linux/printk.h>

#define SEQ_printf(m, x...)	    \
 do {			    \
    if (m)		    \
	seq_printf(m, x);	\
    else		    \
	pr_err(x);	    \
 } while (0)

int mt_need_uart_console = 0;
extern void mt_enable_uart(void);	/* printk.c */
extern void mt_disable_uart(void);	/* printk.c */
extern bool printk_disable_uart;

static int mt_printk_ctrl_show(struct seq_file *m, void *v)
{
	SEQ_printf(m, "=== mt printk controller ===\n");
	SEQ_printf(m, "mt_need_uart_console:%d, printk_disable_uart:%d\n", mt_need_uart_console,
		   printk_disable_uart);

	return 0;
}

static ssize_t mt_printk_ctrl_write(struct file *filp, const char __user *ubuf, size_t count, loff_t *data)
{
	char buf[64];
	int val;
	int ret;
	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(&buf, ubuf, count))
		return -EFAULT;

	buf[count] = 0;

	ret = strict_strtoul(buf, 10, (unsigned long *)&val);
	if (val == 0) {
		mt_disable_uart();
	} else if (val == 1) {
		mt_need_uart_console = 1;
		mt_enable_uart();
		pr_err("need uart log\n");
	}
	if (ret < 0)
		return ret;
	pr_err(" %d\n", val);
	return count;
}

static int mt_printk_ctrl_open(struct inode *inode, struct file *file)
{
	return single_open(file, mt_printk_ctrl_show, NULL);
}

static const struct file_operations mt_printk_ctrl_fops = {
	.open = mt_printk_ctrl_open,	
	.read = seq_read,
	.write = mt_printk_ctrl_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init init_mt_printk_ctrl(void)
{
	struct proc_dir_entry *pe;
	mt_need_uart_console = 0;	/* defualt, no uart */
	pe = proc_create("tctprintk", 0664, NULL, &mt_printk_ctrl_fops);
	if (!pe)
		return -ENOMEM;
	return 0;
}

__initcall(init_mt_printk_ctrl);
