/**
 * mlog.c - a log channel between MSM AP and MODEM
 *
 * use to read message from MODEM and write to kmsg.
 *
 * This code is licenced under the GPL.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/platform_device.h>
#include <soc/qcom/smd.h>
#include <soc/qcom/smsm.h>

/**
 * struct mlog - private data struct
 *
 * @channel: smd channel
 * @kwork: the read work
 * @kworker: the worker used to read data from smd
 * @task: where the worker run on
 * @buf: the temp buffer to read data into
 * @sz_buf: size of buffer
 * @proc: procfs debug node
 * @reader: reader counter
 */
struct mlog {
	smd_channel_t *channel;
	struct kthread_work kwork;
	struct kthread_worker kworker;
	struct task_struct *task;
	char *buf;
	int sz_buf;
	struct proc_dir_entry *proc;
	atomic_t reader;
	struct mutex lock;
	struct list_head queue;
	wait_queue_head_t wait;
};

struct mlog_record {
	char *buf;
	struct list_head node;
};

static struct mlog *g_mlog;

/**
 * alloc_mlog_record - alloc mlog_record and its buffer
 */
static struct mlog_record *alloc_mlog_record(int len)
{
	struct mlog_record *record;

	record = kmalloc(sizeof(*record), GFP_KERNEL);
	if (!record)
		return NULL;

	record->buf = kmalloc(len, GFP_KERNEL);
	if (!record->buf) {
		kfree(record);
		return NULL;
	}

	INIT_LIST_HEAD(&record->node);

	return record;
}

static void free_mlog_record(struct mlog_record *record)
{
	if (record) {
		kfree(record->buf);
		kfree(record);
	}
}

/**
 * mlog_read_data - read modem message and print to kmsg
 */
static void mlog_read_data(struct kthread_work *work)
{
	struct mlog *log =
		container_of(work, struct mlog, kwork);
	int avail;

	while ((avail = smd_read_avail(log->channel)) > 0) {
		smd_read(log->channel, log->buf, avail);
		printk(KERN_INFO "%s", log->buf);

		if (atomic_read(&log->reader)) {
			int len = strlen(log->buf) + 1;
			struct mlog_record *r = alloc_mlog_record(len);
			if (!r)
				continue;

			snprintf(r->buf, len, "%s", log->buf);

			mutex_lock(&log->lock);
			list_add_tail(&r->node, &log->queue);
			mutex_unlock(&log->lock);

			wake_up(&log->wait);
		}
	}
}

/**
 * mlog_notify - mlog smd's event-callback
 */
static void mlog_notify(void *priv, unsigned int smd_event)
{
	struct mlog *log = priv;

	switch(smd_event) {
	case SMD_EVENT_DATA:
		if (smd_read_avail(log->channel) > 0) {
			queue_kthread_work(&log->kworker, &log->kwork);
		}
		break;
	case SMD_EVENT_OPEN:
		break;
	case SMD_EVENT_CLOSE:
		break;
	}
}

static int mlog_probe(struct platform_device *pdev)
{
	struct mlog *log = g_mlog;
	int ret;

	printk(KERN_INFO "mlog startup.\n");

	if (!log)
		return -EFAULT;

	init_kthread_worker(&log->kworker);
	log->task = kthread_run(kthread_worker_fn,
			&log->kworker, "MODEM/mlog");
	if (IS_ERR(log->task)) {
		printk(KERN_ERR "Failed to create kthread.\n");
		return -ENOMEM;
	}
	init_kthread_work(&log->kwork, mlog_read_data);

	ret = smd_named_open_on_edge("mlog",
			SMD_APPS_MODEM,
			&log->channel,
			log,
			mlog_notify);
	if (ret < 0) {
		printk(KERN_ERR "Failed to open smd[mlog] %d.\n", ret);
		flush_kthread_worker(&log->kworker);
		kthread_stop(log->task);
		return ret;
	}

	smd_disable_read_intr(log->channel);
	smsm_change_state(SMSM_APPS_STATE, 0, SMSM_RPCINIT);

	return 0;
}

static int mlog_remove(struct platform_device *pdev)
{
	struct mlog *log = g_mlog;

	printk(KERN_INFO "mlog shutdown.\n");

	if (log) {
		smd_close(log->channel);
		flush_kthread_worker(&log->kworker);
		kthread_stop(log->task);
	}

	return 0;
}

/**
 * qualcomm will register a "mlog" platform_device
 * when modem's mlog channel is opened, so we register
 * a platform_driver here.
 */
static struct platform_driver mlog_driver = {
	.driver = {
		.name = "mlog",
	},
	.probe = mlog_probe,
	.remove = mlog_remove,
};

#ifdef CONFIG_PROC_FS
static ssize_t mlog_proc_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	struct mlog *log = g_mlog;
	char *kbuf;

	kbuf = kmalloc(count+1, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	strncpy_from_user(kbuf, buf, count);

	kbuf[count] = '\0';

	if (smd_write_avail(log->channel) >= (count+1))
		count = smd_write(log->channel, kbuf, count+1);

	kfree(kbuf);

	return count;
}

static ssize_t mlog_proc_read(struct file *file, char __user *buf,
			 size_t count, loff_t *ppos)
{
	struct mlog *log = g_mlog;
	struct mlog_record *new, *next;
	int n, total = 0;

	wait_event_interruptible(log->wait, !list_empty(&log->queue));

	mutex_lock(&log->lock);

	list_for_each_entry_safe(new, next, &log->queue, node) {

		n = strlen(new->buf);
		if (n > count)
			break;

		if (copy_to_user(buf, new->buf, n))
			break;

		buf += n;
		count -= n;
		total += n;

		list_del_init(&new->node);
		free_mlog_record(new);
	}

	mutex_unlock(&log->lock);

	return total;
}

static int mlog_proc_open(struct inode *inode, struct file *file)
{
	struct mlog *log = g_mlog;

	if ((file->f_flags & O_ACCMODE) == O_RDONLY)
		if (atomic_xchg(&log->reader, 1) != 0)
			return -EBUSY;

	return 0;
}

static int mlog_proc_release(struct inode *inode, struct file *file)
{
	struct mlog *log = g_mlog;

	if ((file->f_flags & O_ACCMODE) == O_RDONLY)
		atomic_set(&log->reader, 0);

	return 0;
}

/**
 * "cat /proc/mlog"			- dump out the modem log
 * "echo xxx > /proc/mlog"	- write to modem and it will loopback the message to us
 */
static const struct file_operations mlog_proc_fops = {
	.read = mlog_proc_read,
	.write = mlog_proc_write,
	.open = mlog_proc_open,
	.release = mlog_proc_release,
	.llseek = no_llseek,
};
#endif

static int __init mlog_init(void)
{
	int ret;
	struct mlog *log;

	printk(KERN_INFO "mlog module.\n");

	log = kzalloc(sizeof(*log), GFP_KERNEL);
	if (!log)
		return -ENOMEM;

	log->sz_buf = 8*1024;
	log->buf = kmalloc(log->sz_buf, GFP_KERNEL);
	if (!log) {
		printk(KERN_ERR "Failed to kmalloc.\n");
		kfree(log);
		return -ENOMEM;
	}

	atomic_set(&log->reader, 0);
	mutex_init(&log->lock);
	INIT_LIST_HEAD(&log->queue);
	init_waitqueue_head(&log->wait);

	g_mlog = log;

	ret = platform_driver_register(&mlog_driver);
	if (ret) {
		printk(KERN_ERR "Failed to register mlog_driver.\n");
		kfree(log->buf);
		kfree(log);
		return ret;
	}

#ifdef CONFIG_PROC_FS
	log->proc = proc_create("mlog", 0600, NULL, &mlog_proc_fops);
#endif

	return 0;
}
module_init(mlog_init);

static void __exit mlog_exit(void)
{
	if (g_mlog) {
		struct mlog *log = g_mlog;
#ifdef CONFIG_PROC_FS
		proc_remove(log->proc);
#endif
		platform_driver_unregister(&mlog_driver);
		kfree(log->buf);
		kfree(log);
	}
}
module_exit(mlog_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("A module to get log from modem through smd.");
MODULE_AUTHOR("liangxiaoju <xiaoju.liang@tcl.com>");
