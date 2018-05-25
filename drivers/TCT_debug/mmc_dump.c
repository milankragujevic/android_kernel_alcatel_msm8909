#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/printk.h>
#include <linux/atomic.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <linux/mmc/core.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/klog.h>

#define MMC_DUMP(fmt, ...)	mmc_print(fmt, ##__VA_ARGS__)

struct mmc_record {
	struct mmc_command *sbc;
	struct mmc_command *cmd;
	struct mmc_data *data;
	struct mmc_command *stop;

	struct list_head node;
	struct mmc_host *host;
	int id;
	int error;
	void *reserved[0];
};

struct mem_pool {
	void *va_addr;
	int max_nr;
	int curr_nr;
	int unit;
};

struct mmc_dump {
	struct task_struct *kthread;
	struct list_head queue, user_queue;
	wait_queue_head_t wait, user_wait;
	spinlock_t lock, user_lock;
	struct mem_pool pool, user_pool;
	struct proc_dir_entry *proc;
	atomic_t user;
};

static struct mmc_dump *mmc_dump;
/* TODO: mmc_ramdump.py -- get mmc dump info from 'mmc_dump_phys_addr' */
static phys_addr_t mmc_dump_phys_addr;

static struct mmc_record *grab_next_buffer(struct mmc_dump *dump, int for_user)
{
	struct mmc_record *new;
	struct mem_pool *pool;
	spinlock_t *lock;
	unsigned long flags;

	if (for_user) {
		/* grab free buffer for userspace */
		pool = &dump->user_pool;
		lock = &dump->user_lock;
	} else {
		pool = &dump->pool;
		lock = &dump->lock;
	}

	new = pool->va_addr + pool->unit * pool->curr_nr;
	pool->curr_nr = (pool->curr_nr + 1) % pool->max_nr;

	spin_lock_irqsave(lock, flags);

	if (!list_empty(&new->node) && !for_user)
		pr_info_ratelimited("overrun id %d user %d\n", new->id, for_user);

	list_del_init(&new->node);

	spin_unlock_irqrestore(lock, flags);

	new->sbc = NULL;
	new->cmd = NULL;
	new->data = NULL;
	new->stop = NULL;

	return new;
}

static void init_record_from_mrq(struct mmc_record *new,
		struct mmc_request *mrq)
{
	void *reserved = (void *)(new->reserved);

	new->error = 0;

	if (unlikely(mrq->sbc)) {
		new->sbc = reserved;
		memcpy(new->sbc, mrq->sbc, sizeof(struct mmc_command));
		new->error |= new->sbc->error;
	}

	if (likely(mrq->cmd)) {
		new->cmd = reserved + sizeof(struct mmc_command);
		memcpy(new->cmd, mrq->cmd, sizeof(struct mmc_command));
		new->error |= new->cmd->error;
	}

	if (mrq->data) {
		new->data = reserved + sizeof(struct mmc_command)*2;
		memcpy(new->data, mrq->data, sizeof(struct mmc_data));
		new->error |= new->data->error;
	}

	if (mrq->stop) {
		new->stop = reserved + sizeof(struct mmc_command)*2 +
			sizeof(struct mmc_data);
		memcpy(new->stop, mrq->stop, sizeof(struct mmc_command));
		new->error |= new->stop->error;
	}
}

static void init_record_from_old(struct mmc_record *new,
		struct mmc_record *old)
{
	void *reserved = (void *)(new->reserved);

	new->error = 0;

	if (unlikely(old->sbc)) {
		new->sbc = reserved;
		memcpy(new->sbc, old->sbc, sizeof(struct mmc_command));
		new->error |= new->sbc->error;
	}

	if (likely(old->cmd)) {
		new->cmd = reserved + sizeof(struct mmc_command);
		memcpy(new->cmd, old->cmd, sizeof(struct mmc_command));
		new->error |= new->cmd->error;
	}

	if (old->data) {
		new->data = reserved + sizeof(struct mmc_command)*2;
		memcpy(new->data, old->data, sizeof(struct mmc_data));
		new->error |= new->data->error;
	}

	if (old->stop) {
		new->stop = reserved + sizeof(struct mmc_command)*2 +
			sizeof(struct mmc_data);
		memcpy(new->stop, old->stop, sizeof(struct mmc_command));
		new->error |= new->stop->error;
	}
}

void mmc_dump_event(struct mmc_host *host, struct mmc_request *mrq)
{
	struct mmc_record *new;
	struct mmc_dump *dump = mmc_dump;
	unsigned long flags;

	if (!dump)
		return;

	/* only dump mmc0 */
	if (host && host->index != 0)
		return;

	new = grab_next_buffer(dump, 0);
	if (!new)
		return;

	init_record_from_mrq(new, mrq);
	new->host = host;

	spin_lock_irqsave(&dump->lock, flags);
	list_add_tail(&new->node, &dump->queue);
	spin_unlock_irqrestore(&dump->lock, flags);

	wake_up(&dump->wait);
}
EXPORT_SYMBOL(mmc_dump_event);

static void dump_record(struct mmc_record *record)
{
	struct mmc_command *sbc, *cmd, *stop;
	struct mmc_data *data;
	struct mmc_host *host;

	sbc = record->sbc;
	cmd = record->cmd;
	stop = record->stop;
	data = record->data;
	host = record->host;

	if (sbc)
		MMC_DUMP("%s sbc: CMD%u: arg %08x flags %08x err %d\n",
				mmc_hostname(host), sbc->opcode, sbc->arg, sbc->flags, sbc->error);

	if (cmd)
		MMC_DUMP("%s cmd: CMD%u: arg %08x flags %08x "
				"resp %08x %08x %08x %08x err %d\n",
				mmc_hostname(host), cmd->opcode, cmd->arg, cmd->flags,
				cmd->resp[0], cmd->resp[1], cmd->resp[2], cmd->resp[3],
				cmd->error);

	if (data)
		MMC_DUMP("%s data: blksz %d blocks %d flags %08x "
				"tsac %d ms nsac %d "
				"transferred %d err %d\n",
				mmc_hostname(host), data->blksz, data->blocks, data->flags,
				data->timeout_ns / 1000000, data->timeout_clks,
				data->bytes_xfered, data->error);

	if (stop)
		MMC_DUMP("%s stop: CMD%u: arg %08x flags %08x"
				"resp %08x %08x %08x %08x err %d\n",
				mmc_hostname(host), stop->opcode, stop->arg, stop->flags,
				stop->resp[0], stop->resp[1], stop->resp[2], stop->resp[3],
				stop->error);
}

static int dump_record_to_buffer(struct mmc_record *record, char *buffer, int max)
{
	struct mmc_command *sbc, *cmd, *stop;
	struct mmc_data *data;
	struct mmc_host *host;
	char *b = buffer;
	int n;

	sbc = record->sbc;
	cmd = record->cmd;
	stop = record->stop;
	data = record->data;
	host = record->host;

	if (sbc) {
		n = snprintf(b, max, "%s sbc: CMD%u: arg %08x flags %08x err %d\n",
				mmc_hostname(host), sbc->opcode, sbc->arg, sbc->flags, sbc->error);
		max -= n;
		b += n;
	}

	if (cmd) {
		n = snprintf(b, max, "%s cmd: CMD%u: arg %08x flags %08x "
				"resp %08x %08x %08x %08x err %d\n",
				mmc_hostname(host), cmd->opcode, cmd->arg, cmd->flags,
				cmd->resp[0], cmd->resp[1], cmd->resp[2], cmd->resp[3],
				cmd->error);
		max -= n;
		b += n;
	}

	if (data) {
		n = snprintf(b, max, "%s data: blksz %d blocks %d flags %08x "
				"tsac %d ms nsac %d "
				"transferred %d err %d\n",
				mmc_hostname(host), data->blksz, data->blocks, data->flags,
				data->timeout_ns / 1000000, data->timeout_clks,
				data->bytes_xfered, data->error);
		max -= n;
		b += n;
	}

	if (stop) {
		n = snprintf(b, max, "%s stop: CMD%u: arg %08x flags %08x"
				"resp %08x %08x %08x %08x err %d\n",
				mmc_hostname(host), stop->opcode, stop->arg, stop->flags,
				stop->resp[0], stop->resp[1], stop->resp[2], stop->resp[3],
				stop->error);
		max -= n;
		b += n;
	}

	return strlen(buffer);
}

static void dump_last_records(struct mmc_dump *dump, int head_id, int max_nr)
{
	struct mmc_record *tail, *p;
	int i, curr, nr, start;

	MMC_DUMP("===== dump last records =====\n");
	tail = list_entry(dump->queue.prev, struct mmc_record, node);
	nr = (head_id - tail->id + dump->pool.max_nr) % dump->pool.max_nr;
	nr = min(nr, max_nr);
	start = (head_id - nr + 1 + dump->pool.max_nr) % dump->pool.max_nr;
	for (i = 0, curr = start; i < nr; i++, curr++) {
		curr = curr % dump->pool.max_nr;
		p = dump->pool.va_addr + curr * dump->pool.unit;
		dump_record(p);
	}
	MMC_DUMP("\n");
}

static int dump_error_filter(struct mmc_record *r)
{
	/* only dump mmc0 */
	if (r->host && r->host->index != 0)
		return 0;

	if (r->cmd) {
		switch (r->cmd->opcode) {
		case MMC_SEND_TUNING_BLOCK_HS200:
		case MMC_SEND_STATUS:
			return 0;
		case 52:
		case 55:
		case 8:
		case 5:
			/* ignore these errs if mmc is initializing */
			if (r->host && (r->host->card == NULL))
				return 0;
		default:
			return 1;
		}
	}

	return 1;
}

static void handle_records(struct mmc_dump *dump)
{
	struct list_head *tmp;
	struct mmc_record *record, *urecord;
	unsigned long flags;

	do {
		spin_lock_irqsave(&dump->lock, flags);

		if (list_empty(&dump->queue)) {
			spin_unlock_irqrestore(&dump->lock, flags);
			break;
		}
		tmp = dump->queue.next;
		list_del_init(tmp);

		record = list_entry(tmp, struct mmc_record, node);

		if (record->error) {
			if (dump_error_filter(record))
				dump_last_records(dump, record->id, 10);
		}

		if (atomic_read(&dump->user)) {
			urecord = grab_next_buffer(dump, 1);
			init_record_from_old(urecord, record);
			urecord->host = record->host;
		}

		spin_unlock_irqrestore(&dump->lock, flags);

		if (atomic_read(&dump->user)) {
			spin_lock_irqsave(&dump->user_lock, flags);
			list_add_tail(&urecord->node, &dump->user_queue);
			spin_unlock_irqrestore(&dump->user_lock, flags);
			wake_up(&dump->user_wait);
		}

	} while (1);
}

static int mmc_dump_kthread_fn(void *data)
{
	struct mmc_dump *dump = data;

	set_freezable();

	do {
		handle_records(dump);
		wait_event_freezable(dump->wait,
				!list_empty(&dump->queue) ||
				kthread_should_stop());
	} while (!kthread_should_stop());

	return 0;
}

static int mmc_dump_open(struct inode *inode, struct file *file)
{
	struct mmc_dump *dump = mmc_dump;

	atomic_inc(&dump->user);

	return 0;
}

static int mmc_dump_release(struct inode *inode, struct file *file)
{
	struct mmc_dump *dump = mmc_dump;

	atomic_dec(&dump->user);

	return 0;
}

static unsigned int mmc_dump_poll(struct file *file, poll_table *wait)
{
	struct mmc_dump *dump = mmc_dump;

	poll_wait(file, &dump->user_wait, wait);
	if (!list_empty(&dump->user_queue))
		return POLLIN | POLLRDNORM;
	return 0;
}

static ssize_t mmc_dump_read(struct file *file, char __user *buf,
			 size_t count, loff_t *ppos)
{
	struct mmc_dump *dump = mmc_dump;
	struct mmc_record *new, *next;
	unsigned long flags;
	char *buffer;
	int bsize = 1024;
	int n, total = 0;

	buffer = kzalloc(bsize, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	wait_event_interruptible(dump->user_wait,
			!list_empty(&dump->user_queue));

	spin_lock_irqsave(&dump->user_lock, flags);
	list_for_each_entry_safe(new, next, &dump->user_queue, node) {
		n = dump_record_to_buffer(new, buffer, bsize);
		if (n > count)
			break;

		if (copy_to_user(buf, buffer, n))
			break;

		buf += n;
		count -= n;
		total += n;

		list_del_init(&new->node);
	}
	spin_unlock_irqrestore(&dump->user_lock, flags);

	kfree(buffer);

	return total;
}

static ssize_t mmc_dump_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	return count;
}

static const struct file_operations mmc_dump_proc_fops = {
	.open = mmc_dump_open,
	.release = mmc_dump_release,
	.poll = mmc_dump_poll,
	.read = mmc_dump_read,
	.write = mmc_dump_write,
	.llseek = no_llseek,
};

static int init_pool(struct mem_pool *pool, int max_nr)
{
	void *addr;
	int i;

	pool->max_nr = max_nr;
	pool->unit = sizeof(struct mmc_record) +
		sizeof(struct mmc_command) * 3 + sizeof(struct mmc_data);

	addr = kzalloc(pool->unit * pool->max_nr, GFP_KERNEL);
	if (!addr)
		return -ENOMEM;

	pool->va_addr = addr;
	pool->curr_nr = 0;

	for (i = 0; i < pool->max_nr; i++) {
		struct mmc_record *tmp = pool->va_addr + i * pool->unit;
		INIT_LIST_HEAD(&tmp->node);
		tmp->id = i;
	}

	return 0;
}

static int __init mmc_dump_init(void)
{
	int ret;
	struct task_struct *kthread;
	struct mmc_dump *dump;

	dump = kzalloc(sizeof(*dump), GFP_KERNEL);
	if (!dump)
		return -ENOMEM;

	atomic_set(&dump->user, 0);
	spin_lock_init(&dump->lock);
	spin_lock_init(&dump->user_lock);
	INIT_LIST_HEAD(&dump->queue);
	INIT_LIST_HEAD(&dump->user_queue);
	init_waitqueue_head(&dump->wait);
	init_waitqueue_head(&dump->user_wait);

	kthread = kthread_run(mmc_dump_kthread_fn, dump, "mmc_dump");
	if (IS_ERR(kthread)) {
		ret = PTR_ERR(kthread);
		goto free_dump;
	}
	dump->kthread = kthread;

	if (init_pool(&dump->pool, 2048) < 0)
		goto destroy_kthread;

	mmc_dump_phys_addr = virt_to_phys(dump->pool.va_addr);

	if (init_pool(&dump->user_pool, 1024) < 0)
		goto destroy_pool;

	dump->proc = proc_create("mmc", S_IRWXU, NULL, &mmc_dump_proc_fops);
	if (!dump->proc)
		goto destroy_pool;

	mmc_dump = dump;

	printk(KERN_INFO "mmc_dump init.\n");

	return 0;

destroy_pool:
	if (dump->pool.va_addr)
		kfree(dump->pool.va_addr);
	if (dump->user_pool.va_addr)
		kfree(dump->user_pool.va_addr);
destroy_kthread:
	kthread_stop(kthread);
free_dump:
	kfree(dump);
	return ret;
}

static void __exit mmc_dump_exit(void)
{
	struct mmc_dump *dump = mmc_dump;

	proc_remove(dump->proc);
	kfree(dump->pool.va_addr);
	kfree(dump->user_pool.va_addr);
	kthread_stop(dump->kthread);
	kfree(dump);
}

subsys_initcall(mmc_dump_init);
module_exit(mmc_dump_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("A dumper for mmc.");
MODULE_AUTHOR("liangxiaoju <xiaoju.liang@tcl.com>");
