#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/printk.h>
#include <linux/rwsem.h>
#include <linux/uaccess.h>
#include <net/genetlink.h>
#include <linux/klog.h>

static struct genl_family klog_fam = {
	.id = GENL_ID_GENERATE,
	.name = "klog",
	.hdrsize = 0,
	.version = 1,
	.maxattr = NL_KLOG_ATTR_MAX,
	.netnsok = true,
};

/* XXX: add your group here */
static struct genl_multicast_group klog_grps[] = {
	{
		.name		= "ssr",
	},
	{
		.name		= "fb",
	},
	{
		.name		= "other",
	},
	{
		.name		= "mmc",
	},
};

static int klog_msg_event(unsigned int group,
		unsigned int attr, struct klog_msg *msg)
{
	struct sk_buff *skb;
	void *hdr;
	static unsigned int event_seqnum;

	skb = genlmsg_new(NLMSG_DEFAULT_SIZE, GFP_ATOMIC);
	if (!skb)
		return -ENOMEM;

	hdr = genlmsg_put(skb, 0, event_seqnum++, &klog_fam, 0, NL_KLOG_CMD_EVENT);
	if (!hdr) {
		nlmsg_free(skb);
		return -ENOMEM;
	}

	if (nla_put(skb, attr, sizeof(*msg), msg)) {
    	genlmsg_cancel(skb, hdr);
		nlmsg_free(skb);
    	return -EMSGSIZE;
	}

	if (genlmsg_end(skb, hdr) < 0) {
		nlmsg_free(skb);
		return -ENOMEM;
	}

	return genlmsg_multicast(skb, 0, group, GFP_ATOMIC);
}

int klog_print(const char *name, const char *fmt, ...)
{
	va_list args;
	unsigned int group;
	int err;
	struct klog_msg *msg;
	int index = -1;
	int i;

	for (i = 0; i < ARRAY_SIZE(klog_grps); i++) {
		if (strncmp(name, klog_grps[i].name,
					strlen(klog_grps[i].name)) == 0)
			index = i;
	}

	if (index < 0) {
		printk(KERN_ERR "Please add your group [%s] to klog_grps.\n", name);
		return -EINVAL;
	}

	msg = kzalloc(sizeof(*msg), GFP_ATOMIC);
	if (!msg)
		return -ENOMEM;

	msg->id = index;
	msg->ts_nsec = local_clock();

	va_start(args, fmt);
	vsnprintf(msg->data, sizeof(msg->data), fmt, args);
	va_end(args);

	group = klog_grps[index].id;
	strncpy(msg->label, klog_grps[index].name, sizeof(msg->label));

    //[Feature]-Mod-BEGIN by TCTSZ. print the subsystem crash log wenzhao.guo@tcl.com, 2015/08/06, for [Task-403392]
	pr_info("[%d] %s", index, msg->data);
    //[Feature]-Mod-END by TCTSZ. print the subsystem crash log wenzhao.guo@tcl.com, 2015/08/06, for [Task-403392]
	err = klog_msg_event(group, NL_KLOG_ATTR_FIRST, msg);
	if (err)
		printk(KERN_ERR "Failed to klog_print(%s) %d\n", msg->label, err);

	kfree(msg);

	return err;
}
EXPORT_SYMBOL(klog_print);

static ssize_t klog_proc_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char *msg;
	char *name = "other";
	int i;

	if (count == 0)
		return 0;

	msg = kmalloc(count+1, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	if (copy_from_user(msg, buf, count)) {
		printk(KERN_ERR "Failed to copy_from_user().\n");
		goto out;
	}
	msg[count] = '\0';

	for (i = 0; i < ARRAY_SIZE(klog_grps); i++) {
		if (strncmp(msg, klog_grps[i].name,
					strlen(klog_grps[i].name)) == 0)
			name = klog_grps[i].name;
	}

	klog_print(name, "%s", msg);

out:
	kfree(msg);
	return count;
}

static int klog_proc_show(struct seq_file *m, void *v)
{
	int i;

	seq_printf(m, "Family: %s\n", klog_fam.name);
	seq_printf(m, "Groups:");
	for (i = 0; i < ARRAY_SIZE(klog_grps); i++) {
		seq_printf(m, " %s", klog_grps[i].name);
	}
	seq_printf(m, "\n");

	return 0;
}

static int klog_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, klog_proc_show, NULL);
}

static const struct file_operations klog_proc_fops = {
	.open = klog_proc_open,
	.read = seq_read,
	.write = klog_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct proc_dir_entry *proc;

static int __init klog_genl_init(void)
{
	int err, i;

	err = genl_register_family(&klog_fam);
	if (err < 0)
		return err;

	for (i = 0; i < ARRAY_SIZE(klog_grps); i++) {
		if (strlen(klog_grps[i].name) == 0)
			continue;

		err = genl_register_mc_group(&klog_fam, &klog_grps[i]);
		if (err < 0)
			goto out;
	}

	proc = proc_create("klog", S_IRWXU, NULL, &klog_proc_fops);
	if (!proc)
		goto out;

	return 0;
out:
	genl_unregister_family(&klog_fam);
	return err;
}
fs_initcall(klog_genl_init);

static void __exit klog_genl_exit(void)
{
	proc_remove(proc);
	genl_unregister_family(&klog_fam);
}
module_exit(klog_genl_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Export log to userspace through generic netlink.");
MODULE_AUTHOR("liangxiaoju <xiaoju.liang@tcl.com>");
