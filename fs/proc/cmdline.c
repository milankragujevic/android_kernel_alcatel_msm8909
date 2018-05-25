#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/jrd_project.h>	//add by junfeng.zhou for all_in_one

static int cmdline_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", saved_command_line);
	return 0;
}

static int cmdline_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cmdline_proc_show, NULL);
}

static const struct file_operations cmdline_proc_fops = {
	.open		= cmdline_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

//add by junfeng.zhou for all_in_one begin
#ifdef  TCT_SW_ALL_IN_ONE
static ssize_t versiontrace_read(struct file *f, char __user *buf,
				       size_t count, loff_t *ppos)
{
	return simple_read_from_buffer(buf, count, ppos, static_version_trace, 100);
}

static const struct file_operations versiontrace_fops = {
	.open	= simple_open,
	.read	= versiontrace_read,	
};
#endif
//add by junfeng.zhou for all_in_one end

static int __init proc_cmdline_init(void)
{
	proc_create("cmdline", 0, NULL, &cmdline_proc_fops);
//add by junfeng.zhou for all_in_one begin
#ifdef  TCT_SW_ALL_IN_ONE
	proc_create("versiontrace", (S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH ), NULL,&versiontrace_fops);
#endif
//add by junfeng.zhou for all_in_one end
	return 0;
}
module_init(proc_cmdline_init);
