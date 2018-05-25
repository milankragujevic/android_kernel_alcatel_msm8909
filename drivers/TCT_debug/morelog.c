#include <asm/cacheflush.h>
#include <linux/memblock.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/export.h>
#include <linux/err.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_address.h>
#include <linux/kallsyms.h>
#include <soc/qcom/memory_dump.h>

#define MORELOGSIZE SZ_256K
#define MAGIC_START 0x4D4C5354   //MLST
#define MAGIC_END 0x4D4C4544   //MLED

typedef unsigned int uint32_t;
struct loginfo
{
	uint32_t magic;
	uint32_t size;
};

struct morelogdump {
	void *buffer;
	unsigned long mem_address;
	unsigned long mem_size;
};

static struct morelogdump dump = {
	.mem_address = 0x9fa84000,
	.mem_size = MORELOGSIZE,
};
#if 0
char *last = NULL;
void strstr_reverse(const char *str1, const char *str2)
{
    char *index;
    index = strstr(str1,str2);
    if(index != NULL)
    {
        last = index;
        strstr_reverse(index+4,str2);
    }
    return;
}
static ssize_t log_read(struct file *f, char __user *buf,
				       size_t count, loff_t *ppos)
{
    char *start;
    char *buffer = (char *)dump.buffer;
    int size = 0;
    start = strstr(buffer,"MLST");
    strstr_reverse(buffer,"MLED");
    printk(KERN_ERR "start = 0x%x last = 0x%x",(int)start,(int)last);
    if(start == NULL)
        return simple_read_from_buffer(buf, count, ppos, "log is NULL!\n",12);
        if(last == NULL)
            size = SZ_256K;
        else
            size = (last-start)/sizeof(char);
	return simple_read_from_buffer(buf, count, ppos, buffer,size+4);
}
#else
static ssize_t log_read(struct file *f, char __user *buf,
                        size_t count, loff_t *ppos)
{
    struct loginfo info;
    char error[] = "sbl magic is wrong or size is 0 ,dump is destroyed";
    char *buffer = (char *)dump.buffer;
    memcpy(&info,buffer,8);
    //printk(KERN_ERR "start = 0x%x last = 0x%x",info.magic,info.size);
    if(info.magic != MAGIC_START || info.size <= 0)
        return simple_read_from_buffer(buf, count, ppos, error,sizeof(error));
	return simple_read_from_buffer(buf, count, ppos, dump.buffer+8,info.size-8);
}
#endif
static const struct file_operations morelog_fops = {
	.open	= simple_open,
	.read	= log_read,
};
//add by junfeng.zhou for all_in_one end

static int __init init_morelog_dump_proc(void)
{
	proc_create("morelog", (S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH ), NULL,&morelog_fops);
	return 0;
}

int __init save_log_from_memdump(void)
{
	phys_addr_t base = dump.mem_address;
	phys_addr_t size = dump.mem_size;

	dump.buffer = vmalloc(size);
	if (!dump.buffer)
	{
        printk(KERN_ERR "%s malloc memory failed \n",__func__);
        return 0;
    }
	/* copy morelogdump to our buffer and zero fill it */
	memcpy(dump.buffer, phys_to_virt(base), size);
	//memset(phys_to_virt(base), 0, size);
    memblock_free(base,size);
	return 0;
}

static int __init init_morelog_memdump(void)
{
    //save more log to mem
	save_log_from_memdump();
    //creat proc node
	init_morelog_dump_proc();

	return 0;
}
device_initcall(init_morelog_memdump);

/* call from board reserve */
int __init morelog_dump_memblock_reserve(void)
{
	phys_addr_t base = dump.mem_address;
	phys_addr_t size = dump.mem_size;
	int err;

	/* reserve mem for morelogdump */
	err = memblock_reserve(base, size);
	pr_info("reserve [%#016llx-%#016llx] for morelogdump: %s.\n",
			(unsigned long long)base,
			(unsigned long long)base+size,
			err ? "FAIL" : "SUCCESS");

	return err;
}

