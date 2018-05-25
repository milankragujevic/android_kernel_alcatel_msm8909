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
#include <soc/qcom/watchdog.h>

#define MAX_CPU_CTX_SIZE	2048
#define MAGIC 0x42445953

struct cpu32_ctxt_regs_type
{
    uint64_t u1, u2, r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12;
    uint64_t r13_usr;
    uint64_t r14_usr;
    uint64_t r13_hyp;
    uint64_t r14_irq;
    uint64_t r13_irq;
    uint64_t r14_svc;
    uint64_t r13_svc;
    uint64_t r14_abt;
    uint64_t r13_abt;
    uint64_t r14_und;
    uint64_t r13_und;
    uint64_t r8_fiq;
    uint64_t r9_fiq;
    uint64_t r10_fiq;
    uint64_t r11_fiq;
    uint64_t r12_fiq;
    uint64_t r13_fiq;
    uint64_t r14_fiq;
    uint64_t pc;
    uint64_t cpsr;
    uint64_t r13_mon;
    uint64_t r14_mon;
    uint64_t r14_hyp;
    uint64_t _reserved[5];
};

struct cpu32_secure_ctxt_regs_type
{
    uint64_t r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12;
    uint64_t r13_usr;
    uint64_t r14_usr;
    uint64_t r13_hyp;
    uint64_t r14_irq;
    uint64_t r13_irq;
    uint64_t r14_svc;
    uint64_t r13_svc;
    uint64_t r14_abt;
    uint64_t r13_abt;
    uint64_t r14_und;
    uint64_t r13_und;
    uint64_t r8_fiq;
    uint64_t r9_fiq;
    uint64_t r10_fiq;
    uint64_t r11_fiq;
    uint64_t r12_fiq;
    uint64_t r13_fiq;
    uint64_t r14_fiq;
    uint64_t pc;
    uint64_t cpsr;
    uint64_t r13_mon;
    uint64_t r14_mon;
    uint64_t r14_hyp;
    uint64_t _reserved[5];
};

struct cpu_ctxt_regs
{
     struct cpu32_ctxt_regs_type cpu32_regs;
     struct cpu32_secure_ctxt_regs_type secure_contex;
};

struct msm_dump_table {
	uint32_t version;
	uint32_t num_entries;
	struct msm_dump_entry entries[MAX_NUM_ENTRIES];
};

struct wdogdump {
	struct msm_dump_table *table;
	void *buffer;
	unsigned long mem_address;
	unsigned long mem_size;
};

static struct wdogdump wdogdump = {
	.mem_address = 0x9fa80000,
	.mem_size = SZ_16K,
};

static struct msm_dump_table *msm_dump_get_table(enum msm_dump_table_ids id)
{
	struct msm_dump_table *table = wdogdump.table;
	int i;

	if (!table) {
		pr_err("mem dump base table does not exist\n");
		return ERR_PTR(-EINVAL);
	}

	for (i = 0; i < MAX_NUM_ENTRIES; i++) {
		if (table->entries[i].id == id)
			break;
	}
	if (i == MAX_NUM_ENTRIES || !table->entries[i].addr) {
		pr_err("mem dump base table entry %d invalid\n", id);
		return ERR_PTR(-EINVAL);
	}

	/* Get the apps table pointer */
	table = phys_to_virt(table->entries[i].addr);

	return table;
}

static struct msm_dump_entry *msm_dump_get_entry(
		enum msm_dump_table_ids tid, enum msm_dump_data_ids did)
{
	struct msm_dump_entry *e = NULL;
	struct msm_dump_table *table;
	int i;

	table = msm_dump_get_table(tid);
	if (IS_ERR_OR_NULL(table))
		return NULL;

	for (i = 0; i < MAX_NUM_ENTRIES; i++) {
		e = &table->entries[i];
		if (e->id == did)
			break;
	}

	return e;
}

static inline char *find_symbol(char *buffer, unsigned long addr)
{
	sprint_symbol(buffer, addr);
	return buffer;
}

static void dump_cpu_regs(struct seq_file *m, struct cpu_ctxt_regs *regs)
{
	char s[256];

	seq_printf(m, "PC is at: %s <0x%08llx>\n",
			find_symbol(s, regs->cpu32_regs.pc), regs->cpu32_regs.pc);
	seq_printf(m, "LR is at: %s <0x%08llx>\n",
			find_symbol(s, regs->cpu32_regs.r14_svc), regs->cpu32_regs.r14_svc);

	seq_printf(m, "regs:\n");
	seq_printf(m, "\tr0\t= 0x%016llx\n", regs->cpu32_regs.r0);
	seq_printf(m, "\tr1\t= 0x%016llx\n", regs->cpu32_regs.r1);
	seq_printf(m, "\tr2\t= 0x%016llx\n", regs->cpu32_regs.r2);
	seq_printf(m, "\tr3\t= 0x%016llx\n", regs->cpu32_regs.r3);
	seq_printf(m, "\tr4\t= 0x%016llx\n", regs->cpu32_regs.r4);
	seq_printf(m, "\tr5\t= 0x%016llx\n", regs->cpu32_regs.r5);
	seq_printf(m, "\tr6\t= 0x%016llx\n", regs->cpu32_regs.r6);
	seq_printf(m, "\tr7\t= 0x%016llx\n", regs->cpu32_regs.r7);
	seq_printf(m, "\tr8\t= 0x%016llx\n", regs->cpu32_regs.r8);
	seq_printf(m, "\tr9\t= 0x%016llx\n", regs->cpu32_regs.r9);
	seq_printf(m, "\tr10\t= 0x%016llx\n", regs->cpu32_regs.r10);
	seq_printf(m, "\tr11\t= 0x%016llx\n", regs->cpu32_regs.r11);
	seq_printf(m, "\tr12\t= 0x%016llx\n", regs->cpu32_regs.r12);

	seq_printf(m, "\tr13_usr\t= 0x%016llx\n", regs->cpu32_regs.r13_usr);
	seq_printf(m, "\tr14_usr\t= 0x%016llx\n", regs->cpu32_regs.r14_usr);
	seq_printf(m, "\tr13_hyp\t= 0x%016llx\n", regs->cpu32_regs.r13_hyp);
	seq_printf(m, "\tr14_irq\t= 0x%016llx [%s]\n",
			regs->cpu32_regs.r14_irq, find_symbol(s, regs->cpu32_regs.r14_irq));
	seq_printf(m, "\tr13_irq\t= 0x%016llx\n", regs->cpu32_regs.r13_irq);
	seq_printf(m, "\tr14_svc\t= 0x%016llx [%s]\n",
			regs->cpu32_regs.r14_svc, find_symbol(s, regs->cpu32_regs.r14_svc));
	seq_printf(m, "\tr13_svc\t= 0x%016llx\n", regs->cpu32_regs.r13_svc);
	seq_printf(m, "\tr14_abt\t= 0x%016llx [%s]\n",
			regs->cpu32_regs.r14_abt, find_symbol(s, regs->cpu32_regs.r14_abt));
	seq_printf(m, "\tr13_abt\t= 0x%016llx\n", regs->cpu32_regs.r13_abt);
	seq_printf(m, "\tr14_und\t= 0x%016llx [%s]\n",
			regs->cpu32_regs.r14_und, find_symbol(s, regs->cpu32_regs.r14_und));
	seq_printf(m, "\tr13_und\t= 0x%016llx\n", regs->cpu32_regs.r13_und);
	seq_printf(m, "\tr8_fiq\t= 0x%016llx\n", regs->cpu32_regs.r8_fiq);
	seq_printf(m, "\tr9_fiq\t= 0x%016llx\n", regs->cpu32_regs.r9_fiq);
	seq_printf(m, "\tr10_fiq\t= 0x%016llx\n", regs->cpu32_regs.r10_fiq);
	seq_printf(m, "\tr11_fiq\t= 0x%016llx\n", regs->cpu32_regs.r11_fiq);
	seq_printf(m, "\tr12_fiq\t= 0x%016llx\n", regs->cpu32_regs.r12_fiq);
	seq_printf(m, "\tr13_fiq\t= 0x%016llx\n", regs->cpu32_regs.r13_fiq);
	seq_printf(m, "\tr14_fiq\t= 0x%016llx [%s]\n",
			regs->cpu32_regs.r14_fiq, find_symbol(s, regs->cpu32_regs.r14_fiq));
	seq_printf(m, "\tpc\t= 0x%016llx [%s]\n",
			regs->cpu32_regs.pc, find_symbol(s, regs->cpu32_regs.pc));
	seq_printf(m, "\tcpsr\t= 0x%016llx\n", regs->cpu32_regs.cpsr);
	seq_printf(m, "\tr13_mon\t= 0x%016llx\n", regs->cpu32_regs.r13_mon);
	seq_printf(m, "\tr14_mon\t= 0x%016llx [%s]\n",
			regs->cpu32_regs.r14_mon, find_symbol(s, regs->cpu32_regs.r14_mon));
	seq_printf(m, "\tr14_hyp\t= 0x%016llx [%s]\n",
			regs->cpu32_regs.r14_hyp, find_symbol(s, regs->cpu32_regs.r14_hyp));

	seq_printf(m, "=============== secure contex ===========\n");
	seq_printf(m, "\tr0\t= 0x%016llx\n", regs->secure_contex.r0);
	seq_printf(m, "\tr1\t= 0x%016llx\n", regs->secure_contex.r1);
	seq_printf(m, "\tr2\t= 0x%016llx\n", regs->secure_contex.r2);
	seq_printf(m, "\tr3\t= 0x%016llx\n", regs->secure_contex.r3);
	seq_printf(m, "\tr4\t= 0x%016llx\n", regs->secure_contex.r4);
	seq_printf(m, "\tr5\t= 0x%016llx\n", regs->secure_contex.r5);
	seq_printf(m, "\tr6\t= 0x%016llx\n", regs->secure_contex.r6);
	seq_printf(m, "\tr7\t= 0x%016llx\n", regs->secure_contex.r7);
	seq_printf(m, "\tr8\t= 0x%016llx\n", regs->secure_contex.r8);
	seq_printf(m, "\tr9\t= 0x%016llx\n", regs->secure_contex.r9);
	seq_printf(m, "\tr10\t= 0x%016llx\n", regs->secure_contex.r10);
	seq_printf(m, "\tr11\t= 0x%016llx\n", regs->secure_contex.r11);
	seq_printf(m, "\tr12\t= 0x%016llx\n", regs->secure_contex.r12);

	seq_printf(m, "\tr13_usr\t= 0x%016llx\n", regs->secure_contex.r13_usr);
	seq_printf(m, "\tr14_usr\t= 0x%016llx\n", regs->secure_contex.r14_usr);
	seq_printf(m, "\tr13_hyp\t= 0x%016llx\n", regs->secure_contex.r13_hyp);
	seq_printf(m, "\tr14_irq\t= 0x%016llx [%s]\n",
			regs->secure_contex.r14_irq, find_symbol(s, regs->secure_contex.r14_irq));
	seq_printf(m, "\tr13_irq\t= 0x%016llx\n", regs->secure_contex.r13_irq);
	seq_printf(m, "\tr14_svc\t= 0x%016llx [%s]\n",
			regs->secure_contex.r14_svc, find_symbol(s, regs->secure_contex.r14_svc));
	seq_printf(m, "\tr13_svc\t= 0x%016llx\n", regs->secure_contex.r13_svc);
	seq_printf(m, "\tr14_abt\t= 0x%016llx [%s]\n",
			regs->secure_contex.r14_abt, find_symbol(s, regs->secure_contex.r14_abt));
	seq_printf(m, "\tr13_abt\t= 0x%016llx\n", regs->secure_contex.r13_abt);
	seq_printf(m, "\tr14_und\t= 0x%016llx [%s]\n",
			regs->secure_contex.r14_und, find_symbol(s, regs->secure_contex.r14_und));
	seq_printf(m, "\tr13_und\t= 0x%016llx\n", regs->secure_contex.r13_und);
	seq_printf(m, "\tr8_fiq\t= 0x%016llx\n", regs->secure_contex.r8_fiq);
	seq_printf(m, "\tr9_fiq\t= 0x%016llx\n", regs->secure_contex.r9_fiq);
	seq_printf(m, "\tr10_fiq\t= 0x%016llx\n", regs->secure_contex.r10_fiq);
	seq_printf(m, "\tr11_fiq\t= 0x%016llx\n", regs->secure_contex.r11_fiq);
	seq_printf(m, "\tr12_fiq\t= 0x%016llx\n", regs->secure_contex.r12_fiq);
	seq_printf(m, "\tr13_fiq\t= 0x%016llx\n", regs->secure_contex.r13_fiq);
	seq_printf(m, "\tr14_fiq\t= 0x%016llx [%s]\n",
			regs->secure_contex.r14_fiq, find_symbol(s, regs->secure_contex.r14_fiq));
	seq_printf(m, "\tpc\t= 0x%016llx [%s]\n",
			regs->secure_contex.pc, find_symbol(s, regs->secure_contex.pc));
	seq_printf(m, "\tcpsr\t= 0x%016llx\n", regs->secure_contex.cpsr);
	seq_printf(m, "\tr13_mon\t= 0x%016llx\n", regs->secure_contex.r13_mon);
	seq_printf(m, "\tr14_mon\t= 0x%016llx [%s]\n",
			regs->secure_contex.r14_mon, find_symbol(s, regs->secure_contex.r14_mon));
	seq_printf(m, "\tr14_hyp\t= 0x%016llx [%s]\n",
			regs->secure_contex.r14_hyp, find_symbol(s, regs->secure_contex.r14_hyp));
	seq_printf(m, "============ end secure context ===========\n");
}

static int wdogdump_proc_show(struct seq_file *m, void *v)
{
	int cpu;
	struct msm_dump_data *cpu_data =
		(struct msm_dump_data *)(wdogdump.buffer);
	struct cpu_ctxt_regs *regs =
		(struct cpu_ctxt_regs *)(wdogdump.buffer +
				num_present_cpus() * sizeof(struct msm_dump_data));

	for_each_cpu(cpu, cpu_present_mask) {
		if (cpu_data[cpu].magic == MAGIC) {
			seq_printf(m, "# cpu%d\n", cpu);
			dump_cpu_regs(m, (struct cpu_ctxt_regs *)((void *)regs + cpu * MAX_CPU_CTX_SIZE));
		} else {
			seq_printf(m, "cpu%d dump magic not match, skip.\n", cpu);
		}
	}

	return 0;
}

static int wdogdump_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, wdogdump_proc_show, NULL);
}

static const struct file_operations wdogdump_proc_fops = {
	.open = wdogdump_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t wdog_trigger_proc_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	msm_trigger_wdog_bite();

	return count;
}

static const struct file_operations wdog_trigger_proc_fops = {
	.write = wdog_trigger_proc_write,
};

static int __init init_wdogdump_proc(void)
{
	proc_create("last_cpu", S_IRWXU, NULL, &wdogdump_proc_fops);
	proc_create("watchdog", S_IWUSR, NULL, &wdog_trigger_proc_fops);

	return 0;
}

int check_watchdog_dump_magic(void)
{
	struct msm_dump_data *cpu_data =
		(struct msm_dump_data *)(wdogdump.buffer);

	return cpu_data[0].magic == MAGIC;
}
EXPORT_SYMBOL(check_watchdog_dump_magic);

static struct msm_dump_table *get_memdump_table(void)
{
	struct device_node *np;
	void __iomem *imem_base;
	u32 table_phys;
	struct msm_dump_table *table;

	np = of_find_compatible_node(NULL, NULL,
				     "qcom,msm-imem-mem_dump_table");
	if (!np) {
		pr_err("mem dump base table DT node does not exist\n");
		return ERR_PTR(-ENODEV);
	}

	imem_base = of_iomap(np, 0);
	if (!imem_base) {
		pr_err("mem dump base table imem offset mapping failed\n");
		return ERR_PTR(-ENOMEM);
	}

	table_phys = readl_relaxed(imem_base);

	iounmap(imem_base);

	table = phys_to_virt(table_phys);
	return table;
}

int __init earlyinit_wdog_memdump(void)
{
	phys_addr_t base = wdogdump.mem_address;
	phys_addr_t size = wdogdump.mem_size;

	wdogdump.buffer = vmalloc(size);
	if (!wdogdump.buffer)
		return -ENOMEM;

	/* copy last wdogdump to our buffer and zero fill it */
	memcpy(wdogdump.buffer, phys_to_virt(base), size);
	memset(phys_to_virt(base), 0, size);

	/* for test */
	do {
		int cpu;
		struct msm_dump_data *cpu_data =
			(struct msm_dump_data *)(wdogdump.buffer);
		for_each_cpu(cpu, cpu_present_mask) {
			pr_info("cpu%d: ver=0x%x magic=0x%x\n", cpu,
					cpu_data[cpu].version, cpu_data[cpu].magic);
		}
	} while (0);

	return 0;
}

static int __init init_wdog_memdump(void)
{
	struct msm_dump_table *table;
	struct msm_dump_entry *entry;
	struct msm_dump_data *cpu_data;
	void *cpu_buf;
	int cpu;

	earlyinit_wdog_memdump();

	wdogdump.table = get_memdump_table();

	cpu_data = (struct msm_dump_data *)phys_to_virt(wdogdump.mem_address);
	cpu_buf = (void *)phys_to_virt(wdogdump.mem_address +
		num_present_cpus() * sizeof(struct msm_dump_data));

	/* change the watchdog dump address to our own reserve range */
	for_each_cpu(cpu, cpu_present_mask) {
		entry = msm_dump_get_entry(MSM_DUMP_TABLE_APPS,
				MSM_DUMP_DATA_CPU_CTX + cpu);
		if ((!entry) || (entry->type != MSM_DUMP_TYPE_DATA)) {
			pr_err("Failed to find dump entry for cpu%d\n", cpu);
			continue;
		}
		cpu_data[cpu].addr = virt_to_phys(cpu_buf + cpu * MAX_CPU_CTX_SIZE);
		cpu_data[cpu].len = MAX_CPU_CTX_SIZE;
		entry->addr = virt_to_phys(&cpu_data[cpu]);
	}

	table = msm_dump_get_table(MSM_DUMP_TABLE_APPS);
	dmac_flush_range(table, (void *)table + sizeof(struct msm_dump_table));

	init_wdogdump_proc();

	return 0;
}
device_initcall(init_wdog_memdump);

/* call from board reserve */
int __init wdog_dump_memblock_reserve(void)
{
	phys_addr_t base = wdogdump.mem_address;
	phys_addr_t size = wdogdump.mem_size;
	int err;

	/* reserve mem for wdogdump */
	err = memblock_reserve(base, size);
	pr_info("reserve [%#016llx-%#016llx] for wdogdump: %s.\n",
			(unsigned long long)base,
			(unsigned long long)base+size,
			err ? "FAIL" : "SUCCESS");

	return err;
}

