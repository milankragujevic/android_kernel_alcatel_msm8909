#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/pstore.h>
#include <linux/pstore_ram.h>
#include <linux/platform_device.h>
#include <linux/memblock.h>

#define MIN_MEM_SIZE 4096UL

static struct platform_device *ramoops;

static struct ramoops_platform_data ramoops_pdata = {
	.mem_size = SZ_512K,
	.mem_address = 0x9fa00000,
	.record_size = MIN_MEM_SIZE,
	.console_size = SZ_256K,
	.ftrace_size = MIN_MEM_SIZE,
	.dump_oops = 1,
	.ecc_info = {
		.ecc_size = 0,
	}
};

int __init ramoops_memblock_reserve(void)
{
	phys_addr_t base = ramoops_pdata.mem_address;
	phys_addr_t size = ramoops_pdata.mem_size;
	int err;

	err = memblock_reserve(base, size);

	pr_info("reserve [%#016llx-%#016llx] for ramoops: %s.\n",
			(unsigned long long)base,
			(unsigned long long)base+size,
			err ? "FAIL" : "SUCCESS");

	return err;
}

static int __init ramoops_init(void)
{
	if (!memblock_is_region_reserved(ramoops_pdata.mem_address, ramoops_pdata.mem_size)) {
		pr_err("no memblock reserved for ramoops.\n");
		return -ENOMEM;
	}

	ramoops = platform_device_register_data(NULL, "ramoops",
			PLATFORM_DEVID_AUTO, &ramoops_pdata, sizeof(ramoops_pdata));
	if (IS_ERR(ramoops)) {
		pr_err("could not create ramoops pdev: %ld\n", PTR_ERR(ramoops));
		return PTR_ERR(ramoops);
	}

	return 0;
}

device_initcall(ramoops_init);

/*
 * If we enable ftrace *after* module loaded,
 * ftrace will unable to modify module's text section,
 * so switch to RW between modify.
 */
#ifdef CONFIG_DYNAMIC_FTRACE
int ftrace_arch_code_modify_prepare(void)
{
	set_all_modules_text_rw();
	return 0;
}

int ftrace_arch_code_modify_post_process(void)
{
	set_all_modules_text_ro();
	return 0;
}
#endif
