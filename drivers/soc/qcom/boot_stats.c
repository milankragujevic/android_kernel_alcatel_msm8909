/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <soc/qcom/smem.h>

struct boot_stats {
	uint32_t bootloader_start;
	uint32_t bootloader_end;
	uint32_t bootloader_display;
	uint32_t bootloader_load_kernel;
};

static void __iomem *mpm_counter_base;
static uint32_t mpm_counter_freq;
static struct boot_stats __iomem *boot_stats;

static int mpm_parse_dt(void)
{
	struct device_node *np;
	u32 freq;

	np = of_find_compatible_node(NULL, NULL, "qcom,msm-imem-boot_stats");
	if (!np) {
		pr_err("can't find qcom,msm-imem node\n");
		return -ENODEV;
	}
	boot_stats = of_iomap(np, 0);
	if (!boot_stats) {
		pr_err("boot_stats: Can't map imem\n");
		return -ENODEV;
	}

	np = of_find_compatible_node(NULL, NULL, "qcom,mpm2-sleep-counter");
	if (!np) {
		pr_err("mpm_counter: can't find DT node\n");
		return -ENODEV;
	}

	if (!of_property_read_u32(np, "clock-frequency", &freq))
		mpm_counter_freq = freq;
	else
		return -ENODEV;

	if (of_get_address(np, 0, NULL, NULL)) {
		mpm_counter_base = of_iomap(np, 0);
		if (!mpm_counter_base) {
			pr_err("mpm_counter: cant map counter base\n");
			return -ENODEV;
		}
	}

	return 0;
}

//[Feature]-Add-BEGIN by TCTSZ. yongzhong.cheng@tcl.com,2015/6/25,for ALM366701:print reason in kernel
void get_power_onoff_reason0(void)
{   
    u8 pon_sts=0;
	u16 poff_sts,warm_reset,soft_reset;
	u32 GCC_RESET_STATUS;
	char pwr_on_reason[180]={0};
    uint32_t* temp_ret;

	temp_ret = smem_alloc(SMEM_PON_POFF_REASON, 5*sizeof(uint32_t),0,SMEM_ANY_HOST_FLAG);
	if(temp_ret!=NULL){
		pon_sts=(u8)*temp_ret;
		poff_sts=(u16)*(temp_ret+1);
		warm_reset=(u16)*(temp_ret+2);
		soft_reset=(u16)*(temp_ret+3);
		GCC_RESET_STATUS=(u32)*(temp_ret+4);
	}else{
		pr_info("get_power_onoff_reason0 error");
		return;
	}

	pr_info("\n pon_reason2:pon_sts=0x%x,poff_sts=0x%x,warm_reset=0x%x,soft_reset=0x%x,GCC=0x%x\n",
		pon_sts,poff_sts,warm_reset,soft_reset,GCC_RESET_STATUS);
	
	sprintf(pwr_on_reason, "Power-on reason: 0x%hhx ", pon_sts);
    if(pon_sts & 0x80)
		strcat(pwr_on_reason, " KPD (power key press),");
	if(pon_sts & 0x40)
		strcat(pwr_on_reason, " CBL (external power supply),");
	if(pon_sts & 0x20)
		strcat(pwr_on_reason, " PON1 (secondary PMIC),");
	if(pon_sts & 0x10)
		strcat(pwr_on_reason, " USB (USB charger insertion),");
	if(pon_sts & 0x08)
		strcat(pwr_on_reason, " DC (DC charger insertion),");
	if(pon_sts & 0x04)
		strcat(pwr_on_reason, " RTC (RTC alarm expiry),");
	if(pon_sts & 0x02)
		strcat(pwr_on_reason, " SMPL (sudden momentary power loss),");
	if(pon_sts & 0x01)
		strcat(pwr_on_reason, " Hard Reset");

    pr_info("%s\n",pwr_on_reason);

	sprintf(pwr_on_reason, "Power-off reason: 0x%hx  ", poff_sts);
      switch(poff_sts) {
		case 0x8000:
			strcat(pwr_on_reason, "stage3 reset");
			break;
		case 0x4000:
			strcat(pwr_on_reason, "Overtemp");
			break;
		case 0x2000:
			strcat(pwr_on_reason, "UVLO");
			break;
		case 0x1000:
			strcat(pwr_on_reason, "TFT");
			break;
		case 0x0800:
			strcat(pwr_on_reason, "Charger");
			break;
		case 0x0400:
			strcat(pwr_on_reason, "AVDD_RB");
			break;

		case 0x80:
			strcat(pwr_on_reason, "KPDPWR_N");
			break;
		case 0x40:
			strcat(pwr_on_reason, "RESIN_N");
			break;
		case 0x20:
			strcat(pwr_on_reason, "simultaneous KPDPWR_N + RESIN_N");
			break;
		case 0x10:
			strcat(pwr_on_reason, "Keypad_Reset2");
			break;
		case 0x08:
			strcat(pwr_on_reason, "Keypad_Reset1");
			break;
		case 0x04:
			strcat(pwr_on_reason, "PMIC watchdog");
			break;
		case 0x02:
			strcat(pwr_on_reason, "PS_HOLD");
			break;
		case 0x01:
			strcat(pwr_on_reason, "Software");
			break;
		default:
			break;
	}
    pr_info("%s\n",pwr_on_reason);

    sprintf(pwr_on_reason, "PON_WARM_RESET_REASON: 0x%hx  ", warm_reset);
      switch(warm_reset) {
		case 0x1000:
			strcat(pwr_on_reason, "TFT");
			break;
		case 0x80:
			strcat(pwr_on_reason, "KPDPWR_N");
			break;
		case 0x40:
			strcat(pwr_on_reason, "RESIN_N");
			break;
		case 0x20:
			strcat(pwr_on_reason, "simultaneous KPDPWR_N + RESIN_N");
			break;
		case 0x10:
			strcat(pwr_on_reason, "Keypad_Reset2");
			break;
		case 0x08:
			strcat(pwr_on_reason, "Keypad_Reset1");
			break;
		case 0x04:
			strcat(pwr_on_reason, "PMIC watchdog");
			break;
		case 0x02:
			strcat(pwr_on_reason, "PS_HOLD");
			break;
		case 0x01:
			strcat(pwr_on_reason, "Software");
			break;
		default:
			break;
	}

  pr_info("%s\n",pwr_on_reason);

  sprintf(pwr_on_reason, "PON_SOFT_RESET_REASON: 0x%hx  ", soft_reset);
	switch(soft_reset) {
		case 0x1000:
			strcat(pwr_on_reason, "TFT");
		case 0x80:
			strcat(pwr_on_reason, "KPDPWR_N");
			break;
		case 0x40:
			strcat(pwr_on_reason, "RESIN_N");
			break;
		case 0x20:
			strcat(pwr_on_reason, "KPDPWR_AND_RESIN");
			break;
		case 0x10:
			strcat(pwr_on_reason, "Keypad_Reset2");
			break;
		case 0x08:
			strcat(pwr_on_reason, "Keypad_Reset1");
			break;
		case 0x04:
			strcat(pwr_on_reason, "PMIC watchdog");
			break;
		case 0x02:
			strcat(pwr_on_reason, "PS_HOLD");
			break;
		case 0x01:
			strcat(pwr_on_reason, "Software");
			break;
		default:
			break;
	}

   pr_info("%s\n",pwr_on_reason);

   sprintf(pwr_on_reason, "GCC_RESET_STATUS: 0x%hhx ", GCC_RESET_STATUS);
	  if(GCC_RESET_STATUS & 0x20)
		  strcat(pwr_on_reason, " Secure wdog expired,");
	  if(GCC_RESET_STATUS & 0x10)
		  strcat(pwr_on_reason, " abnormal resin triggered,");
	  if(GCC_RESET_STATUS & 0x08)
		  strcat(pwr_on_reason, " tsensor triggered,");
	  if(GCC_RESET_STATUS & 0x04)
		  strcat(pwr_on_reason, " srst triggered,");
	  if(GCC_RESET_STATUS & 0x02)
		  strcat(pwr_on_reason, " RESERVED2,");
	  if(GCC_RESET_STATUS & 0x01)
		  strcat(pwr_on_reason, " RESERVED1,");

	  pr_info("%s\n",pwr_on_reason);
}
//[Feature]-Add-END by TCTSZ. yongzhong.cheng@TCL.com, 2015/6/25,for ALM366701

static void print_boot_stats(void)
{
	pr_info("KPI: Bootloader start count = %u\n",
		readl_relaxed(&boot_stats->bootloader_start));
	pr_info("KPI: Bootloader end count = %u\n",
		readl_relaxed(&boot_stats->bootloader_end));
	pr_info("KPI: Bootloader display count = %u\n",
		readl_relaxed(&boot_stats->bootloader_display));
	pr_info("KPI: Bootloader load kernel count = %u\n",
		readl_relaxed(&boot_stats->bootloader_load_kernel));
	pr_info("KPI: Kernel MPM timestamp = %u\n",
		readl_relaxed(mpm_counter_base));
	pr_info("KPI: Kernel MPM Clock frequency = %u\n",
		mpm_counter_freq);
//[Feature]-Add-BEGIN by TCTSZ. yongzhong.cheng@tcl.com,2015/6/25,for ALM366701:print reason in kernel
	get_power_onoff_reason0();
//[Feature]-Add-END by TCTSZ. yongzhong.cheng@TCL.com, 2015/6/25,for ALM366701
}

int boot_stats_init(void)
{
	int ret;

	ret = mpm_parse_dt();
	if (ret < 0)
		return -ENODEV;

	print_boot_stats();

	iounmap(boot_stats);
	iounmap(mpm_counter_base);

	return 0;
}

