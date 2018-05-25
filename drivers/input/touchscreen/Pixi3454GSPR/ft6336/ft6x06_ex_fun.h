
#ifndef __LINUX_FT6X06_EX_FUN_H__
#define __LINUX_FT6X06_EX_FUN_H__

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>

#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>



#define FT_UPGRADE_AA	0xAA
#define FT_UPGRADE_55 	0x55


//upgrade config of FT6X06
/*
#define FT6X06_UPGRADE_AA_DELAY 		100
#define FT6X06_UPGRADE_55_DELAY 		10
#define FT6X06_UPGRADE_ID_1			0x79
#define FT6X06_UPGRADE_ID_2			0x08
#define FT6X06_UPGRADE_READID_DELAY 	10
#define FT6X06_UPGRADE_EARSE_DELAY	2000
*/

/*upgrade config of FT6X36*/
#define FT6X06_UPGRADE_AA_DELAY 		10
#define FT6X06_UPGRADE_55_DELAY 		10
#define FT6X06_UPGRADE_ID_1			0x79
#define FT6X06_UPGRADE_ID_2			0x18
#define FT6X06_UPGRADE_READID_DELAY 	10
#define FT6X06_UPGRADE_EARSE_DELAY	2000

#define FTS_PACKET_LENGTH        128
#define FTS_SETTING_BUF_LEN        128

#define FTS_UPGRADE_LOOP	20

#define FTS_FACTORYMODE_VALUE		0x40
#define FTS_WORKMODE_VALUE		0x00

//#define    AUTO_CLB
#define FTS_DBG
#ifdef FTS_DBG
#define DBG(fmt, args...) printk("[FTS]" fmt, ## args)
#else
#define DBG(fmt, args...) do{}while(0)
#endif

/*create sysfs for debug*/
int ft6x06_create_sysfs(struct i2c_client * client);

void ft6x06_release_sysfs(struct i2c_client * client);

int ft6x06_create_apk_debug_channel(struct i2c_client *client);

void ft6x06_release_apk_debug_channel(void);

/*
*ft6x06_write_reg- write register
*@client: handle of i2c
*@regaddr: register address
*@regvalue: register value
*
*/
int ft6x06_write_reg(struct i2c_client * client,u8 regaddr, u8 regvalue);

int ft6x06_read_reg(struct i2c_client * client,u8 regaddr, u8 *regvalue);

int fts_ctpm_auto_upgrade(struct i2c_client *client);

int fts_ctpm_update_project_setting_6x06(struct i2c_client *client);

#endif

