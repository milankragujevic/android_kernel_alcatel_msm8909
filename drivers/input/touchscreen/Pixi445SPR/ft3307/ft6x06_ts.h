/*
 *
 * FocalTech ft5x06 TouchScreen driver header file.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
 
 
#ifndef __LINUX_FT5X06_TS_H__
#define __LINUX_FT5X06_TS_H__

#define FT5X06_ID		0x55
#define FT5X16_ID		0x0A
#define FT5X36_ID		0x14
#define FT6X06_ID		0x06
#define VIRTUAL_KEY		0  //modify virtual key by gong.lan for 1 to 0 PR:882844 2014/14/24
#define CONFIG_TOUCHSCREEN_FT6X06_FIRMWARE 1
struct fw_upgrade_info {
	bool auto_cal;
	u16 delay_aa;
	u16 delay_55;
	u8 upgrade_id_1;
	u8 upgrade_id_2;
	u16 delay_readid;
	u16 delay_erase_flash;
};

struct ft6x06_ts_platform_data {
	struct fw_upgrade_info info;
	const char *name;
	const char *fw_name;
	u32 irqflags;
	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 reset_gpio;
	u32 reset_gpio_flags;
	u32 family_id;
	u32 x_max;
	u32 y_max;
	u32 x_min;
	u32 y_min;
	u32 panel_minx;
	u32 panel_miny;
	u32 panel_maxx;
	u32 panel_maxy;
	u32 group_id;
	u32 hard_rst_dly;
	u32 soft_rst_dly;
	u32 num_max_touches;
	bool fw_vkey_support;
	bool no_force_update;
	bool i2c_pull_up;
#if VIRTUAL_KEY
	//const char *name;
	int disp_maxx;
	int disp_maxy;
	int pan_maxx;
	int pan_maxy;

	int *keycodes;
	int num_keys;
	int y_offset;
#endif
    bool ignore_id_check;
	int (*power_init) (bool);
	int (*power_on) (bool);
};

#endif

#if defined CONFIG_TOUCHSCREEN_FT6X06_FIRMWARE
#define IC_FT5X06	0
#define IC_FT5606	1
#define IC_FT5316	2
#define IC_FT5X36	3
#define IC_FT6306	4

#define FT_UPGRADE_AA	0xAA
#define FT_UPGRADE_55 	0x55
#define FT_UPGRADE_EARSE_DELAY		2000

/*upgrade config of FT5606*/
#define FT5606_UPGRADE_AA_DELAY 		50
#define FT5606_UPGRADE_55_DELAY 		10
#define FT5606_UPGRADE_ID_1			0x79
#define FT5606_UPGRADE_ID_2			0x06
#define FT5606_UPGRADE_READID_DELAY 	100

/*upgrade config of FT5316*/
#define FT5316_UPGRADE_AA_DELAY 		50
#define FT5316_UPGRADE_55_DELAY 		40
#define FT5316_UPGRADE_ID_1			0x79
#define FT5316_UPGRADE_ID_2			0x07
#define FT5316_UPGRADE_READID_DELAY 	1

/*upgrade config of FT5x06(x=2,3,4)*/
#define FT5X06_UPGRADE_AA_DELAY 		50
#define FT5X06_UPGRADE_55_DELAY 		30
#define FT5X06_UPGRADE_ID_1			0x79
#define FT5X06_UPGRADE_ID_2			0x03
#define FT5X06_UPGRADE_READID_DELAY 	1

/*upgrade config of FT5X36*/
#define FT5X36_UPGRADE_AA_DELAY 		30
#define FT5X36_UPGRADE_55_DELAY 		30
#define FT5X36_UPGRADE_ID_1			0x79
#define FT5X36_UPGRADE_ID_2			0x11
#define FT5X36_UPGRADE_READID_DELAY 	10

/*upgrade config of FT6306*/
#define FT6306_UPGRADE_AA_DELAY 		80
#define FT6306_UPGRADE_55_DELAY 		30
#define FT6306_UPGRADE_ID_1			0x79
#define FT6306_UPGRADE_ID_2			0x08
#define FT6306_UPGRADE_READID_DELAY 	1

#define DEVICE_IC_TYPE	IC_FT6306

#define FTS_PACKET_LENGTH        128
#define FTS_SETTING_BUF_LEN        128

#define BL_VERSION_LZ4        0
#define BL_VERSION_Z7        1
#define BL_VERSION_GZF        2

#define FTS_TX_MAX				40
#define FTS_RX_MAX				40
#define FTS_DEVICE_MODE_REG	0x00
#define FTS_TXNUM_REG			0x03
#define FTS_RXNUM_REG			0x04
#define FTS_RAW_READ_REG		0x01
#define FTS_RAW_BEGIN_REG		0x10
#define FTS_VOLTAGE_REG		0x05

#define FTS_FACTORYMODE_VALUE		0x40
#define FTS_WORKMODE_VALUE		0x00

/*register address*/
#define FT5x0x_REG_FW_VER		0xA6
#define FT5x0x_REG_POINT_RATE	0x88
#define FT5X0X_REG_THGROUP	0x80

#define FT6x06_REG_FW_VER		0xA6
#define FT_REG_VENDOR_ID    0xA8
#define FT_REG_ID		0xA3

#define FT6x06_REG_PROJECT_ID 0xBD

/*[BUGFIX]-Add-BEGIN by TCTNB.XQJ, 2013/12/05, refer to bug 564812 for tp upgrade*/
#define IS_TRULY_TP 0x5a
#define IS_BIEL_TP 0x3b
int fts_ctpm_fw_i_file_config(struct i2c_client *client);
/*[BUGFIX]-Add-END by TCTNB.XQJ */
int fts_ctpm_update_project_setting(struct i2c_client *client);

int ft6x06_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen);
int ft6x06_i2c_write(struct i2c_client *client, char *writebuf, int writelen);

int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client *client);
u8 fts_ctpm_get_i_file_ver(void);
int fts_ctpm_auto_clb(struct i2c_client *client);

/*create sysfs for debug*/
int ft5x0x_create_sysfs(struct i2c_client * client);
void ft5x0x_release_mutex(void);

#endif

