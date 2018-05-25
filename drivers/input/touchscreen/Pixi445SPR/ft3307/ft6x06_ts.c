/*
 *
 * FocalTech ft6x06 TouchScreen driver.
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

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include<linux/spinlock.h>
#include <linux/kthread.h>
#include "ft6x06_ts.h"

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>

#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define FT_SUSPEND_LEVEL 1
#endif

#define FTS_CTL_IIC
#define SYSFS_DEBUG
#define FTS_APK_DEBUG
#define UPGRADE_FIRMWARE       // update firmware
#define PINCTRL_FT6336
//#define FT6336_GESTURE_FUNCTION  //add by junfeng.zhou
#define FT6336_ESD_PROTECT

struct i2c_client  *ft_g_client=NULL;

#ifdef FT6336_GESTURE_FUNCTION
#define FT5X06_REG_GESTURE_SET    0xd0
#define FT5X06_REG_GESTURE_STATE    0xd3
#define  GESTURE_V 0x14
#define  GESTURE_DB 0x24
#define  GESTURE_C 0x18

#undef  KEY_POWER
#define KEY_POWER  KEY_UNLOCK

struct tp_gesture_data{
    u8 option;
    u8 flag;
    u8 stateReg_val;
};

static struct  tp_gesture_data gesture={
    .option=0x00,
    .flag=0x00,
    .stateReg_val=0x00,
 };
#endif

#ifdef PINCTRL_FT6336
#define PINCTRL_STATE_ACTIVE	"pmx_ts_active"
#define PINCTRL_STATE_SUSPEND	"pmx_ts_suspend"
#define PINCTRL_STATE_RELEASE	"pmx_ts_release"

#endif

#ifdef UPGRADE_FIRMWARE
#include "ft6x06_ex_fun.h"
#endif
//#define FT6X06_DOWNLOAD

#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif

#ifdef SYSFS_DEBUG
#include "ft6x06_ex_fun.h"
#endif


#define FT_DRIVER_VERSION	0x02

#define FT_META_REGS		3
#define FT_ONE_TCH_LEN		6
#define FT_TCH_LEN(x)		(FT_META_REGS + FT_ONE_TCH_LEN * x)

#define FT_PRESS		0x7F
#define FT_MAX_ID		0x0F
#define FT_TOUCH_X_H_POS	3
#define FT_TOUCH_X_L_POS	4
#define FT_TOUCH_Y_H_POS	5
#define FT_TOUCH_Y_L_POS	6
#define FT_TD_STATUS		2
#define FT_TOUCH_EVENT_POS	3
#define FT_TOUCH_ID_POS		5
#define FT_TOUCH_DOWN		0
#define FT_TOUCH_CONTACT	2

/*register address*/
#define FT_REG_DEV_MODE		0x00
#define FT_DEV_MODE_REG_CAL	0x02
#define FT_REG_ID		0xA3
#define FT_REG_PMODE		0xA5
#define FT_REG_FW_VER		0xA6
#define FT_REG_FW_VENDOR_ID	0xA8
#define FT_REG_POINT_RATE	0x88
#define FT_REG_THGROUP		0x80
#define FT_REG_ECC		0xCC
#define FT_REG_RESET_FW		0x07
#define FT_REG_FW_MAJ_VER	0xB1
#define FT_REG_FW_MIN_VER	0xB2
#define FT_REG_FW_SUB_MIN_VER	0xB3
#define FT5X0X_REG_HW		0xBD

/* power register bits*/
#define FT_PMODE_ACTIVE		0x00
#define FT_PMODE_MONITOR	0x01
#define FT_PMODE_STANDBY	0x02
#define FT_PMODE_HIBERNATE	0x03
#define FT_FACTORYMODE_VALUE	0x40
#define FT_WORKMODE_VALUE	0x00
#define FT_RST_CMD_REG1		0xFC
#define FT_RST_CMD_REG2		0xBC
#define FT_READ_ID_REG		0x90
#define FT_ERASE_APP_REG	0x61
#define FT_ERASE_PANEL_REG	0x63
#define FT_FW_START_REG		0xBF

#define FT_STATUS_NUM_TP_MASK	0x0F

#define FT_VTG_MIN_UV		2600000
#define FT_VTG_MAX_UV		3300000
#define FT_I2C_VTG_MIN_UV	1800000
#define FT_I2C_VTG_MAX_UV	1800000

#define FT_COORDS_ARR_SIZE	4
#define MAX_BUTTONS		4

#define FT_8BIT_SHIFT		8
#define FT_4BIT_SHIFT		4
#define FT_FW_NAME_MAX_LEN	50

#define FT5316_ID		0x0A
#define FT5306I_ID		0x55
#define FT6X06_ID		0x06


#define FT6X36_ID		0x36


#define FT_UPGRADE_AA		0xAA
#define FT_UPGRADE_55		0x55

#define FT_FW_MIN_SIZE		8
#define FT_FW_MAX_SIZE		32768

/* Firmware file is not supporting minor and sub minor so use 0 */
#define FT_FW_FILE_MAJ_VER(x)	((x)->data[(x)->size - 2])
#define FT_FW_FILE_MIN_VER(x)	0
#define FT_FW_FILE_SUB_MIN_VER(x) 0

#define FT_FW_CHECK(x)		\
	(((x)->data[(x)->size - 8] ^ (x)->data[(x)->size - 6]) == 0xFF \
	&& (((x)->data[(x)->size - 7] ^ (x)->data[(x)->size - 5]) == 0xFF \
	&& (((x)->data[(x)->size - 3] ^ (x)->data[(x)->size - 4]) == 0xFF)))

#define FT_MAX_TRIES		5
#define FT_RETRY_DLY		20

#define FT_MAX_WR_BUF		10
#define FT_MAX_RD_BUF		2
#define FT_FW_PKT_LEN		128
#define FT_FW_PKT_META_LEN	6
#define FT_FW_PKT_DLY_MS	20
#define FT_FW_LAST_PKT		0x6ffa
#define FT_EARSE_DLY_MS		100
#define FT_55_AA_DLY_NS		5000

#define FT_UPGRADE_LOOP		30
#define FT_CAL_START		0x04
#define FT_CAL_FIN		0x00
#define FT_CAL_STORE		0x05
#define FT_CAL_RETRY		100
#define FT_REG_CAL		0x00
#define FT_CAL_MASK		0x70

#define FT_INFO_MAX_LEN		512
#if VIRTUAL_KEY
#define MAX_BUF_SIZE	256
#define VKEY_VER_CODE	"0x01"

#define HEIGHT_SCALE_NUM 8
#define HEIGHT_SCALE_DENOM 10

#define VKEY_Y_OFFSET_DEFAULT 0

#define BORDER_ADJUST_NUM 3
#define BORDER_ADJUST_DENOM 4
struct kobject *vkey_kobj;
static char *vkey_buf;
unsigned char hardware_type_tp=0;

#endif

#define FT_BLOADER_SIZE_OFF	12
#define FT_BLOADER_NEW_SIZE	30
#define FT_DATA_LEN_OFF_OLD_FW	8
#define FT_DATA_LEN_OFF_NEW_FW	14
#define FT_FINISHING_PKT_LEN_OLD_FW	6
#define FT_FINISHING_PKT_LEN_NEW_FW	12
#define FT_MAGIC_BLOADER_Z7	0x7bfa
#define FT_MAGIC_BLOADER_LZ4	0x6ffa
#define FT_MAGIC_BLOADER_GZF_30	0x7ff4
#define FT_MAGIC_BLOADER_GZF	0x7bf4

enum {
	FT_BLOADER_VERSION_LZ4 = 0,
	FT_BLOADER_VERSION_Z7 = 1,
	FT_BLOADER_VERSION_GZF = 2,
};

enum {
	FT_FT5336_FAMILY_ID_0x11 = 0x11,
	FT_FT5336_FAMILY_ID_0x12 = 0x12,
	FT_FT5336_FAMILY_ID_0x13 = 0x13,
	FT_FT5336_FAMILY_ID_0x14 = 0x14,
};

#define FT_STORE_TS_INFO(buf, id, name, max_tch, group_id, fw_vkey_support, \
			fw_name, fw_maj, fw_min, fw_sub_min) \
			snprintf(buf, FT_INFO_MAX_LEN, \
				"controller\t= focaltech\n" \
				"model\t\t= 0x%x\n" \
				"name\t\t= %s\n" \
				"max_touches\t= %d\n" \
				"drv_ver\t\t= 0x%x\n" \
				"group_id\t= 0x%x\n" \
				"fw_vkey_support\t= %s\n" \
				"fw_name\t\t= %s\n" \
				"fw_ver\t\t= %d.%d.%d\n", id, name, \
				max_tch, FT_DRIVER_VERSION, group_id, \
				fw_vkey_support, fw_name, fw_maj, fw_min, \
				fw_sub_min)

#define FT_DEBUG_DIR_NAME	"ts_debug"

struct ft6x06_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct ft6x06_ts_platform_data *pdata;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	char fw_name[FT_FW_NAME_MAX_LEN];
	bool loading_fw;
	u8 family_id;
	struct dentry *dir;
	u16 addr;
	bool suspended;
	char *ts_info;
	u8 *tch_data;
	u32 tch_data_len;
	u8 fw_ver[3];
	u8 power_on;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif

#ifdef PINCTRL_FT6336
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
#endif
};
bool FT6xx6DebugMode = false;
static struct workqueue_struct *ft6xx6_workqueue = NULL;

#define FT6XX6_TP_DBG(format, ...) \
{ \
    if(FT6xx6DebugMode) \
    { \
        printk(KERN_INFO "FT6x06 debug " format , ## __VA_ARGS__); \
    } \
}

int fts_esd_running = 0;
#ifdef FT6336_ESD_PROTECT
static spinlock_t irq_lock;
static s32 irq_is_disable = 0;
struct ft6x06_ts_data *g_data = NULL;
static int esd_work_cycle = 2000; 
static struct delayed_work esd_check_work;
static struct mutex esd_lock;
#endif





#if defined CONFIG_TOUCHSCREEN_FT6X06_FIRMWARE
/*[BUGFIX]-Add-BEGIN by TCTNB.XQJ, 2013/12/05, refer to bug 564812 for tp upgrade*/
int TP_VENDOR = 0;
int TP_TYPE=0;
/*[BUGFIX]-Add-BEGIN by TCTNB.XQJ*/
int ft6x06_i2c_read(struct i2c_client *client, char *writebuf,
			   int writelen, char *readbuf, int readlen)
#else
static int ft6x06_i2c_read(struct i2c_client *client, char *writebuf,
			   int writelen, char *readbuf, int readlen)
#endif
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = 0,
				 .len = writelen,
				 .buf = writebuf,
			 },
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}

#if defined CONFIG_TOUCHSCREEN_FT6X06_FIRMWARE
int ft6x06_i2c_write(struct i2c_client *client, char *writebuf,
			    int writelen)
#else
static int ft6x06_i2c_write(struct i2c_client *client, char *writebuf,
			    int writelen)
#endif
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
		 },
	};
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s: i2c write error.\n", __func__);

	return ret;
}

static int ft5x0x_write_reg(struct i2c_client *client, u8 addr, const u8 val)
{
	u8 buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;

	return ft6x06_i2c_write(client, buf, sizeof(buf));
}

static int ft5x0x_read_reg(struct i2c_client *client, u8 addr, u8 *val)
{
	return ft6x06_i2c_read(client, &addr, 1, val, 1);
}


#ifdef FT6336_GESTURE_FUNCTION
static ssize_t dbclick_switch_show ( struct device *dev,
                                      struct device_attribute *attr, char *buf )
{
    return sprintf ( buf, "%d\n", gesture.option);
}
static ssize_t dbclick_switch_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t count )
{
   struct ft6x06_ts_data *data;

	data = (struct ft6x06_ts_data *)i2c_get_clientdata(ft_g_client);
    if (NULL == buf)
        return -EINVAL;
    if (data->suspended) //if in sleep state,return error
          return -EINVAL;
    if (0 == count)
        return 0;
    if( buf[0] == '0')
    {
        gesture.option = 0x00;//dblick_switch=0;
        device_init_wakeup(&ft_g_client->dev, 0);
    }
    else if(buf[0]=='1')
    {
        gesture.option = 0x01;
        device_init_wakeup(&ft_g_client->dev, 1);
    }
    else
    {
        pr_err("invalid  command! \n");
        return -1;
    }
    return count;
}
static DEVICE_ATTR(dbclick, 0664, dbclick_switch_show, dbclick_switch_store);
#endif

static void ft6x06_update_fw_ver(struct ft6x06_ts_data *data)
{
	struct i2c_client *client = data->client;
	u8 reg_addr;
	int err;

	reg_addr = FT_REG_FW_VER;
	err = ft6x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[0], 1);
	if (err < 0)
		dev_err(&client->dev, "fw major version read failed");
 	dev_info(&client->dev, "Firmware version = %d\n", data->fw_ver[0]);
}


static irqreturn_t ft6x06_ts_interrupt(int irq, void *dev_id)
{
	struct ft6x06_ts_data *data = dev_id;
	struct input_dev *ip_dev;
	int rc, i;
	u32 id, x, y, status, num_touches;
	u8 reg = 0x00, *buf;
	bool update_input = false;
#ifdef FT6336_GESTURE_FUNCTION
    u8 reg_value;
    //char txbuf[2];
#endif
	if (!data) {
		pr_err("%s: Invalid data\n", __func__);
		return IRQ_HANDLED;
	}
#ifdef FT6336_GESTURE_FUNCTION
    if((0x01==gesture.option)&&(0x01==gesture.flag))//gesture INT
    {
        //msleep(5);
        reg = FT5X06_REG_GESTURE_STATE;
        rc = ft6x06_i2c_read(data->client, &reg, 1, &reg_value, 1);
        if(GESTURE_DB == reg_value)
        {
            /*gesture.flag=0x00;
            txbuf[0] = FT5X06_REG_GESTURE_SET;
            txbuf[1] = 0x00; //disable gesture
            ft6x06_i2c_write(data->client, txbuf, sizeof(txbuf));
            gesture.flag = 0x00;//clean flag*/
            gesture.stateReg_val=reg_value;
            input_report_key(data->input_dev, KEY_POWER, 1);
            input_sync(data->input_dev);
            input_report_key(data->input_dev, KEY_POWER, 0);
            input_sync(data->input_dev);
            printk(KERN_ERR "%s DBClick report KEY_POWER\n",__func__);
        }
        else
        {
            pr_err("=>error  reg_addr 0x01= 0x%04x\n",reg_value);
        }
         return IRQ_HANDLED;
    }
#endif
	ip_dev = data->input_dev;
	buf = data->tch_data;

	rc = ft6x06_i2c_read(data->client, &reg, 1,
			buf, data->tch_data_len);
	if (rc < 0) {
		dev_err(&data->client->dev, "%s: read data fail\n", __func__);
		return IRQ_HANDLED;
	}

	for (i = 0; i < data->pdata->num_max_touches; i++) {
		id = (buf[FT_TOUCH_ID_POS + FT_ONE_TCH_LEN * i]) >> 4;
		if (id >= FT_MAX_ID)
			break;

		update_input = true;

		x = (buf[FT_TOUCH_X_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) << 8 |
			(buf[FT_TOUCH_X_L_POS + FT_ONE_TCH_LEN * i]);
		y = (buf[FT_TOUCH_Y_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) << 8 |
			(buf[FT_TOUCH_Y_L_POS + FT_ONE_TCH_LEN * i]);

		status = buf[FT_TOUCH_EVENT_POS + FT_ONE_TCH_LEN * i] >> 6;

		num_touches = buf[FT_TD_STATUS] & FT_STATUS_NUM_TP_MASK;

		/* invalid combination */
		if (!num_touches && !status && !id)
			break;

		input_mt_slot(ip_dev, id);
		if (status == FT_TOUCH_DOWN || status == FT_TOUCH_CONTACT) 
		{
            if(y >= 900)
			{
				switch(x)
				{
					case 80:
						input_report_key(ip_dev, KEY_BACK, 1);
						break;
					case 240:
						input_report_key(ip_dev, KEY_HOME, 1);
						break;
					case 400:
						input_report_key(ip_dev, KEY_APP_SWITCH, 1);
						break;
					default:
					    printk(KERN_INFO "%s x out of range",__func__);
					    break;
				}
			}
			else{
			input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 1);
			input_report_abs(ip_dev, ABS_MT_POSITION_X, x);
			input_report_abs(ip_dev, ABS_MT_POSITION_Y, y);
			}
			FT6XX6_TP_DBG("@@@@ x=%d, y=%d\n", x, y);
			//input_report_abs(ip_dev, ABS_MT_PRESSURE, pressure);//delete by rongxiao.deng for touchpanel useless sometimes
		} else {
			input_report_key(ip_dev, KEY_BACK, 0);
			input_report_key(ip_dev, KEY_APP_SWITCH, 0);
			input_report_key(ip_dev, KEY_HOME, 0);
			input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 0);
		}
	}

	if (update_input) {
		input_mt_report_pointer_emulation(ip_dev, false);
		input_sync(ip_dev);
	}

	return IRQ_HANDLED;
}

static int ft6x06_power_on(struct ft6x06_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	if(1 == data->power_on)
		return 0;
	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}
	data->power_on = 1;
	printk("%s: %d\n",__func__,on);
	
	return rc;

power_off:
	if(0 == data->power_on)
		return 0;
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
		}
	}
	data->power_on = 0;
	printk("%s: %d\n",__func__,on);
	return rc;
}

static int ft6x06_power_init(struct ft6x06_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
					   FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FT_I2C_VTG_MIN_UV,
					   FT_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

#ifdef FT6336_ESD_PROTECT
void ft6xx6_esd_switch(s32 on)
{
	mutex_lock(&esd_lock);
	if (true == on) {	/* switch on esd check */
		if (!fts_esd_running) {
			fts_esd_running = 1;
			printk("[FTS] Esd protector started!\n");
			queue_delayed_work(ft6xx6_workqueue, &esd_check_work, esd_work_cycle);
		}
	} else {		/* switch off esd check */
		if (fts_esd_running) {
			fts_esd_running = 0;
			printk("[FTS] Esd protector stoped!\n");
			cancel_delayed_work(&esd_check_work);
		}
	}
	mutex_unlock(&esd_lock);
}

void ft6xx6_irq_enable(void)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&irq_lock, irqflags);
	if (irq_is_disable) {
		enable_irq(ft_g_client->irq);
//#ifdef	GTP_GESTURE_WAKEUP
//		if(1 == gesture_enabled){
//			enable_irq_wake(ft_g_client->irq);
//		}
//#endif
		irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&irq_lock, irqflags);
}

void ft6xx6_irq_disable(void)
{
	unsigned long irqflags;
	spin_lock_irqsave(&irq_lock, irqflags);
	if (!irq_is_disable) {
		irq_is_disable = 1;
		disable_irq_nosync(ft_g_client->irq);
//#ifdef	GTP_GESTURE_WAKEUP
//		if(1 == gesture_enabled){
//			disable_irq_wake(ft_g_client->irq);
//		}
//#endif
	}
	spin_unlock_irqrestore(&irq_lock, irqflags);
}

void ft6xx6_power_reset(void)
{
	static int rst_flag;
	printk("[FTS] force_reset_guitar \n");
	rst_flag = 1;

	ft6xx6_irq_disable();
	ft6x06_power_on(g_data, false);
	msleep(20);
	ft6x06_power_on(g_data, true);
	msleep(20);
	
	if (gpio_is_valid(g_data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(g_data->pdata->reset_gpio, 1);
		msleep(20);
		gpio_set_value_cansleep(g_data->pdata->reset_gpio, 0);
		msleep(20);
		gpio_set_value_cansleep(g_data->pdata->reset_gpio, 1);
	}

	msleep(10);

	ft6xx6_irq_enable();
	rst_flag = 0;
}

static void ft6xx6_esd_check_func(struct work_struct *work)
{
	s32 i = 0;
	s32 ret = -1;
	u8 reg_value;
	u8 reg_addr;

	if (!fts_esd_running) {
		pr_err("Esd protector suspended! \n");
		return;
	}

	for (i = 0; i < 3; i++) {
		reg_addr = 0xA8;
		ret = ft6x06_i2c_read(ft_g_client, &reg_addr, 1, &reg_value, 1);
		if (ret >= 0 && reg_value == 0xe0) {
			break;
		}
		msleep(50);
	}

	if (likely(i < 3)) {
		/* IC works normally, not do anything */
		FT6XX6_TP_DBG("[FTS] IC works normally!\n");
		//printk("[FTS] IC works normally!\n");
	} else {
		if (fts_esd_running) {
			printk("[FTS] IC works abnormally! Process reset guitar.\n");
			msleep(50);
			ft6xx6_power_reset();
		} else {
			printk("[FTS] Esd protector suspended, no need reset!\n");
		}
	}

	mutex_lock(&esd_lock);
	if (fts_esd_running) {
		queue_delayed_work(ft6xx6_workqueue, &esd_check_work, esd_work_cycle);
	} else {
		printk("[FTS] Esd protector suspended!\n");
	}
	mutex_unlock(&esd_lock);
}

void ft6xx6_init_esd_protect(void)
{
	esd_work_cycle = 2 * HZ;	// HZ: clock ticks in 1 second generated by system
	printk("[FTS] Clock ticks for an esd cycle: %d \n", esd_work_cycle);
	INIT_DELAYED_WORK(&esd_check_work, ft6xx6_esd_check_func);
	mutex_init(&esd_lock);
}

#endif

#ifdef PINCTRL_FT6336
static int ft5x06_ts_pinctrl_init(struct ft6x06_ts_data *ft5x06_data)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	ft5x06_data->ts_pinctrl = devm_pinctrl_get(&(ft5x06_data->client->dev));
	if (IS_ERR_OR_NULL(ft5x06_data->ts_pinctrl)) {
		dev_dbg(&ft5x06_data->client->dev,
			"Target does not use pinctrl\n");
		retval = PTR_ERR(ft5x06_data->ts_pinctrl);
		ft5x06_data->ts_pinctrl = NULL;
		return retval;
	}

	ft5x06_data->gpio_state_active
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
			PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(ft5x06_data->gpio_state_active)) {
		dev_dbg(&ft5x06_data->client->dev,
			"Can not get ts default pinstate\n");
		retval = PTR_ERR(ft5x06_data->gpio_state_active);
		ft5x06_data->ts_pinctrl = NULL;
		return retval;
	}

	ft5x06_data->gpio_state_suspend
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
			PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(ft5x06_data->gpio_state_suspend)) {
		dev_err(&ft5x06_data->client->dev,
			"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(ft5x06_data->gpio_state_suspend);
		ft5x06_data->ts_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int ft5x06_ts_pinctrl_select(struct ft6x06_ts_data *ft5x06_data,
						bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ? ft5x06_data->gpio_state_active
		: ft5x06_data->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(ft5x06_data->ts_pinctrl, pins_state);
		if (ret) {
			dev_err(&ft5x06_data->client->dev,
				"can not set %s pins\n",
				on ? PINCTRL_STATE_ACTIVE : PINCTRL_STATE_SUSPEND);
			return ret;
		}
	} else {
		dev_err(&ft5x06_data->client->dev,
			"not a valid '%s' pinstate\n",
				on ? PINCTRL_STATE_ACTIVE : PINCTRL_STATE_SUSPEND);
	}

	return 0;
}
#endif

#ifdef CONFIG_PM
static int ft6x06_ts_suspend(struct device *dev)
{
	struct ft6x06_ts_data *data = dev_get_drvdata(dev);
	char txbuf[2],i;
	int err = 0;
	
	if (data->loading_fw) {
		dev_info(dev, "Firmware loading in process...\n");
		return 0;
	}

	if (data->suspended) {
		dev_info(dev, "Already in suspend state\n");
		return 0;
	}
#ifdef FT6336_GESTURE_FUNCTION
    if(gesture.option==0)
        disable_irq(data->client->irq);//double click is not set,
#else
	disable_irq(data->client->irq);
#endif

#ifdef FT6336_ESD_PROTECT
	ft6xx6_esd_switch(false);
#endif
    
#ifdef FT6336_GESTURE_FUNCTION
    if(gesture.option==1)//double click is set,it need wake up by double click,it need other setting
    {
        //disable_irq(data->client->irq);
        //gesture.flag=0x01;
        txbuf[0] = FT5X06_REG_GESTURE_SET;
        txbuf[1] = 0x01;// enable tp gesture
        ft6x06_i2c_write(data->client, txbuf, sizeof(txbuf));
        gesture.flag=0x01;

        if (device_may_wakeup(dev))
        {
            err=enable_irq_wake(data->client->irq);
        }
        //enable_irq(data->client->irq);
        data->suspended = true;
        return err ;
    }
#endif
	/* release all touches */
	for (i = 0; i < data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	//input_report_key(data->input_dev, BTN_TOUCH, 0);
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);
	
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		txbuf[0] = FT_REG_PMODE;
		txbuf[1] = FT_PMODE_HIBERNATE;
		ft6x06_i2c_write(data->client, txbuf, sizeof(txbuf));
	}
#ifdef PINCTRL_FT6336
		if (data->ts_pinctrl) {
			err = ft5x06_ts_pinctrl_select(data,1);
			if (err < 0) {
				dev_err(dev, "Cannot get idle pinctrl state %d\n",
					err);
				return err;
			}
		}
#endif

	if (data->pdata->power_on) {
		err = data->pdata->power_on(false);
		if (err) {
			dev_err(dev, "power off failed");
			goto pwr_off_fail;
		}
	} else {
		err = ft6x06_power_on(data, false);
		if (err) {
			dev_err(dev, "power off failed");
			goto pwr_off_fail;
		}
	}

	data->suspended = true;
	
	printk("TP enter suspend: %s \n",__func__);
	
	return 0;

pwr_off_fail:
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}
	enable_irq(data->client->irq);
	return err;
}

static int ft6x06_ts_resume(struct device *dev)
{
	struct ft6x06_ts_data *data = dev_get_drvdata(dev);
	int err = 0;
#ifdef FT6336_GESTURE_FUNCTION
    char txbuf[2];
#endif

	if (!data->suspended) {
		dev_dbg(dev, "Already in awake state\n");
		return 0;
	}
#ifdef FT6336_GESTURE_FUNCTION
    if(gesture.option==1)//double click is set,it need wake up by double click,it need other setting
    {
      if (device_may_wakeup(&ft_g_client->dev)) {
            disable_irq_wake(data->client->irq);
        }
        //gesture.flag=0x00;
        txbuf[0] = FT5X06_REG_GESTURE_SET;
        txbuf[1] = 0x00; //disable gesture
        ft6x06_i2c_write(data->client, txbuf, sizeof(txbuf));
        gesture.flag = 0x00;//clean flag
        data->suspended = false;
        msleep(100);
		return 0 ;
    }
#endif
	if (data->pdata->power_on) {
		err = data->pdata->power_on(true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	} else {
		err = ft6x06_power_on(data, true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	}
#ifdef PINCTRL_FT6336
			if (data->ts_pinctrl) {
                err = ft5x06_ts_pinctrl_select(data,0);
				if (err < 0) {
					dev_err(dev, "Cannot get idle pinctrl state %d\n",
						err);
					return err;
				}
			}
#endif
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}

	msleep(data->pdata->soft_rst_dly);

	enable_irq(data->client->irq);
	
#ifdef FT6336_ESD_PROTECT
	ft6xx6_esd_switch(true);
#endif
	
	data->suspended = false;

	printk("TP enter resume: %s\n",__func__);
	
	return 0;
}

static const struct dev_pm_ops ft6x06_ts_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend = ft6x06_ts_suspend,
	.resume = ft6x06_ts_resume,
#endif
};

#else
static int ft6x06_ts_suspend(struct device *dev)
{
	return 0;
}

static int ft5x06_ts_resume(struct device *dev)
{
	return 0;
}

#endif
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct ft6x06_ts_data *ft6x06_data =
		container_of(self, struct ft6x06_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			ft6x06_data && ft6x06_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			ft6x06_ts_resume(&ft6x06_data->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			ft6x06_ts_suspend(&ft6x06_data->client->dev);
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void ft6x06_ts_early_suspend(struct early_suspend *handler)
{
	struct ft6x06_ts_data *data = container_of(handler,
						   struct ft6x06_ts_data,
						   early_suspend);

	ft6x06_ts_suspend(&data->client->dev);
}

static void ft6x06_ts_late_resume(struct early_suspend *handler)
{
	struct ft6x06_ts_data *data = container_of(handler,
						   struct ft6x06_ts_data,
						   early_suspend);

	ft6x06_ts_resume(&data->client->dev);
}
#endif


static int ft6x06_auto_cal(struct i2c_client *client)
{
	struct ft6x06_ts_data *data = i2c_get_clientdata(client);
	u8 temp = 0, i;

	/* set to factory mode */
	msleep(2 * data->pdata->soft_rst_dly);
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_FACTORYMODE_VALUE);
	msleep(data->pdata->soft_rst_dly);

	/* start calibration */
	ft5x0x_write_reg(client, FT_DEV_MODE_REG_CAL, FT_CAL_START);
	msleep(2 * data->pdata->soft_rst_dly);
	for (i = 0; i < FT_CAL_RETRY; i++) {
		ft5x0x_read_reg(client, FT_REG_CAL, &temp);
		/*return to normal mode, calibration finish */
		if (((temp & FT_CAL_MASK) >> FT_4BIT_SHIFT) == FT_CAL_FIN)
			break;
	}

	/*calibration OK */
	msleep(2 * data->pdata->soft_rst_dly);
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_FACTORYMODE_VALUE);
	msleep(data->pdata->soft_rst_dly);

	/* store calibration data */
	ft5x0x_write_reg(client, FT_DEV_MODE_REG_CAL, FT_CAL_STORE);
	msleep(2 * data->pdata->soft_rst_dly);

	/* set to normal mode */
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_WORKMODE_VALUE);
	msleep(2 * data->pdata->soft_rst_dly);

	return 0;
}

static int ft6x06_fw_upgrade_start(struct i2c_client *client,
			const u8 *data, u32 data_len)
{
	struct ft6x06_ts_data *ts_data = i2c_get_clientdata(client);
	struct fw_upgrade_info info = ts_data->pdata->info;
	u8 reset_reg;
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	u8 pkt_buf[FT_FW_PKT_LEN + FT_FW_PKT_META_LEN];
	int i, j, temp;
	u32 pkt_num, pkt_len;
	u8 is_5336_new_bootloader = false;
	u8 is_5336_fwsize_30 = false;
	u8 fw_ecc;

	/* determine firmware size */
	if (*(data + data_len - FT_BLOADER_SIZE_OFF) == FT_BLOADER_NEW_SIZE)
		is_5336_fwsize_30 = true;
	else
		is_5336_fwsize_30 = false;

	for (i = 0, j = 0; i < FT_UPGRADE_LOOP; i++) {
		msleep(FT_EARSE_DLY_MS);
		/* reset - write 0xaa and 0x55 to reset register */
		if (ts_data->family_id == FT6X06_ID)
			reset_reg = FT_RST_CMD_REG2;
		else
			reset_reg = FT_RST_CMD_REG1;

		ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_AA);
		msleep(info.delay_aa);

		ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_55);
		if (i <= (FT_UPGRADE_LOOP / 2))
			msleep(info.delay_55 + i * 3);
		else
			msleep(info.delay_55 - (i - (FT_UPGRADE_LOOP / 2)) * 2);

		/* Enter upgrade mode */
		w_buf[0] = FT_UPGRADE_55;
		ft6x06_i2c_write(client, w_buf, 1);
		usleep(FT_55_AA_DLY_NS);
		w_buf[0] = FT_UPGRADE_AA;
		ft6x06_i2c_write(client, w_buf, 1);

		/* check READ_ID */
		msleep(info.delay_readid);
		w_buf[0] = FT_READ_ID_REG;
		w_buf[1] = 0x00;
		w_buf[2] = 0x00;
		w_buf[3] = 0x00;

		ft6x06_i2c_read(client, w_buf, 4, r_buf, 2);

		if (r_buf[0] != info.upgrade_id_1
			|| r_buf[1] != info.upgrade_id_2) {
			dev_err(&client->dev, "Upgrade ID mismatch(%d), IC=0x%x 0x%x, info=0x%x 0x%x\n",
				i, r_buf[0], r_buf[1],
				info.upgrade_id_1, info.upgrade_id_2);
		} else
			break;
	}

	if (i >= FT_UPGRADE_LOOP) {
		dev_err(&client->dev, "Abort upgrade\n");
		return -EIO;
	}

	w_buf[0] = 0xcd;
	ft6x06_i2c_read(client, w_buf, 1, r_buf, 1);

	if (r_buf[0] <= 4)
		is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;
	else if (r_buf[0] == 7)
		is_5336_new_bootloader = FT_BLOADER_VERSION_Z7;
	else if (r_buf[0] >= 0x0f &&
		((ts_data->family_id == FT_FT5336_FAMILY_ID_0x11) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x12) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x13) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x14)))
		is_5336_new_bootloader = FT_BLOADER_VERSION_GZF;
	else
		is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;

	dev_dbg(&client->dev, "bootloader type=%d, r_buf=0x%x, family_id=0x%x\n",
		is_5336_new_bootloader, r_buf[0], ts_data->family_id);
	/* is_5336_new_bootloader = FT_BLOADER_VERSION_GZF; */

	/* erase app and panel paramenter area */
	w_buf[0] = FT_ERASE_APP_REG;
	ft6x06_i2c_write(client, w_buf, 1);
	msleep(info.delay_erase_flash);

	if (is_5336_fwsize_30) {
		w_buf[0] = FT_ERASE_PANEL_REG;
		ft6x06_i2c_write(client, w_buf, 1);
	}
	msleep(FT_EARSE_DLY_MS);

	/* program firmware */
	if (is_5336_new_bootloader == FT_BLOADER_VERSION_LZ4
		|| is_5336_new_bootloader == FT_BLOADER_VERSION_Z7)
		data_len = data_len - FT_DATA_LEN_OFF_OLD_FW;
	else
		data_len = data_len - FT_DATA_LEN_OFF_NEW_FW;

	pkt_num = (data_len) / FT_FW_PKT_LEN;
	pkt_len = FT_FW_PKT_LEN;
	pkt_buf[0] = FT_FW_START_REG;
	pkt_buf[1] = 0x00;
	fw_ecc = 0;

	for (i = 0; i < pkt_num; i++) {
		temp = i * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		pkt_buf[4] = (u8) (pkt_len >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) pkt_len;

		for (j = 0; j < FT_FW_PKT_LEN; j++) {
			pkt_buf[6 + j] = data[i * FT_FW_PKT_LEN + j];
			fw_ecc ^= pkt_buf[6 + j];
		}

		ft6x06_i2c_write(client, pkt_buf,
				FT_FW_PKT_LEN + FT_FW_PKT_META_LEN);
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* send remaining bytes */
	if ((data_len) % FT_FW_PKT_LEN > 0) {
		temp = pkt_num * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		temp = (data_len) % FT_FW_PKT_LEN;
		pkt_buf[4] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			pkt_buf[6 + i] = data[pkt_num * FT_FW_PKT_LEN + i];
			fw_ecc ^= pkt_buf[6 + i];
		}

		ft6x06_i2c_write(client, pkt_buf, temp + FT_FW_PKT_META_LEN);
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* send the finishing packet */
	if (is_5336_new_bootloader == FT_BLOADER_VERSION_LZ4 ||
		is_5336_new_bootloader == FT_BLOADER_VERSION_Z7) {
		for (i = 0; i < FT_FINISHING_PKT_LEN_OLD_FW; i++) {
			if (is_5336_new_bootloader  == FT_BLOADER_VERSION_Z7)
				temp = FT_MAGIC_BLOADER_Z7 + i;
			else if (is_5336_new_bootloader ==
						FT_BLOADER_VERSION_LZ4)
				temp = FT_MAGIC_BLOADER_LZ4 + i;
			pkt_buf[2] = (u8)(temp >> 8);
			pkt_buf[3] = (u8)temp;
			temp = 1;
			pkt_buf[4] = (u8)(temp >> 8);
			pkt_buf[5] = (u8)temp;
			pkt_buf[6] = data[data_len + i];
			fw_ecc ^= pkt_buf[6];

			ft6x06_i2c_write(client,
				pkt_buf, temp + FT_FW_PKT_META_LEN);
			msleep(FT_FW_PKT_DLY_MS);
		}
	} else if (is_5336_new_bootloader == FT_BLOADER_VERSION_GZF) {
		for (i = 0; i < FT_FINISHING_PKT_LEN_NEW_FW; i++) {
			if (is_5336_fwsize_30)
				temp = FT_MAGIC_BLOADER_GZF_30 + i;
			else
				temp = FT_MAGIC_BLOADER_GZF + i;
			pkt_buf[2] = (u8)(temp >> 8);
			pkt_buf[3] = (u8)temp;
			temp = 1;
			pkt_buf[4] = (u8)(temp >> 8);
			pkt_buf[5] = (u8)temp;
			pkt_buf[6] = data[data_len + i];
			fw_ecc ^= pkt_buf[6];

			ft6x06_i2c_write(client,
				pkt_buf, temp + FT_FW_PKT_META_LEN);
			msleep(FT_FW_PKT_DLY_MS);

		}
	}

	/* verify checksum */
	w_buf[0] = FT_REG_ECC;
	ft6x06_i2c_read(client, w_buf, 1, r_buf, 1);
	if (r_buf[0] != fw_ecc) {
		dev_err(&client->dev, "ECC error! dev_ecc=%02x fw_ecc=%02x\n",
					r_buf[0], fw_ecc);
		return -EIO;
	}

	/* reset */
	w_buf[0] = FT_REG_RESET_FW;
	ft6x06_i2c_write(client, w_buf, 1);
	msleep(ts_data->pdata->soft_rst_dly);

	dev_info(&client->dev, "Firmware upgrade successful\n");

	return 0;
}

static int ft6x06_fw_upgrade(struct device *dev, bool force)
{
	struct ft6x06_ts_data *data = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	int rc;
	u8 fw_file_maj, fw_file_min, fw_file_sub_min;
	bool fw_upgrade = false;

	if (data->suspended) {
		dev_err(dev, "Device is in suspend state: Exit FW upgrade\n");
		return -EBUSY;
	}
	rc = request_firmware(&fw, data->fw_name, dev);
	if (rc < 0) {
		dev_err(dev, "Request firmware failed - %s (%d)\n",
						data->fw_name, rc);
		return rc;
	}

	if (fw->size < FT_FW_MIN_SIZE || fw->size > FT_FW_MAX_SIZE) {
		dev_err(dev, "Invalid firmware size (%d)\n", fw->size);
		rc = -EIO;
		goto rel_fw;
	}

	fw_file_maj = FT_FW_FILE_MAJ_VER(fw);
	fw_file_min = FT_FW_FILE_MIN_VER(fw);
	fw_file_sub_min = FT_FW_FILE_SUB_MIN_VER(fw);

	dev_info(dev, "Current firmware: %d.%d.%d", data->fw_ver[0],
				data->fw_ver[1], data->fw_ver[2]);
	dev_info(dev, "New firmware: %d.%d.%d", fw_file_maj,
				fw_file_min, fw_file_sub_min);

	if (force) {
		fw_upgrade = true;
	} else if (data->fw_ver[0] == fw_file_maj) {
			if (data->fw_ver[1] < fw_file_min)
				fw_upgrade = true;
			else if (data->fw_ver[2] < fw_file_sub_min)
				fw_upgrade = true;
			else
				dev_info(dev, "No need to upgrade\n");
	} else
		dev_info(dev, "Firmware versions do not match\n");

	if (!fw_upgrade) {
		dev_info(dev, "Exiting fw upgrade...\n");
		rc = -EFAULT;
		goto rel_fw;
	}

	/* start firmware upgrade */
	if (FT_FW_CHECK(fw)) {
		rc = ft6x06_fw_upgrade_start(data->client, fw->data, fw->size);
		if (rc < 0)
			dev_err(dev, "update failed (%d). try later...\n", rc);
		else if (data->pdata->info.auto_cal)
			ft6x06_auto_cal(data->client);
	} else {
		dev_err(dev, "FW format error\n");
		rc = -EIO;
	}

	ft6x06_update_fw_ver(data);

	FT_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
			data->pdata->num_max_touches, data->pdata->group_id,
			data->pdata->fw_vkey_support ? "yes" : "no",
			data->pdata->fw_name, data->fw_ver[0],
			data->fw_ver[1], data->fw_ver[2]);
rel_fw:
	release_firmware(fw);
	return rc;
}

static ssize_t ft6x06_update_fw_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft6x06_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, 2, "%d\n", data->loading_fw);
}

static ssize_t ft6x06_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft6x06_ts_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	if (data->suspended) {
		dev_info(dev, "In suspend state, try again later...\n");
		return size;
	}

	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw  && val) {
		data->loading_fw = true;
		ft6x06_fw_upgrade(dev, false);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(update_fw, 0664, ft6x06_update_fw_show,
				ft6x06_update_fw_store);

static ssize_t ft6x06_force_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft6x06_ts_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw  && val) {
		data->loading_fw = true;
		ft6x06_fw_upgrade(dev, true);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(force_update_fw, 0664, ft6x06_update_fw_show,
				ft6x06_force_update_fw_store);

static ssize_t ft6x06_fw_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft6x06_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, FT_FW_NAME_MAX_LEN - 1, "%s\n", data->fw_name);
}

static ssize_t ft6x06_fw_name_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft6x06_ts_data *data = dev_get_drvdata(dev);

	if (size > FT_FW_NAME_MAX_LEN - 1)
		return -EINVAL;

	strlcpy(data->fw_name, buf, size);
	if (data->fw_name[size-1] == '\n')
		data->fw_name[size-1] = 0;

	return size;
}

static DEVICE_ATTR(fw_name, 0664, ft6x06_fw_name_show, ft6x06_fw_name_store);
/*Begin--- Add the the attribute for reading the version of firmware by qifu.cheng ,2014/1/6 */
static ssize_t ft6x06_fw_ver_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft6x06_ts_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	u8 fwver = 0;
	int err;

	err =ft6x06_read_reg(client, FT6x06_REG_FW_VER, &fwver);
	if (err < 0)
		dev_err(&client->dev, "fw major version read failed");
	
	dev_info(&client->dev, "Firmware version = 0x%x\n", fwver);
	return sprintf(buf, "0x%x\n",  fwver);

}

static ssize_t ft6x06_fw_ver_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft6x06_ts_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	dev_info(&client->dev, "Firmware version = %d\n", data->fw_ver[0]);
	return size;
}

static DEVICE_ATTR(focaltech_ver, 0664, ft6x06_fw_ver_show, ft6x06_fw_ver_store);
/*End--- Add the the attribute for reading the version of firmware by qifu.cheng ,2014/1/6 */
static bool ft6x06_debug_addr_is_valid(int addr)
{
	if (addr < 0 || addr > 0xFF) {
		pr_err("FT reg address is invalid: 0x%x\n", addr);
		return false;
	}

	return true;
}

static int ft6x06_debug_data_set(void *_data, u64 val)
{
	struct ft6x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (ft6x06_debug_addr_is_valid(data->addr))
		dev_info(&data->client->dev,
			"Writing into FT registers not supported\n");

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int ft6x06_debug_data_get(void *_data, u64 *val)
{
	struct ft6x06_ts_data *data = _data;
	int rc;
	u8 reg;

	mutex_lock(&data->input_dev->mutex);

	if (ft6x06_debug_addr_is_valid(data->addr)) {
		rc = ft5x0x_read_reg(data->client, data->addr, &reg);
		if (rc < 0)
			dev_err(&data->client->dev,
				"FT read register 0x%x failed (%d)\n",
				data->addr, rc);
		else
			*val = reg;
	}

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_data_fops, ft6x06_debug_data_get,
			ft6x06_debug_data_set, "0x%02llX\n");

static int ft6x06_debug_addr_set(void *_data, u64 val)
{
	struct ft6x06_ts_data *data = _data;

	if (ft6x06_debug_addr_is_valid(val)) {
		mutex_lock(&data->input_dev->mutex);
		data->addr = val;
		mutex_unlock(&data->input_dev->mutex);
	}

	return 0;
}

static int ft6x06_debug_addr_get(void *_data, u64 *val)
{
	struct ft6x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (ft6x06_debug_addr_is_valid(data->addr))
		*val = data->addr;

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_addr_fops, ft6x06_debug_addr_get,
			ft6x06_debug_addr_set, "0x%02llX\n");

static int ft6x06_debug_suspend_set(void *_data, u64 val)
{
	struct ft6x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (val)
		ft6x06_ts_suspend(&data->client->dev);
	else
		ft6x06_ts_resume(&data->client->dev);

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int ft6x06_debug_suspend_get(void *_data, u64 *val)
{
	struct ft6x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);
	*val = data->suspended;
	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_suspend_fops, ft6x06_debug_suspend_get,
			ft6x06_debug_suspend_set, "%lld\n");

static int ft6x06_debug_dump_info(struct seq_file *m, void *v)
{
	struct ft6x06_ts_data *data = m->private;

	seq_printf(m, "%s\n", data->ts_info);

	return 0;
}

static int debugfs_dump_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, ft6x06_debug_dump_info, inode->i_private);
}

static const struct file_operations debug_dump_info_fops = {
	.owner		= THIS_MODULE,
	.open		= debugfs_dump_info_open,
	.read		= seq_read,
	.release	= single_release,
};

#ifdef CONFIG_OF

static int ft6x06_get_dt_coords(struct device *dev, char *name,
				struct ft6x06_ts_platform_data *pdata)
{
	u32 coords[FT_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FT_COORDS_ARR_SIZE) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "focaltech,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "focaltech,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}


static int ft6x06_parse_dt(struct device *dev,
			struct ft6x06_ts_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	u32 button_map[MAX_BUTTONS];

	pdata->name = "focaltech";
	rc = of_property_read_string(np, "focaltech,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}
    

	rc = ft6x06_get_dt_coords(dev, "focaltech,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = ft6x06_get_dt_coords(dev, "focaltech,display-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	pdata->i2c_pull_up = of_property_read_bool(np,
						"focaltech,i2c-pull-up");

	pdata->no_force_update = of_property_read_bool(np,
						"focaltech,no-force-update");
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
				0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
				0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;

	pdata->fw_name = "ft_fw.bin";
	rc = of_property_read_string(np, "focaltech,fw-name", &pdata->fw_name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw name\n");
		return rc;
	}

	rc = of_property_read_u32(np, "focaltech,group-id", &temp_val);
	if (!rc)
		pdata->group_id = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,hard-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->hard_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,soft-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->soft_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,num-max-touches", &temp_val);
	if (!rc)
		pdata->num_max_touches = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,fw-delay-aa-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay aa\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_aa =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-55-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay 55\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_55 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id1", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id1\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_1 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id2", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id2\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_2 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-readid-ms",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay read id\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_readid =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-era-flsh-ms",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay erase flash\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_erase_flash =  temp_val;

	pdata->info.auto_cal = of_property_read_bool(np,
					"focaltech,fw-auto-cal");

	pdata->fw_vkey_support = of_property_read_bool(np,
						"focaltech,fw-vkey-support");

	pdata->ignore_id_check = of_property_read_bool(np,
						"focaltech,ignore-id-check");

	rc = of_property_read_u32(np, "focaltech,family-id", &temp_val);
	if (!rc)
		pdata->family_id = temp_val;
	else
		return rc;

	prop = of_find_property(np, "focaltech,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
			"focaltech,button-map", button_map,
			num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
	}

	return 0;
}
#else
static int ft6x06_parse_dt(struct device *dev,
			struct ft6x06_ts_platform_data *pdata)
{
	return -ENODEV;
}
#endif

#if VIRTUAL_KEY
static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    /*

	return sprintf(buf,
	   __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":45:540:70:60"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":155:540:70:60"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)  ":270:540:80:60"
	   "\n");*/
	   	strlcpy(buf, vkey_buf, MAX_BUF_SIZE);
	return strnlen(buf, MAX_BUF_SIZE);

}

static struct kobj_attribute virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.ft5x06_ts",	// X WARNING: please keep it sync with *input device* name
		.mode = S_IRUGO,
	},
	.show = &virtual_keys_show,
};

static struct attribute *virtual_key_properties_attrs[] = {
	&virtual_keys_attr.attr,
	NULL
};

static struct attribute_group vkey_group = {
	.attrs = virtual_key_properties_attrs,
};
static int vkey_parse_dt(struct device *dev,struct ft6x06_ts_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	struct property *prop;
	int width, height, center_x, center_y;
	int x1 = 0, x2 = 0, i, c = 0,  border;
	int rc, val;
/*Ricahrd add the second and the flag to detect which one should be used*/

/*
	rc = of_property_read_string(np, "label1", &pdata->name);
	if (rc) {
		dev_err(dev, "Failed to read label\n");
		return -EINVAL;
	}
*/

/*	rc = of_property_read_u32(np, "focaltech,disp-maxx", &pdata->disp_maxx);
	if (rc) {
		dev_err(dev, "Failed to read display max x\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(np, "focaltech,disp-maxy", &pdata->disp_maxy);
	if (rc) {
		dev_err(dev, "Failed to read display max y\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(np, "focaltech,pan-maxx", &pdata->pan_maxx);
	if (rc) {
		dev_err(dev, "Failed to read panel max x\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(np, "focaltech,pan-maxy", &pdata->pan_maxy);
	if (rc) {
		dev_err(dev, "Failed to read panel max y\n");
		return -EINVAL;
	}
	*/
	
	prop = of_find_property(np, "focaltech,key-codes", NULL);
	if (prop) {
		pdata->num_keys = prop->length / sizeof(u32);
		pdata->keycodes = devm_kzalloc(dev,
			sizeof(u32) * pdata->num_keys, GFP_KERNEL);
		if (!pdata->keycodes)
			return -ENOMEM;
		rc = of_property_read_u32_array(np, "focaltech,key-codes",
				pdata->keycodes, pdata->num_keys);
		if (rc) {
			dev_err(dev, "Failed to read key codes\n");
			return -EINVAL;
		}
	}

	pdata->y_offset = VKEY_Y_OFFSET_DEFAULT;
	rc = of_property_read_u32(np, "focaltech,y-offset", &val);
	if (!rc)
		pdata->y_offset = val;
	else if (rc != -EINVAL) {
		dev_err(dev, "Failed to read y position offset\n");
		return rc;
	}

	vkey_buf = devm_kzalloc(dev, MAX_BUF_SIZE, GFP_KERNEL);
	if (!vkey_buf) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}
	
	border = (pdata->panel_maxx - pdata->disp_maxx) * 2;
	width = ((pdata->x_max - (border * (pdata->num_keys - 1)))
			/ pdata->num_keys);
	height = (pdata->panel_maxy - pdata->disp_maxy);
	center_y = pdata->y_max + (height / 2) + pdata->y_offset;
	height = height * HEIGHT_SCALE_NUM / HEIGHT_SCALE_DENOM;

	x2 -= border * BORDER_ADJUST_NUM / BORDER_ADJUST_DENOM;

	for (i = 0; i < pdata->num_keys; i++) {
		x1 = x2 + border;
		x2 = x2 + border + width;
		center_x = x1 + (x2 - x1) / 2;
		c += snprintf(vkey_buf + c, MAX_BUF_SIZE - c,
				"%s:%d:%d:%d:%d:%d\n",
				VKEY_VER_CODE, pdata->keycodes[i],
				center_x, center_y, width, height);
	}

	vkey_buf[c] = '\0';

	return 0;
}
static int vkeys_init(struct kobject *kobj,const struct attribute_group *grp,bool on)
{
	int err;
		if(!on)
			goto release_key;
		
		vkey_kobj = kobject_create_and_add("board_properties", NULL);
		if (vkey_kobj) {
			err = sysfs_create_group(vkey_kobj,grp);//&vkey_group
		}
		if (!vkey_kobj || err) {
			printk(KERN_INFO "failed to create board_properties\n");
			kobject_put(vkey_kobj);
		}
		return 0;
release_key:
	sysfs_remove_group(vkey_kobj, grp);//&vkey_group
	kobject_put(vkey_kobj);
	return 1;
}
#endif 
/*
void select_params(struct i2c_client *client,struct ft6x06_ts_platform_data *pdata)
{
    //choose the panel-coords
	pdata->panel_minx = coords[0];
	pdata->panel_miny = coords[1];
	pdata->panel_maxx = coords[2];
	pdata->panel_maxy = coords[3];
    //choosec the display-coords
	pdata->x_min = coords[0];
	pdata->y_min = coords[1];
	pdata->x_max = coords[2];
	pdata->y_max = coords[3];
		
}
*/

static int ft6x06_I2c_Test(struct i2c_client *client)
{
	int rc;
	char temp;
	struct i2c_msg msgs[] =
	{
		{
		.addr = client->addr,//client->addr,
		.flags = 0,//I2C_M_RD,
		.len = 1,
		.buf = &temp,
		},
	};
	printk(" ft6x06_I2c_Test --client addr : %x ***\n", client->addr);

	rc = i2c_transfer(client->adapter, msgs, 1);
	if( rc < 0 )
	{
		printk("*** ft6x06_I2c_Test error %d ***\n", rc);
		return rc;
	}
	
	msleep(10);
	return 0;
}

static int ft6x06_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ft6x06_ts_platform_data *pdata;
	struct ft6x06_ts_data *data;
	struct input_dev *input_dev;
	struct dentry *temp;
	u8 reg_value;
	u8 reg_addr;
	int err, len/*, ret*/;
	printk(KERN_ERR "%s\n",__func__);
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct ft6x06_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = ft6x06_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "DT parsing failed\n");
			return err;
		}
        
	#if VIRTUAL_KEY	
		err = vkey_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "DT parsing failed\n");
		return err;
		}
	#endif
	} else
		pdata = client->dev.platform_data;
    
	if (!pdata) {
		dev_err(&client->dev, "Invalid pdata\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C not supported\n");
		return -ENODEV;
	}
	
	data = devm_kzalloc(&client->dev,
			sizeof(struct ft6x06_ts_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}
   
	if (pdata->fw_name) {
		len = strlen(pdata->fw_name);
		if (len > FT_FW_NAME_MAX_LEN - 1) {
			dev_err(&client->dev, "Invalid firmware name\n");
			return -EINVAL;
		}

		strlcpy(data->fw_name, pdata->fw_name, len + 1);
	}

	data->tch_data_len = FT_TCH_LEN(pdata->num_max_touches);
	data->tch_data = devm_kzalloc(&client->dev,
				data->tch_data_len, GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}
	spin_lock_init(&irq_lock);
	data->input_dev = input_dev;
	data->client = client;
	data->pdata = pdata;

	input_dev->name = "ft5x06_ts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	__set_bit(KEY_BACK, input_dev->keybit);
	__set_bit(KEY_HOME, input_dev->keybit);
	__set_bit(KEY_APP_SWITCH, input_dev->keybit);

	input_mt_init_slots(input_dev, pdata->num_max_touches,0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min,
			     pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min,
			     pdata->y_max, 0, 0);
	//input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, FT_PRESS, 0, 0);//delete by rongxiao.deng for touchpanel useless sometimes

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, "Input device registration failed\n");
		goto free_inputdev;
	}

#if VIRTUAL_KEY
 err =  vkeys_init(vkey_kobj, &vkey_group,true);
 if(err)
 	{
 		dev_err(&client->dev, "The Virtual_key can't be created!\n");
		vkeys_init(vkey_kobj, &vkey_group,false);
 	}
#endif
	if (pdata->power_init) {
		err = pdata->power_init(true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			goto unreg_inputdev;
		}
	} else {
		err = ft6x06_power_init(data, true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			goto unreg_inputdev;
		}
	}
    
    
	data->power_on = 0;
	if (pdata->power_on) {
		err = pdata->power_on(true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	} else {
		err = ft6x06_power_on(data, true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	}

	err = ft6x06_I2c_Test(client);
	if(err < 0)
	{
		printk("ft6x06_I2c_Test fail!\n");
		goto pwr_deinit;
	}
	
#ifdef PINCTRL_FT6336
	err = ft5x06_ts_pinctrl_init(data);
	if (!err && data->ts_pinctrl) {
		err = ft5x06_ts_pinctrl_select(data, false);
		if (err < 0)
			goto pwr_off;
	}
#endif
	if (gpio_is_valid(pdata->irq_gpio)) {
		err = gpio_request(pdata->irq_gpio, "ft6x06_irq_gpio");
		if (err) {
			dev_err(&client->dev, "irq gpio request failed");
			goto pwr_off;
		}
		err = gpio_direction_input(pdata->irq_gpio);
		if (err) {
			dev_err(&client->dev,
				"set_direction for irq gpio failed\n");
			goto free_irq_gpio;
		}
	}

	if (gpio_is_valid(pdata->reset_gpio)) {
		err = gpio_request(pdata->reset_gpio, "ft6x06_reset_gpio");
		if (err) {
			dev_err(&client->dev, "reset gpio request failed");
			goto free_irq_gpio;
		}

		err = gpio_direction_output(pdata->reset_gpio, 0);
		if (err) {
			dev_err(&client->dev,
				"set_direction for reset gpio failed\n");
			goto free_reset_gpio;
		}
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}
	
	// reset tp
	gpio_set_value(pdata->reset_gpio, 1);
	gpio_set_value(pdata->reset_gpio, 0);
	msleep(200);  /* Note that the RST must be in LOW 10ms at least */
	gpio_set_value(pdata->reset_gpio, 1);
	/* Enable the interrupt service thread/routine for INT after 50ms */
	msleep(50);
        
	/* make sure CTP already finish startup process */
	msleep(data->pdata->soft_rst_dly);
//judge the project and IC to select params begin .
    //select_params(client);
//judge the project and IC to select params end .
	reg_addr = 0xA8;
	err = ft6x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0) {
	    dev_err(&client->dev, "TP FW version read failed");
	    goto free_reset_gpio;
	}
	dev_info(&client->dev, "[FTS] bootloader vendor ID=0x%x\n", reg_value);

	reg_addr = FT_REG_POINT_RATE;
	ft6x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "report rate read failed");

	dev_info(&client->dev, "report rate = %dHz\n", reg_value * 10);

	/* check the controller id */
	reg_addr = FT5X0X_REG_HW;
	err = ft6x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0) {
		dev_err(&client->dev, "HW read failed");
		goto free_reset_gpio;
	}
//add by junfeng.zhou for update_project begin .
#ifdef UPGRADE_FIRMWARE
//decide need upgrade_firmware
	err = fts_ctpm_auto_upgrade(client);


#endif
#ifdef SYSFS_DEBUG
	ft6x06_create_sysfs(client);
#endif

#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(client) < 0)
		dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",
				__func__);
#endif

#ifdef FTS_APK_DEBUG
	ft6x06_create_apk_debug_channel(client);
#endif    

    //add by junfeng.zhou for update_project end .

	err = request_threaded_irq(client->irq, NULL,
				ft6x06_ts_interrupt,
				pdata->irqflags | IRQF_ONESHOT,
				client->dev.driver->name, data);
	if (err) {
		dev_err(&client->dev, "request irq failed\n");
		goto free_reset_gpio;
	}

/*
	err = ft5x0x_create_sysfs(client);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto irq_free;
	}       
*/
	err = device_create_file(&client->dev, &dev_attr_fw_name);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto irq_free;
	}
/* Add the the attribute for reading the version of firmware by qifu.cheng ,2014/1/6 */	
	err = device_create_file(&client->dev, &dev_attr_focaltech_ver);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_focaltech_ver_sys;
	}
	
	err = device_create_file(&client->dev, &dev_attr_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_fw_name_sys;
	}

	err = device_create_file(&client->dev, &dev_attr_force_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_update_fw_sys;
	}

	data->dir = debugfs_create_dir(FT_DEBUG_DIR_NAME, NULL);
	if (data->dir == NULL || IS_ERR(data->dir)) {
		pr_err("debugfs_create_dir failed(%ld)\n", PTR_ERR(data->dir));
		err = PTR_ERR(data->dir);
		goto free_force_update_fw_sys;
	}

	temp = debugfs_create_file("addr", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_addr_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("data", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_data_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("suspend", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_suspend_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("dump_info", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_dump_info_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	data->ts_info = devm_kzalloc(&client->dev,
				FT_INFO_MAX_LEN, GFP_KERNEL);
	if (!data->ts_info) {
		dev_err(&client->dev, "Not enough memory\n");
		goto free_debug_dir;
	}
	
	ft6xx6_workqueue = create_singlethread_workqueue("ft6xx6_workthread");
	if (ft6xx6_workqueue == NULL) {
		dev_err(&client->dev,"Create workqueue failed!");
	}
	
	ft_g_client = client;	
	
#ifdef FT6336_GESTURE_FUNCTION
    if(device_create_file(&(client->dev), &dev_attr_dbclick) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_dbclick.attr.name);
   // ft_g_client = client;
    input_set_capability(data->input_dev, EV_KEY, KEY_POWER);

#endif

#ifdef FT6336_ESD_PROTECT
	g_data = data;
	ft6xx6_init_esd_protect();
	ft6xx6_esd_switch(true);
#endif

	/*get some register information */

	FT_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
			data->pdata->num_max_touches, data->pdata->group_id,
			data->pdata->fw_vkey_support ? "yes" : "no",
			data->pdata->fw_name, data->fw_ver[0],
			data->fw_ver[1], data->fw_ver[2]);

#if defined(CONFIG_FB)
	data->fb_notif.notifier_call = fb_notifier_callback;

	err = fb_register_client(&data->fb_notif);

	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
			err);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						    FT_SUSPEND_LEVEL;
	data->early_suspend.suspend = ft6x06_ts_early_suspend;
	data->early_suspend.resume = ft6x06_ts_late_resume;
	register_early_suspend(&data->early_suspend);
#endif
    printk(KERN_ERR "%s probe ok!\n",__func__);
	return 0;

free_debug_dir:
	debugfs_remove_recursive(data->dir);
free_force_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_force_update_fw);
free_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_update_fw);
free_fw_name_sys:
	device_remove_file(&client->dev, &dev_attr_fw_name);
/* Add the the attribute for reading the version of firmware by qifu.cheng ,2014/1/6 */	
free_focaltech_ver_sys:
		device_remove_file(&client->dev, &dev_attr_focaltech_ver);
irq_free:
	free_irq(client->irq, data);
free_reset_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
#ifdef PINCTRL_FT6336
	if (data->ts_pinctrl) {
		err = ft5x06_ts_pinctrl_select(data, true);
		if (err < 0)
			pr_err("Cannot get idle pinctrl state\n");
	}

#endif
free_irq_gpio:
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
#ifdef PINCTRL_FT6336
	if (data->ts_pinctrl) {
		err = ft5x06_ts_pinctrl_select(data, true);
		if (err < 0)
			pr_err("Cannot get idle pinctrl state\n");
	}
#endif
pwr_off:
	if (pdata->power_on)
		pdata->power_on(false);
	else
		ft6x06_power_on(data, false);
pwr_deinit:
	if (pdata->power_init)
		pdata->power_init(false);
	else
		ft6x06_power_init(data, false);
unreg_inputdev:
	input_unregister_device(input_dev);
	input_dev = NULL;
free_inputdev:
	input_free_device(input_dev);
	printk(KERN_INFO "%s probe failed!\n",__func__);
	return err;
}

static int ft6x06_ts_remove(struct i2c_client *client)
{
	struct ft6x06_ts_data *data = i2c_get_clientdata(client);
#ifdef PINCTRL_FT6336
	int retval;
#endif

	debugfs_remove_recursive(data->dir);
	device_remove_file(&client->dev, &dev_attr_force_update_fw);
	device_remove_file(&client->dev, &dev_attr_update_fw);
	device_remove_file(&client->dev, &dev_attr_fw_name);

#if defined(CONFIG_FB)
	if (fb_unregister_client(&data->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif
	free_irq(client->irq, data);

	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);

	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);
#ifdef PINCTRL_FT6336
	if (data->ts_pinctrl) {
		retval = ft5x06_ts_pinctrl_select(data, true);
		if (retval < 0)
			pr_err("Cannot get idle pinctrl state\n");
	}
#endif
	if (data->pdata->power_on)
		data->pdata->power_on(false);
	else
		ft6x06_power_on(data, false);

	if (data->pdata->power_init)
		data->pdata->power_init(false);
	else
		ft6x06_power_init(data, false);
#ifdef FT6336_ESD_PROTECT
	g_data = NULL;
#endif

	ft_g_client = NULL;

	input_unregister_device(data->input_dev);

	return 0;
}

static const struct i2c_device_id ft6x06_ts_id[] = {
	{"ft5x06_ts", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ft6x06_ts_id);

#ifdef CONFIG_OF
static struct of_device_id ft6x06_match_table[] = {
	{ .compatible = "focaltech,ft6x06",},
	{ },
};
#else
#define ft6x06_match_table NULL
#endif

static struct i2c_driver ft6x06_ts_driver = {
	.probe = ft6x06_ts_probe,
	.remove = ft6x06_ts_remove,
	.driver = {
		   .name = "ft6x06_ts",
		   .owner = THIS_MODULE,
		.of_match_table = ft6x06_match_table,
#ifdef CONFIG_PM
		   .pm = &ft6x06_ts_pm_ops,
#endif
		   },
	.id_table = ft6x06_ts_id,
};

static int __init ft6x06_ts_init(void)
{
	return i2c_add_driver(&ft6x06_ts_driver);
}
module_init(ft6x06_ts_init);

static void __exit ft6x06_ts_exit(void)
{
	i2c_del_driver(&ft6x06_ts_driver);
}
module_exit(ft6x06_ts_exit);

MODULE_DESCRIPTION("FocalTech ft6x06 TouchScreen driver");
MODULE_LICENSE("GPL v2");

