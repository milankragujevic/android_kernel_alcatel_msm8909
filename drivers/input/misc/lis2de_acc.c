/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
 *
 * File Name          : lis2de_acc.c
 * Authors            : AMS - Motion Mems Division - Application Team
 *		      : Matteo Dameno (matteo.dameno@st.com)
 *		      : Denis Ciocca (denis.ciocca@st.com)
 *		      : Both authors are willing to be considered the contact
 *		      : and update points for the driver.
 * Version            : V.1.0.2
 * Date               : 2012/Oct/15
 * Description        : LIS2DE accelerometer sensor API
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 ******************************************************************************
 Revision 1.0.1: 2012/Oct/12
  first revision
 Revision 1.0.2: 2012/Oct/12
  better get_acceleration_data version. Modified _acc_remove function
 ******************************************************************************/

#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/delay.h>
#include	<linux/fs.h>
#include	<linux/i2c.h>
#include	<linux/input.h>
#include	<linux/uaccess.h>
#include	<linux/workqueue.h>
#include	<linux/irq.h>
#include	<linux/gpio.h>
#include	<linux/interrupt.h>
#include	<linux/slab.h>
#include	<linux/kernel.h>
#include	<linux/device.h>
#include	<linux/module.h>
#include	<linux/moduleparam.h>
#include 	<linux/sensors.h>

#include	"lis2de.h"
#include 	"bstclass.h"

//#define	DEBUG		1
#define SELF_TEST_NODE		1
#ifdef SELF_TEST_NODE
/* Self-test output change */
#define MIN_X	50
#define MIN_Y 	50
#define MIN_Z 	50
#define MAX_X	1800
#define MAX_Y	1800
#define MAX_Z	1800
#endif
#define LIS2DE12
#define POWER_REGULATOR
#ifdef POWER_REGULATOR
#include <linux/regulator/consumer.h>
/* POWER SUPPLY VOLTAGE RANGE */
#define LIS2DE_VDD_MIN_UV	2000000
#define LIS2DE_VDD_MAX_UV	3300000
#define LIS2DE_VIO_MIN_UV	1750000
#define LIS2DE_VIO_MAX_UV	1950000
#endif

#define	G_MAX		16000


/* Accelerometer Sensor Operating Mode */
#define LIS2DE_ACC_ENABLE	(0x01)
#define LIS2DE_ACC_DISABLE	(0x00)


#define	AXISDATA_REG		(0x28)
#define WHOAMI_LIS2DE_ACC	(0x33)	/*	Expctd content for WAI	*/

/*	CONTROL REGISTERS	*/
#define WHO_AM_I		(0x0F)	/*	WhoAmI register		*/
#define	TEMP_CFG_REG		(0x1F)	/*	temper sens control reg	*/
/* ctrl 1: ODR3 ODR2 ODR ODR0 LPen Zenable Yenable Zenable */
#define	CTRL_REG1		(0x20)	/*	control reg 1		*/
#define	CTRL_REG2		(0x21)	/*	control reg 2		*/
#define	CTRL_REG3		(0x22)	/*	control reg 3		*/
#define	CTRL_REG4		(0x23)	/*	control reg 4		*/
#define	CTRL_REG5		(0x24)	/*	control reg 5		*/
#define	CTRL_REG6		(0x25)	/*	control reg 6		*/

#define	FIFO_CTRL_REG		(0x2E)	/*	FiFo control reg	*/

#define	INT_CFG1		(0x30)	/*	interrupt 1 config	*/
#define	INT_SRC1		(0x31)	/*	interrupt 1 source	*/
#define	INT_THS1		(0x32)	/*	interrupt 1 threshold	*/
#define	INT_DUR1		(0x33)	/*	interrupt 1 duration	*/


#define	TT_CFG			(0x38)	/*	tap config		*/
#define	TT_SRC			(0x39)	/*	tap source		*/
#define	TT_THS			(0x3A)	/*	tap threshold		*/
#define	TT_LIM			(0x3B)	/*	tap time limit		*/
#define	TT_TLAT			(0x3C)	/*	tap time latency	*/
#define	TT_TW			(0x3D)	/*	tap time window		*/
/*	end CONTROL REGISTRES	*/


#define ALL_ZEROES		(0x00)

#define LIS2DE_ACC_PM_OFF		(0x00)
#define LIS2DE_ACC_ENABLE_ALL_AXES	(0x07)


#define PMODE_MASK		(0x08)
#define ODR_MASK		(0XF0)

#define LIS2DE_ACC_ODR1	(0x10)  /* 1Hz output data rate */
#define LIS2DE_ACC_ODR10	(0x20)  /* 10Hz output data rate */
#define LIS2DE_ACC_ODR25	(0x30)  /* 25Hz output data rate */
#define LIS2DE_ACC_ODR50	(0x40)  /* 50Hz output data rate */
#define LIS2DE_ACC_ODR100	(0x50)  /* 100Hz output data rate */
#define LIS2DE_ACC_ODR200	(0x60)  /* 200Hz output data rate */
#define LIS2DE_ACC_ODR400	(0x70)  /* 400Hz output data rate */
#define LIS2DE_ACC_ODR1250	(0x90)  /* 1250Hz output data rate */



#define	IA			(0x40)
#define	ZH			(0x20)
#define	ZL			(0x10)
#define	YH			(0x08)
#define	YL			(0x04)
#define	XH			(0x02)
#define	XL			(0x01)
/* */
/* CTRL REG BITS*/
#define	CTRL_REG3_I1_AOI1	(0x40)
#define	CTRL_REG4_BDU_ENABLE	(0x80)
#define	CTRL_REG4_BDU_MASK	(0x80)
#define	CTRL_REG6_I2_TAPEN	(0x80)
#define	CTRL_REG6_HLACTIVE	(0x02)
/* */
#define NO_MASK			(0xFF)
#define INT1_DURATION_MASK	(0x7F)
#define	INT1_THRESHOLD_MASK	(0x7F)
#define TAP_CFG_MASK		(0x3F)
#define	TAP_THS_MASK		(0x7F)
#define	TAP_TLIM_MASK		(0x7F)
#define	TAP_TLAT_MASK		NO_MASK
#define	TAP_TW_MASK		NO_MASK


/* TAP_SOURCE_REG BIT */
#define	DTAP			(0x20)
#define	STAP			(0x10)
#define	SIGNTAP			(0x08)
#define	ZTAP			(0x04)
#define	YTAP			(0x02)
#define	XTAZ			(0x01)


#define	FUZZ			0
#define	FLAT			0
#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5
#define	I2C_AUTO_INCREMENT	(0x80)

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_CTRL_REG6		5

#define	RES_INT_CFG1		6
#define	RES_INT_THS1		7
#define	RES_INT_DUR1		8

#define	RES_TT_CFG		9
#define	RES_TT_THS		10
#define	RES_TT_LIM		11
#define	RES_TT_TLAT		12
#define	RES_TT_TW		13

#define	RES_TEMP_CFG_REG	14
#define	RES_REFERENCE_REG	15
#define	RES_FIFO_CTRL_REG	16

#define	RESUME_ENTRIES		17
/* end RESUME STATE INDICES */

/* Polling delay in msecs */
#define POLL_INTERVAL_MIN_MS	5
#define POLL_INTERVAL_MAX_MS	4000
#define POLL_DEFAULT_INTERVAL_MS 60

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lis2de_acc_odr_table[] = {
		{    1, LIS2DE_ACC_ODR1250 },
		{    3, LIS2DE_ACC_ODR400  },
		{    5, LIS2DE_ACC_ODR200  },
		{   10, LIS2DE_ACC_ODR100  },
		{   20, LIS2DE_ACC_ODR50   },
		{   40, LIS2DE_ACC_ODR25   },
		{  100, LIS2DE_ACC_ODR10   },
		{ 1000, LIS2DE_ACC_ODR1    },
};

static int int1_gpio = LIS2DE_ACC_DEFAULT_INT1_GPIO;
static int int2_gpio = LIS2DE_ACC_DEFAULT_INT2_GPIO;
module_param(int1_gpio, int, S_IRUGO);
module_param(int2_gpio, int, S_IRUGO);

/* add by shengwang.luo 20141210 begin*/
static struct sensors_classdev sensors_cdev = {
	.name = "lis2de12_acc",
	.vendor = "STMicroelectronics",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "156.8",
	.resolution = "0.01",
	.sensor_power = "0.01",
	.min_delay = POLL_INTERVAL_MIN_MS * 1000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = POLL_DEFAULT_INTERVAL_MS,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_self_test = NULL,
};
/* add by shengwang.luo 20141210 end*/

struct lis2de_acc_status {
	struct i2c_client *client;
	struct lis2de_acc_platform_data *pdata;

	struct mutex lock;
	//struct delayed_work input_work;
	struct work_struct poll_work;
	struct hrtimer poll_timer;	
	struct workqueue_struct *poll_wq;

	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;
	int use_smbus;

	u8 resume_state[RESUME_ENTRIES];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;
    int sensitivity;
#ifdef POWER_REGULATOR
	struct regulator *vdd;
	struct regulator *vio;
	bool power_enabled; 
#endif
#ifdef DEBUG
	u8 reg_addr;
#endif

	// add by shengwang.luo, 20141210
	struct sensors_classdev cdev;
#ifdef SELF_TEST_NODE
	int testflag;
#endif
	spinlock_t xyz_data_lock;
 	int xyz[3];
};


static struct lis2de_acc_platform_data default_lis2de_acc_pdata = {
	.fs_range = LIS2DE_ACC_G_2G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,

	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.poll_interval = POLL_DEFAULT_INTERVAL_MS,
	.min_interval = LIS2DE_ACC_MIN_POLL_PERIOD_MS,
	.gpio_int1 = LIS2DE_ACC_DEFAULT_INT1_GPIO,
	.gpio_int2 = LIS2DE_ACC_DEFAULT_INT2_GPIO,
};

static int lis2de_acc_get_acceleration_data(struct lis2de_acc_status *stat, int *xyz);
static void lis2de_acc_report_values(struct lis2de_acc_status *stat, int *xyz);

static int lis2de_acc_i2c_read(struct lis2de_acc_status *stat, u8 *buf,
									int len)
{
	int ret;
	u8 reg = buf[0];
	u8 cmd = reg;
#ifdef DEBUG
	unsigned int ii;
#endif
/*
	if (len > sizeof(buf))
			dev_err(&stat->client->dev,
				"read error insufficient buffer length: "
				"len:%d, buf size=%d\n",
				len, sizeof(buf));
*/
	if (len > 1)
		cmd = (I2C_AUTO_INCREMENT | reg);
	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_read_byte_data(stat->client, cmd);
			buf[0] = ret & 0xff;
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_read_byte_data: ret=0x%02x, len:%d ,"
				"command=0x%02x, buf[0]=0x%02x\n",
				ret, len, cmd , buf[0]);
#endif
		} else if (len > 1) {
			ret = i2c_smbus_read_i2c_block_data(stat->client,
								cmd, len, buf);
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_read_i2c_block_data: ret:%d len:%d, "
				"command=0x%02x, ",
				ret, len, cmd);
			
			for (ii = 0; ii < len; ii++)
				printk(KERN_DEBUG "buf[%d]=0x%02x,",
								ii, buf[ii]);

			printk("\n");
#endif
		} else
			ret = -1;

		if (ret < 0) {
			dev_err(&stat->client->dev,
				"read transfer error: len:%d, command=0x%02x\n",
				len, cmd);
			return 0; /* failure */
		}
		return len; /* success */
	}

	ret = i2c_master_send(stat->client, &cmd, sizeof(cmd));
	if (ret != sizeof(cmd))
		return ret;

	return i2c_master_recv(stat->client, buf, len);
}

static int lis2de_acc_i2c_write(struct lis2de_acc_status *stat, u8 *buf,
									int len)
{
	int ret;
	u8 reg, value;
#ifdef DEBUG	
	unsigned int ii;
#endif
	if (len > 1)
		buf[0] = (I2C_AUTO_INCREMENT | buf[0]);

	reg = buf[0];
	value = buf[1];

	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_write_byte_data(stat->client,
								reg, value);
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_write_byte_data: ret=%d, len:%d, "
				"command=0x%02x, value=0x%02x\n",
				ret, len, reg , value);
#endif
			return ret;
		} else if (len > 1) {
			ret = i2c_smbus_write_i2c_block_data(stat->client,
							reg, len, buf + 1);
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_write_i2c_block_data: ret=%d, "
				"len:%d, command=0x%02x, ",
				ret, len, reg);
			
			for (ii = 0; ii < (len + 1); ii++)
				printk(KERN_DEBUG "value[%d]=0x%02x,",
								ii, buf[ii]);

			printk("\n");
#endif
			return ret;
		}
	}

	ret = i2c_master_send(stat->client, buf, len+1);
	return (ret == len+1) ? 0 : ret;
}

static int lis2de_acc_hw_init(struct lis2de_acc_status *stat)
{
	int err = -1;
	u8 buf[7];

	pr_info("%s: hw init start\n", LIS2DE_ACC_DEV_NAME);

	buf[0] = WHO_AM_I;
	err = lis2de_acc_i2c_read(stat, buf, 1);
	if (err < 0) {
		dev_warn(&stat->client->dev, "Error reading WHO_AM_I:"
				" is device available/working?\n");
		goto err_firstread;
	} else
		stat->hw_working = 1;

	if (buf[0] != WHOAMI_LIS2DE_ACC) {
		dev_err(&stat->client->dev,
			"device unknown. Expected: 0x%02x,"
			" Replies: 0x%02x\n",
			WHOAMI_LIS2DE_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}


	buf[0] = CTRL_REG1;
	buf[1] = stat->resume_state[RES_CTRL_REG1];
	err = lis2de_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = TEMP_CFG_REG;
	buf[1] = stat->resume_state[RES_TEMP_CFG_REG];
	err = lis2de_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = FIFO_CTRL_REG;
	buf[1] = stat->resume_state[RES_FIFO_CTRL_REG];
	err = lis2de_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = TT_THS;
	buf[1] = stat->resume_state[RES_TT_THS];
	buf[2] = stat->resume_state[RES_TT_LIM];
	buf[3] = stat->resume_state[RES_TT_TLAT];
	buf[4] = stat->resume_state[RES_TT_TW];
	err = lis2de_acc_i2c_write(stat, buf, 4);
	if (err < 0)
		goto err_resume_state;
	buf[0] = TT_CFG;
	buf[1] = stat->resume_state[RES_TT_CFG];
	err = lis2de_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = INT_THS1;
	buf[1] = stat->resume_state[RES_INT_THS1];
	buf[2] = stat->resume_state[RES_INT_DUR1];
	err = lis2de_acc_i2c_write(stat, buf, 2);
	if (err < 0)
		goto err_resume_state;
	buf[0] = INT_CFG1;
	buf[1] = stat->resume_state[RES_INT_CFG1];
	err = lis2de_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;
#if 1
	buf[0] = CTRL_REG5;
	buf[1] = 0x10;
	err = lis2de_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;
#endif
#if 0
	buf[0] = CTRL_REG2;
	buf[1] = stat->resume_state[RES_CTRL_REG2];
	buf[2] = stat->resume_state[RES_CTRL_REG3];
	buf[3] = stat->resume_state[RES_CTRL_REG4];
	buf[4] = stat->resume_state[RES_CTRL_REG5];
	buf[5] = stat->resume_state[RES_CTRL_REG6];
	err = lis2de_acc_i2c_write(stat, buf, 5);
	if (err < 0)
		goto err_resume_state;
#endif
	// add by shengwang.luo 20141210 begin
	buf[0] = CTRL_REG3;
	buf[1] = stat->resume_state[RES_CTRL_REG3];
	err = lis2de_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = CTRL_REG6;
	buf[1] = stat->resume_state[RES_CTRL_REG6];
	err = lis2de_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	// add by shengwang.luo 20141210 end
	stat->hw_initialized = 1;
	pr_info("%s: hw init done\n", LIS2DE_ACC_DEV_NAME);
	return 0;

err_firstread:
	stat->hw_working = 0;
err_unknown_device:
err_resume_state:
	stat->hw_initialized = 0;
	dev_err(&stat->client->dev, "hw init error 0x%02x,0x%02x: %d\n", buf[0],
			buf[1], err);
	return err;
}

#ifdef POWER_REGULATOR
static int lis2de_power_ctl(struct lis2de_acc_status *stat, bool on)
{
	int ret = 0;
    	
	dev_info(&stat->client->dev,"%s enable = %d, power_enabled = %d", __func__, on, stat->power_enabled);
	if (!on && stat->power_enabled) {
		ret = regulator_disable(stat->vdd);
		if (ret) {
			dev_err(&stat->client->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(stat->vio);
		if (ret) {
			dev_err(&stat->client->dev,
				"Regulator vio disable failed ret=%d\n", ret);
			ret=regulator_enable(stat->vdd);
			return ret;
		}
		stat->power_enabled = on;
		dev_dbg(&stat->client->dev, "lis2de_power_ctl on=%d\n", on);
	} else if (on && !stat->power_enabled) {

		ret = regulator_enable(stat->vdd);
		if (ret) {
			dev_err(&stat->client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_enable(stat->vio);
		if (ret) {
			dev_err(&stat->client->dev,
				"Regulator vio enable failed ret=%d\n", ret);
			regulator_disable(stat->vdd);
			return ret;
		}
		stat->power_enabled = on;
		dev_dbg(&stat->client->dev, "lis2de_power_ctl on=%d\n", on);
	} else {
		dev_warn(&stat->client->dev,
				"Power on=%d. enabled=%d\n",
				on, stat->power_enabled);
	}

	return ret;
}

static int lis2de_acc_power_init(struct lis2de_acc_status *stat, bool on)
{
	int ret;

	if (!on) {
		if (regulator_count_voltages(stat->vdd) > 0)
			regulator_set_voltage(stat->vdd,
					0, LIS2DE_VDD_MAX_UV);

		regulator_put(stat->vdd);

		if (regulator_count_voltages(stat->vio) > 0)
			regulator_set_voltage(stat->vio,
					0, LIS2DE_VIO_MAX_UV);

		regulator_put(stat->vio);
	} else {
		stat->vdd = regulator_get(&stat->client->dev, "vdd");
		if (IS_ERR(stat->vdd)) {
			ret = PTR_ERR(stat->vdd);
			dev_err(&stat->client->dev,
				"Regulator get failed vdd ret=%d\n", ret);
			return ret;
		}

		if (regulator_count_voltages(stat->vdd) > 0) {
			ret = regulator_set_voltage(stat->vdd,
					LIS2DE_VDD_MIN_UV,
					LIS2DE_VDD_MAX_UV);
			if (ret) {
				dev_err(&stat->client->dev,
					"Regulator set failed vdd ret=%d\n",
					ret);
				goto reg_vdd_put;
			}
		}

		stat->vio = regulator_get(&stat->client->dev, "vio");
		if (IS_ERR(stat->vio)) {
			ret = PTR_ERR(stat->vio);
			dev_err(&stat->client->dev,
				"Regulator get failed vio ret=%d\n", ret);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(stat->vio) > 0) {
			ret = regulator_set_voltage(stat->vio,
					LIS2DE_VIO_MIN_UV,
					LIS2DE_VIO_MAX_UV);
			if (ret) {
				dev_err(&stat->client->dev,
				"Regulator set failed vio ret=%d\n", ret);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(stat->vio);
reg_vdd_set:
	if (regulator_count_voltages(stat->vdd) > 0)
		regulator_set_voltage(stat->vdd, 0, LIS2DE_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(stat->vdd);
	return ret;
}

static int lis2de_device_ctl(struct lis2de_acc_status *stat, bool enable)
{
	int ret;
	struct device *dev = &stat->client->dev;

	if (enable && !stat->power_enabled) {
		ret = lis2de_power_ctl(stat, true);
		if (ret) {
			dev_err(dev, "Failed to enable device power\n");
			goto err_exit;
		}
		mdelay(10);	// add by shengwang.luo for need to delay from power-on to read i2c slave addr.  

	} else if (!enable && stat->power_enabled) {
	    ret = lis2de_power_ctl(stat, false);
	    if (ret) {
			dev_err(dev, "Failed to disable device power\n");
			goto err_exit;
		}
	} else {
		dev_dbg(dev, "device control: enable=%d, power_enabled=%d\n",
			enable, stat->power_enabled);
	}
	return 0;

err_exit:
	return ret;
}
#endif
//modify(add) by junfeng.zhou.sz for add power supply end . 
static void lis2de_acc_device_power_off(struct lis2de_acc_status *stat)
{
	int err;
	u8 buf[2] = { CTRL_REG1, LIS2DE_ACC_PM_OFF };
	err = lis2de_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		dev_err(&stat->client->dev, "soft power off failed: %d\n", err);

    	lis2de_device_ctl(stat,false);
	stat->hw_initialized = 0;
	
}

static int lis2de_acc_device_power_on(struct lis2de_acc_status *stat)
{
	int err = -1;
    
    	lis2de_device_ctl(stat,true);

	if (!stat->hw_initialized) {
		err = lis2de_acc_hw_init(stat);
		if (stat->hw_working == 1 && err < 0) {
			lis2de_acc_device_power_off(stat);
			return err;
		}
	}
	
	return 0;
}

#if defined LIS2DE_ACC_ENABLE_IRQ1 || defined LIS2DE_ACC_ENABLE_IRQ2
static irqreturn_t lis2de_acc_isr1(int irq, void *dev)
{
	struct lis2de_acc_status *stat = dev;
	
	disable_irq_nosync(irq);
	queue_work(stat->irq1_work_queue, &stat->irq1_work);
	
	return IRQ_HANDLED;
}

static irqreturn_t lis2de_acc_isr2(int irq, void *dev)
{
	struct lis2de_acc_status *stat = dev;

	disable_irq_nosync(irq);
	queue_work(stat->irq2_work_queue, &stat->irq2_work);
	pr_info("%s: isr2 queued\n", LIS2DE_ACC_DEV_NAME);

	return IRQ_HANDLED;
}
// modify by shengwang.luo 20141210 begin
static void lis2de_acc_irq1_work_func(struct work_struct *work)
{
	int xyz[3] = { 0 }; 
	int err;
	
	struct lis2de_acc_status *stat =
	container_of(work, struct lis2de_acc_status, irq1_work);
	/* TODO  add interrupt service procedure.
		 ie:lis2de_acc_get_int1_source(stat); */

	/* ; */

	mutex_lock(&stat->lock);
	err = lis2de_acc_get_acceleration_data(stat, xyz);
	if (err < 0)
		dev_err(&stat->client->dev, "get_acceleration_data failed\n");
	else
		lis2de_acc_report_values(stat, xyz);

	//schedule_delayed_work(&stat->input_work, msecs_to_jiffies(
	//		stat->pdata->poll_interval));
	mutex_unlock(&stat->lock);
#ifdef DEBUG
	pr_info("%s: IRQ1 triggered\n", LIS2DE_ACC_DEV_NAME);
#endif
/* exit: */
	enable_irq(stat->irq1);
}
// modify by shengwang.luo 20141210 end
static void lis2de_acc_irq2_work_func(struct work_struct *work)
{

	struct lis2de_acc_status *stat =
	container_of(work, struct lis2de_acc_status, irq2_work);
	/* TODO  add interrupt service procedure.
		 ie:lis2de_acc_get_tap_source(stat); */

	/* ; */

	pr_info("%s: IRQ2 triggered\n", LIS2DE_ACC_DEV_NAME);
/* exit: */
	enable_irq(stat->irq2);
}
#endif
static int lis2de_acc_update_fs_range(struct lis2de_acc_status *stat,
							u8 new_fs_range)
{
	int err = -1;

	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = LIS2DE_ACC_FS_MASK;

	switch (new_fs_range) {
	case LIS2DE_ACC_G_2G:
	    stat->sensitivity = LIS2DE_ACC_SENSITIVITY_2G;
		break;
		
	case LIS2DE_ACC_G_4G:
	    stat->sensitivity = LIS2DE_ACC_SENSITIVITY_4G;
		break;
		
	case LIS2DE_ACC_G_8G:
	    stat->sensitivity = LIS2DE_ACC_SENSITIVITY_8G;
		break;
		
	case LIS2DE_ACC_G_16G:
	    stat->sensitivity = LIS2DE_ACC_SENSITIVITY_16G;
		break;
		
	default:
		dev_err(&stat->client->dev, "invalid fs range requested: %u\n",
				new_fs_range);
		return -EINVAL;
	}


	/* Updates configuration register 4,
	* which contains fs range setting */
	buf[0] = CTRL_REG4;
	err = lis2de_acc_i2c_read(stat, buf, 1);
	if (err < 0)
		goto error;
	init_val = buf[0];
	stat->resume_state[RES_CTRL_REG4] = init_val;
	new_val = new_fs_range;
	updated_val = ((mask & new_val) | ((~mask) & init_val));
	buf[1] = updated_val;
	buf[0] = CTRL_REG4;
	err = lis2de_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto error;
	stat->resume_state[RES_CTRL_REG4] = updated_val;


	return err;
error:
	dev_err(&stat->client->dev,
			"update fs range failed 0x%02x,0x%02x: %d\n",
			buf[0], buf[1], err);

	return err;
}

static int lis2de_acc_update_odr(struct lis2de_acc_status *stat,
							int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lis2de_acc_odr_table) - 1; i >= 0; i--) {
		if ((lis2de_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
								|| (i == 0))
			break;
	}
	config[1] = lis2de_acc_odr_table[i].mask;

	config[1] |= LIS2DE_ACC_ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&stat->enabled)) {
		config[0] = CTRL_REG1;
		err = lis2de_acc_i2c_write(stat, config, 1);
		if (err < 0)
			goto error;
		stat->resume_state[RES_CTRL_REG1] = config[1];
	}

	return err;

error:
	dev_err(&stat->client->dev, "update odr failed 0x%02x,0x%02x: %d\n",
			config[0], config[1], err);

	return err;
}



static int lis2de_acc_register_write(struct lis2de_acc_status *stat,
					u8 *buf, u8 reg_address, u8 new_value)
{
	int err = -1;

		/* Sets configuration register at reg_address
		 *  NOTE: this is a straight overwrite  */
		buf[0] = reg_address;
		buf[1] = new_value;
		err = lis2de_acc_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;
	return err;
}

/*
static int lis2de_acc_register_read(struct lis2de_acc_status *stat,
							u8 *buf, u8 reg_address)
{

	int err = -1;
	buf[0] = (reg_address);
	err = lis2de_acc_i2c_read(stat, buf, 1);
	return err;
}
*/

/*
static int lis2de_acc_register_update(struct lis2de_acc_status *stat,
		u8 *buf, u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;
	err = lis2de_acc_register_read(stat, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[1];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = lis2de_acc_register_write(stat, buf, reg_address,
				updated_val);
	}
	return err;
}
*/

/* Add by shengwang.luo for data filter function on 2015/03/23 begin */
/* ***********************************************************
 this func used for data filter,  
 divide for 3 function for each x,y,z axis data .
 data_in :   data in
 level :  number data in filter
 
 return :      data after filter.
**************************************************************/
#ifdef LIS2DE_ACC_ENABLE_FILTER
int AccelFilter_AntiShake_X(int data_in, int level)
{
	int max, min;
	static int queue_x[FILTER_ELEMENT_DTH] = {0};
	int sum;
	int i;
	
	if(level > FILTER_ELEMENT_DTH) {
		level = FILTER_ELEMENT_DTH;
	}

	queue_x[0] = data_in;
	max = queue_x[0] ;
	min = queue_x[0] ;
	sum = queue_x[0] ;

	for(i = level - 1; i > 0; i--)
	{
		if(queue_x[i] > max)
			max = queue_x[i];
		else if(queue_x[i] < min)
		{
			min = queue_x[i];
		}
		sum = sum + queue_x[i];
		queue_x[i] = queue_x[i-1];
	}
	i = level - 2;
	sum = sum - max - min;
	sum = sum / i;

	return sum;
}

int AccelFilter_AntiShake_Y(int data_in, int level)
{
	int max, min;
	static int queue_y[FILTER_ELEMENT_DTH] = {0};
	int sum;
	int i;
	
	if(level > FILTER_ELEMENT_DTH) {
		level = FILTER_ELEMENT_DTH;
	}
	
	queue_y[0] = data_in;
	max = queue_y[0];
	min = queue_y[0];
	sum = queue_y[0];

	for(i = level - 1; i > 0; i--)
	{
		if(queue_y[i] > max)
			max = queue_y[i];
		else if(queue_y[i] < min)
		{
			min = queue_y[i];
		}
		sum = sum + queue_y[i];
		queue_y[i] = queue_y[i-1];
	}
	i = level - 2;
	sum = sum - max - min;
	sum = sum / i;

	return sum;
}

int AccelFilter_AntiShake_Z(int data_in, int level)
{
	int max, min;
	static int queue_z[FILTER_ELEMENT_DTH] = {0};
	int sum;
	int i;
	
	if(level > FILTER_ELEMENT_DTH) {
		level = FILTER_ELEMENT_DTH;
	}

	queue_z[0] = data_in;
	max = queue_z[0] ;
	min = queue_z[0] ;
	sum = queue_z[0] ;

	for(i = level - 1; i > 0; i--)
	{
		if(queue_z[i] > max)
			max = queue_z[i];
		else if(queue_z[i] < min)
		{
			min = queue_z[i];
		}
		sum = sum + queue_z[i];
		queue_z[i] = queue_z[i-1] ;
	}
	i = level - 2;
	sum = sum - max - min;
	sum = sum / i;

	return sum;
}
#endif
/* Add by shengwang.luo for data filter function on 2015/03/23 end */

/* Begin modify by shengwang.luo for change to get data mothod on 20150210 */
static int lis2de_acc_get_acceleration_data(
				struct lis2de_acc_status *stat, int *xyz)
{
	int err = -1;
	int raw_data[3];
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };
/*	u8 raw_x;
	u8 raw_y;
	u8 raw_z;*/
#if 0
	u8 ctrl_1;
	u8 ctrl_2;
	u8 ctrl_3;
	u8 ctrl_4;
	u8 ctrl_5;
	u8 ctrl_6;
#endif
	unsigned long flags;
	acc_data[0] = (AXISDATA_REG);
	err = lis2de_acc_i2c_read(stat, acc_data, 6);
	if (err < 0)
		return err;

/*	raw_x = 0x29;
	err = lis2de_acc_i2c_read(stat, &raw_x, 1);
	if (err < 0)
		return err;
	raw_y = 0x2B;
	err = lis2de_acc_i2c_read(stat, &raw_y, 1);
	if (err < 0)
		return err;
	raw_z = 0x2D;
	err = lis2de_acc_i2c_read(stat, &raw_z, 1);
	if (err < 0)
		return err;

	acc_data[1] = raw_x;
	acc_data[3] = raw_y;
	acc_data[5] = raw_z;
*/
//	pr_info("x=%d, y=%d, z=%d\n",raw_x, raw_y, raw_z);

#ifdef LIS2DE12
	hw_d[0] = (s16) ((s8) acc_data[1]) * stat->sensitivity / 10;
	hw_d[1] = (s16) ((s8) acc_data[3]) * stat->sensitivity / 10;
	hw_d[2] = (s16) ((s8) acc_data[5]) * stat->sensitivity / 10;
#else
    	hw_d[0] = (s16) ((((s8) acc_data[1])<<8 | ((u8) acc_data[0])) >> 4);
	hw_d[1] = (s16) ((((s8) acc_data[3])<<8 | ((u8) acc_data[2])) >> 4);
	hw_d[2] = (s16) ((((s8) acc_data[5])<<8 | ((u8) acc_data[4])) >> 4);
#endif

	raw_data[0] = ((stat->pdata->negate_x) ? (-hw_d[stat->pdata->axis_map_x])
		   : (hw_d[stat->pdata->axis_map_x]));
	raw_data[1] = ((stat->pdata->negate_y) ? (-hw_d[stat->pdata->axis_map_y])
		   : (hw_d[stat->pdata->axis_map_y]));
	raw_data[2] = ((stat->pdata->negate_z) ? (-hw_d[stat->pdata->axis_map_z])
		   : (hw_d[stat->pdata->axis_map_z]));

	// Add by shengwang.luo for data filter on 2015/03/23 begin.
	spin_lock_irqsave(&stat->xyz_data_lock, flags);
	#ifdef LIS2DE_ACC_ENABLE_FILTER
	xyz[0] = AccelFilter_AntiShake_X(raw_data[0], 10);
	xyz[1] = AccelFilter_AntiShake_Y(raw_data[1], 10);
	xyz[2] = AccelFilter_AntiShake_Z(raw_data[2], 10);
	#else
	xyz[0] = raw_data[0];
	xyz[1] = raw_data[1];
	xyz[2] = raw_data[2];
	#endif
	spin_unlock_irqrestore(&stat->xyz_data_lock, flags);
	// Add by shengwang.luo for data filter on 2015/03/23 end.

#ifdef DEBUG

	dev_info(&stat->client->dev,"%s read x=%d, y=%d, z=%d\n",
			LIS2DE_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);

#endif

#if 0
	ctrl_1 = CTRL_REG1;
	lis2de_acc_i2c_read(stat, &ctrl_1, 1);
	ctrl_2 = CTRL_REG2;
	lis2de_acc_i2c_read(stat, &ctrl_2, 1);
	ctrl_3 = CTRL_REG3;
	lis2de_acc_i2c_read(stat, &ctrl_3, 1);
	ctrl_4 = CTRL_REG4;
	lis2de_acc_i2c_read(stat, &ctrl_4, 1);
	ctrl_5 = CTRL_REG5;
	lis2de_acc_i2c_read(stat, &ctrl_5, 1);
	ctrl_6 = CTRL_REG6;
	err = lis2de_acc_i2c_read(stat, &ctrl_6, 1);
	pr_info("%s ctrl1~6=0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
			LIS2DE_ACC_DEV_NAME, ctrl_1, ctrl_2, ctrl_3, ctrl_4, ctrl_5, ctrl_6);

#endif
	return err;
}
/* End modify by shengwang.luo for change to get data mothod on 20150210 */

static void lis2de_acc_report_values(struct lis2de_acc_status *stat,
					int *xyz)
{
	unsigned long flags;
	ktime_t ts;
	ts = ktime_get_boottime();

	spin_lock_irqsave(&stat->xyz_data_lock, flags);
	input_report_abs(stat->input_dev, ABS_X, xyz[0]);
	input_report_abs(stat->input_dev, ABS_Y, xyz[1]);
	input_report_abs(stat->input_dev, ABS_Z, xyz[2]);
	spin_unlock_irqrestore(&stat->xyz_data_lock, flags);
	
	input_event(stat->input_dev, EV_SYN, SYN_TIME_SEC,
			ktime_to_timespec(ts).tv_sec);
	input_event(stat->input_dev, EV_SYN, SYN_TIME_NSEC,
			ktime_to_timespec(ts).tv_nsec);
	input_sync(stat->input_dev);
	//printk(KERN_ERR " ABS_X:%d ABS_Y:%d ABS_Z:%d \n",xyz[0],xyz[1],xyz[2]);
}


static int lis2de_acc_enable(struct lis2de_acc_status *stat)
{
	int err;
	
	if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {
		err = lis2de_acc_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);
			return err;
		}
#if defined LIS2DE_ACC_ENABLE_IRQ1 || defined LIS2DE_ACC_ENABLE_IRQ2
		if (stat->hw_initialized) {
				if (stat->pdata->gpio_int1 >= 0)
					enable_irq(stat->irq1);
				if (stat->pdata->gpio_int2 >= 0)
					enable_irq(stat->irq2);
		}
#else
		hrtimer_start(&stat->poll_timer, ns_to_ktime(stat->pdata->poll_interval * NSEC_PER_MSEC), HRTIMER_MODE_REL);
#endif	
	}
	return 0;
}

static int lis2de_acc_disable(struct lis2de_acc_status *stat)
{
	if (stat->hw_initialized) {
		if (stat->pdata->gpio_int1 >= 0)
			disable_irq_nosync(stat->irq1);
		if (stat->pdata->gpio_int2 >= 0)
			disable_irq_nosync(stat->irq2);
		//stat->hw_initialized = 0;
	}

	if (atomic_cmpxchg(&stat->enabled, 1, 0)) {
		if (stat->pdata->gpio_int1 < 0)
		{
			//cancel_delayed_work_sync(&stat->input_work);
			hrtimer_cancel(&stat->poll_timer);
			cancel_work_sync(&stat->poll_work);
		}
		
		lis2de_acc_device_power_off(stat);
	}

	return 0;
}

// add by shengwang.luo 20141210 begin
static int lis2de_set_enable(struct sensors_classdev *sensors_cdev,
				unsigned int enable)
{
	struct lis2de_acc_status *stat = container_of(sensors_cdev, 
						struct lis2de_acc_status, cdev);
	int err;

	if (enable)
		err = lis2de_acc_enable(stat);
	else
		err = lis2de_acc_disable(stat);
	
	return err;
	
}
static int lis2de_cdev_poll_delay(struct sensors_classdev *sensors_cdev,
				unsigned int delay_ms)
{
	struct lis2de_acc_status *stat = container_of(sensors_cdev, 
						struct lis2de_acc_status, cdev);
	int err;

	mutex_lock(&stat->lock);
	stat->pdata->poll_interval = delay_ms;
	err = lis2de_acc_update_odr(stat, delay_ms);
	mutex_unlock(&stat->lock);
	
	return err;	
}
// end
static ssize_t read_single_reg(struct device *dev, char *buf, u8 reg)
{
	ssize_t ret;
	struct lis2de_acc_status *stat = dev_get_drvdata(dev);
	int err;

	u8 data = reg;
	err = lis2de_acc_i2c_read(stat, &data, 1);
	if (err < 0)
		return err;
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;

}

static int write_reg(struct device *dev, const char *buf, u8 reg,
		u8 mask, int resumeIndex)
{
	int err = -1;
	struct lis2de_acc_status *stat = dev_get_drvdata(dev);
	u8 x[2];
	u8 new_val;
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	new_val = ((u8) val & mask);
	x[0] = reg;
	x[1] = new_val;
	err = lis2de_acc_register_write(stat, x, reg, new_val);
	if (err < 0)
		return err;
	stat->resume_state[resumeIndex] = new_val;
	return err;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct lis2de_acc_status *stat = dev_get_drvdata(dev);
	mutex_lock(&stat->lock);
	val = stat->pdata->poll_interval;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lis2de_acc_status *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	interval_ms = max((unsigned int)interval_ms, stat->pdata->min_interval);
	mutex_lock(&stat->lock);
	stat->pdata->poll_interval = interval_ms;
	lis2de_acc_update_odr(stat, interval_ms);
	mutex_unlock(&stat->lock);
	return size;
}

static ssize_t attr_get_range(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	char val;
	struct lis2de_acc_status *stat = dev_get_drvdata(dev);
	char range = 2;
	mutex_lock(&stat->lock);
	val = stat->pdata->fs_range ;
	switch (val) {
	case LIS2DE_ACC_G_2G:
		range = 2;
		break;
	case LIS2DE_ACC_G_4G:
		range = 4;
		break;
	case LIS2DE_ACC_G_8G:
		range = 8;
		break;
	case LIS2DE_ACC_G_16G:
		range = 16;
		break;
	}
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct lis2de_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	int err;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	switch (val) {
	case 2:
		range = LIS2DE_ACC_G_2G;
		break;
	case 4:
		range = LIS2DE_ACC_G_4G;
		break;
	case 8:
		range = LIS2DE_ACC_G_8G;
		break;
	case 16:
		range = LIS2DE_ACC_G_16G;
		break;
	default:
		dev_err(&stat->client->dev, "invalid range request: %lu,"
				" discarded\n", val);
		return -EINVAL;
	}
	mutex_lock(&stat->lock);
	err = lis2de_acc_update_fs_range(stat, range);
	if (err < 0) {
		mutex_unlock(&stat->lock);
		return err;
	}
	stat->pdata->fs_range = range;
	mutex_unlock(&stat->lock);
	dev_info(&stat->client->dev, "range set to: %lu g\n", val);

	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lis2de_acc_status *stat = dev_get_drvdata(dev);
	int val = atomic_read(&stat->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lis2de_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val) {
		lis2de_acc_enable(stat);
	}
	else {
		lis2de_acc_disable(stat);
	}

	return size;
}

static ssize_t attr_set_intconfig1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_CFG1, NO_MASK, RES_INT_CFG1);
}

static ssize_t attr_get_intconfig1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_CFG1);
}

static ssize_t attr_set_duration1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_DUR1, INT1_DURATION_MASK, RES_INT_DUR1);
}

static ssize_t attr_get_duration1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_DUR1);
}

static ssize_t attr_set_thresh1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THS1, INT1_THRESHOLD_MASK, RES_INT_THS1);
}

static ssize_t attr_get_thresh1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_THS1);
}

static ssize_t attr_get_source1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_SRC1);
}

static ssize_t attr_set_click_cfg(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_CFG, TAP_CFG_MASK, RES_TT_CFG);
}

static ssize_t attr_get_click_cfg(struct device *dev,
		struct device_attribute *attr,	char *buf)
{

	return read_single_reg(dev, buf, TT_CFG);
}

static ssize_t attr_get_click_source(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_SRC);
}

static ssize_t attr_set_click_ths(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_THS, TAP_THS_MASK, RES_TT_THS);
}

static ssize_t attr_get_click_ths(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_THS);
}

static ssize_t attr_set_click_tlim(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_LIM, TAP_TLIM_MASK, RES_TT_LIM);
}

static ssize_t attr_get_click_tlim(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_LIM);
}

static ssize_t attr_set_click_tlat(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_TLAT, TAP_TLAT_MASK, RES_TT_TLAT);
}

static ssize_t attr_get_click_tlat(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_TLAT);
}

static ssize_t attr_set_click_tw(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_TLAT, TAP_TW_MASK, RES_TT_TLAT);
}

static ssize_t attr_get_click_tw(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_TLAT);
}


#ifdef DEBUG
/* PAY ATTENTION: These DEBUG functions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct lis2de_acc_status *stat = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&stat->lock);
	x[0] = stat->reg_addr;
	mutex_unlock(&stat->lock);
	x[1] = val;
	rc = lis2de_acc_i2c_write(stat, x, 1);
	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct lis2de_acc_status *stat = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&stat->lock);
	data = stat->reg_addr;
	mutex_unlock(&stat->lock);
	rc = lis2de_acc_i2c_read(stat, &data, 1);
	/*TODO: error need to be managed */
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lis2de_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&stat->lock);
	stat->reg_addr = val;
	mutex_unlock(&stat->lock);
	return size;
}
#endif

// Begin add by shengwang.luo for test node on 20150209
static ssize_t attr_value_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int xyz[3] = { 0 };
	struct input_dev *input = to_input_dev(dev);
	struct lis2de_acc_status *stat = input_get_drvdata(input);

	lis2de_acc_get_acceleration_data(stat, xyz);
	
	return snprintf(buf, PAGE_SIZE, "%d %d %d\n", xyz[0], xyz[1], xyz[2]);	
}
#ifdef SELF_TEST_NODE
static ssize_t attr_get_selftest(struct device *dev,
			        struct device_attribute *attr, char *buf)
{
	struct lis2de_acc_status *stat = dev_get_drvdata(dev);
	int err = -1;
	int xyz[3] = { 0 };
	u8 reg[2];
	u8 addr;
	int outx_nost = 0;
	int outy_nost = 0;
	int outz_nost = 0;
	int outx_st = 0;
	int outy_st = 0;
	int outz_st = 0;
	int diff_x = 0;
	int diff_y = 0;
	int diff_z = 0;
	int count = 0;

	stat->testflag = 0;	/*set selftest status */

	/* write 0x57 to CTRL_REG1(20h). write 0x80 to CTRL_REG4(23h) */	 	
	reg[0] = 0x20;	// register addr
	reg[1] = 0x57;	// value
	err = lis2de_acc_i2c_write(stat, reg, 1);
	if (err < 0) {
		pr_info("write register 0x20 failed.\n");
		goto write_register_failed;
	}
	// print the value of register
	addr = 0x20;
	lis2de_acc_i2c_read(stat, &addr, 1);
	pr_info("register 0x20: 0x%x\n", addr);
	reg[0] = 0x23;
	reg[1] = 0x80;
	err = lis2de_acc_i2c_write(stat, reg, 1);
	if (err < 0) {
		pr_info("write register 0x23 failed.\n");
		goto write_register_failed;
	}
	// print the value of register
	addr = 0x23;
	lis2de_acc_i2c_read(stat, &addr, 1);
	pr_info("register 0x23: 0x%x\n", addr);

	/* read OUTX/OUTY/OUTZ to clear ZYXDA bit in STATUS_REG2(27h)
	 * wait for 80ms for stable output */
	mdelay(80);
	addr = 0x29; /* OUT_X(29h) */
	err = lis2de_acc_i2c_read(stat, &addr, 1);
	if (err < 0) {
		pr_info("read register 0x29 failed.\n");
		goto read_register_failed;
	}
	addr = 0x2B; /* OUT_Y(2Bh) */
	err = lis2de_acc_i2c_read(stat, &addr, 1);
	if (err < 0) {
		pr_info("read register 0x2B failed.\n");
		goto read_register_failed;
	}
	addr = 0x2D; /* OUT_Z(2Dh) */
	err = lis2de_acc_i2c_read(stat, &addr, 1);
	if (err < 0) {
		pr_info("read register 0x2D failed.\n");
		goto read_register_failed;
	}

	while(count < 5) { /* read the output register 5 times */
		addr = 0x27; /* STATUS_REG2(27h) */
		lis2de_acc_i2c_read(stat, &addr, 1);
		if ((addr & 0x08) == 0x08) { /* checking ZYXDA bit */
			lis2de_acc_get_acceleration_data(stat, xyz);
			pr_info("The %d time get_data: x = %d, y = %d, z = %d\n", count+1, xyz[0], xyz[1], xyz[2]);
			outx_nost += xyz[0];
			outy_nost += xyz[1];
			outz_nost += xyz[2];
			xyz[0] = 0;
			xyz[1] = 0;
			xyz[2] = 0;
			count++;
			mdelay(80);		
		}
	}

	/* average the stored data on each axis. */	
	outx_nost = outx_nost / 5;
	outy_nost = outy_nost / 5;
	outz_nost = outz_nost / 5;
	pr_info("average value outx_nost = %d, outy_nost = %d, outz_nost = %d\n", 
		outx_nost, outy_nost, outz_nost);	
	mdelay(80);
	
	/* Enable Self Test : write 0x82 to CTRL_REG4(23h), then wait for 80 ms */
	reg[0] = 0x23;
	reg[1] = 0x82; // Self-test 0
	//reg[1] = 0x84; // Self-test 1
	err = lis2de_acc_i2c_write(stat, reg, 1);
	if (err < 0) {
		pr_info("write register 0x23 failed.\n");
		goto write_register_failed;
	}
	// print the value of register
	addr = 0x23;
	lis2de_acc_i2c_read(stat, &addr, 1);
	pr_info("register 0x23: 0x%x\n", addr);
	mdelay(80);

	/* clear ZYXDA again */
	addr = 0x29; /* OUT_X(29h) */
	err = lis2de_acc_i2c_read(stat, &addr, 1);
	if (err < 0) {
		pr_info("read register 0x29 failed.\n");
		goto st_read_register_failed;
	}
	addr = 0x2B; /* OUT_Y(2Bh) */
	err = lis2de_acc_i2c_read(stat, &addr, 1);
	if (err < 0) {
		pr_info("read register 0x2B failed.\n");
		goto st_read_register_failed;
	}
	addr = 0x2D; /* OUT_Z(2Dh) */
	err = lis2de_acc_i2c_read(stat, &addr, 1);
	if (err < 0) {
		pr_info("read register 0x2D failed.\n");
		goto st_read_register_failed;
	}

	count = 0;
	while(count < 5) { /* read the output register 5 times */
		addr = 0x27; /* STATUS_REG2(27h) */
		lis2de_acc_i2c_read(stat, &addr, 1);
		if ((addr & 0x08) == 0x08) { /* checking ZYXDA bit */
			lis2de_acc_get_acceleration_data(stat, xyz);
			pr_info("The %d time get_data: x = %d, y = %d, z = %d\n", count+1, xyz[0], xyz[1], xyz[2]);
			outx_st += xyz[0];
			outy_st += xyz[1];
			outz_st += xyz[2];
			xyz[0] = 0;
			xyz[1] = 0;
			xyz[2] = 0;
			count++;
			mdelay(80);		
		}
	}
	/* average the stored data on each axis */
	outx_st = outx_st / 5;
	outy_st = outy_st / 5;
	outz_st = outz_st / 5;
	pr_info("average value outx_st = %d, outy_st = %d, outz_st = %d\n", 
		outx_st, outy_st, outz_st);

	/* acceleration self-test output change if between Min and Max */
	diff_x = abs(outx_st - outx_nost);
	diff_y = abs(outy_st - outy_nost);
	diff_z = abs(outz_st - outz_nost);
	if (diff_x < MIN_X || diff_x > MAX_X || diff_y < MIN_Y || diff_y > MAX_Y || diff_z < MIN_Z || diff_z > MAX_Z) {
		pr_info("error data: diff_x: %d, diff_y: %d, diff_z: %d\n", diff_x, diff_y, diff_z);
		goto diff_fail;
	}
			
	stat->testflag = 1;
	/* disable self test */
	reg[0] = 0x23;
	reg[1] = 0x80;
	lis2de_acc_i2c_write(stat, reg, 1);
	pr_info("selftest success. %d\n", stat->testflag);

	return sprintf(buf, "Self-test success. %d\n", stat->testflag);
				
diff_fail:	
st_read_register_failed:
	reg[0] = 0x23;
	reg[1] = 0x80;
	lis2de_acc_i2c_write(stat, reg, 1);

read_register_failed:
write_register_failed:
	pr_info("selftest failed. %d\n", stat->testflag);
	
	return sprintf(buf, "Self-test failed. %d\n", stat->testflag);
}
#endif
// End add by shengwang.luo for test node on 20150209 

static struct device_attribute attributes[] = {

	__ATTR(pollrate_ms, 0664, attr_get_polling_rate, attr_set_polling_rate),
	__ATTR(range, 0664, attr_get_range, attr_set_range),
	__ATTR(enable_device, 0664, attr_get_enable, attr_set_enable),
	__ATTR(int1_config, 0664, attr_get_intconfig1, attr_set_intconfig1),
	__ATTR(int1_duration, 0664, attr_get_duration1, attr_set_duration1),
	__ATTR(int1_threshold, 0664, attr_get_thresh1, attr_set_thresh1),
	__ATTR(int1_source, 0444, attr_get_source1, NULL),
	__ATTR(click_config, 0664, attr_get_click_cfg, attr_set_click_cfg),
	__ATTR(click_source, 0444, attr_get_click_source, NULL),
	__ATTR(click_threshold, 0664, attr_get_click_ths, attr_set_click_ths),
	__ATTR(click_timelimit, 0664, attr_get_click_tlim, attr_set_click_tlim),
	__ATTR(click_timelatency, 0664, attr_get_click_tlat,
							attr_set_click_tlat),
	__ATTR(click_timewindow, 0664, attr_get_click_tw, attr_set_click_tw),

#ifdef DEBUG
	__ATTR(reg_value, 0666, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0666, NULL, attr_addr_set),
#endif

// Begin add by shengwang.luo for test node on 20150209
	__ATTR(show_value, 0444, attr_value_show, NULL),
#ifdef SELF_TEST_NODE
	__ATTR(self_test, 0444, attr_get_selftest, NULL),
#endif
// End add by shengwang.luo for test node on 20150209
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

/*
static void lis2de_acc_input_work_func(struct work_struct *work)
{
	struct lis2de_acc_status *stat;

	int xyz[3] = { 0 };
	int err;
	stat = container_of((struct delayed_work *)work,
			struct lis2de_acc_status, input_work);

	mutex_lock(&stat->lock);
	err = lis2de_acc_get_acceleration_data(stat, xyz);
	if (err < 0)
		dev_err(&stat->client->dev, "get_acceleration_data failed\n");
	else
		lis2de_acc_report_values(stat, xyz);

	schedule_delayed_work(&stat->input_work, msecs_to_jiffies(
			stat->pdata->poll_interval));
	mutex_unlock(&stat->lock);

}
*/

static void lis2de_poll_work_func(struct work_struct *work)
{
	struct lis2de_acc_status *stat;

	//int xyz[3] = { 0 };
	int err;
	stat = container_of(work, struct lis2de_acc_status, poll_work);

	mutex_lock(&stat->lock);
	err = lis2de_acc_get_acceleration_data(stat, stat->xyz);
	if (err < 0)
		dev_err(&stat->client->dev, "get_acceleration_data failed\n");
	/*else
		lis2de_acc_report_values(stat, xyz);*/
	
	mutex_unlock(&stat->lock);
}


static enum hrtimer_restart lis2de_poll_timer_func(struct hrtimer *timer)
{
	struct lis2de_acc_status *stat = container_of(timer, struct lis2de_acc_status, poll_timer);
	queue_work(stat->poll_wq, &stat->poll_work);
	if(0 != stat->xyz[0] || 0 != stat->xyz[1] || 0 != stat->xyz[2]) {
		lis2de_acc_report_values(stat, stat->xyz);
	}
	hrtimer_forward_now(&stat->poll_timer, ns_to_ktime(stat->pdata->poll_interval * NSEC_PER_MSEC));
	//printk("%s\n", __func__);
	return HRTIMER_RESTART;	
	
}

int lis2de_acc_input_open(struct input_dev *input)
{
	struct lis2de_acc_status *stat = input_get_drvdata(input);
	dev_dbg(&stat->client->dev, "%s\n", __func__);
	stat->xyz[0] = 0;
	stat->xyz[1] = 0;
	stat->xyz[2] = 0;
	return lis2de_acc_enable(stat);
}

void lis2de_acc_input_close(struct input_dev *dev)
{
	struct lis2de_acc_status *stat = input_get_drvdata(dev);
	dev_dbg(&stat->client->dev, "%s\n", __func__);
	lis2de_acc_disable(stat);
}

static int lis2de_acc_validate_pdata(struct lis2de_acc_status *stat)
{
	/* checks for correctness of minimal polling period */
	stat->pdata->min_interval =
		max((unsigned int)LIS2DE_ACC_MIN_POLL_PERIOD_MS,
						stat->pdata->min_interval);

	stat->pdata->poll_interval = max(stat->pdata->poll_interval,
			stat->pdata->min_interval);

	if (stat->pdata->axis_map_x > 2 ||
		stat->pdata->axis_map_y > 2 ||
		 stat->pdata->axis_map_z > 2) {
		dev_err(&stat->client->dev, "invalid axis_map value "
			"x:%u y:%u z%u\n", stat->pdata->axis_map_x,
					stat->pdata->axis_map_y,
						stat->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (stat->pdata->negate_x > 1 || stat->pdata->negate_y > 1
			|| stat->pdata->negate_z > 1) {
		dev_err(&stat->client->dev, "invalid negate value "
			"x:%u y:%u z:%u\n", stat->pdata->negate_x,
				stat->pdata->negate_y, stat->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (stat->pdata->poll_interval < stat->pdata->min_interval) {
		dev_err(&stat->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lis2de_acc_input_init(struct lis2de_acc_status *stat)
{
	int err;
	if (stat->pdata->gpio_int1 < 0)
	{
		//INIT_DELAYED_WORK(&stat->input_work, lis2de_acc_input_work_func);
		stat->poll_wq = create_singlethread_workqueue("lis2de_poll_wq");
		INIT_WORK(&stat->poll_work, lis2de_poll_work_func);
		hrtimer_init(&stat->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		stat->poll_timer.function = lis2de_poll_timer_func;
	}

	stat->input_dev = input_allocate_device();
	if (!stat->input_dev) {
		err = -ENOMEM;
		dev_err(&stat->client->dev, "input device allocation failed\n");
		goto err0;
	}

	stat->input_dev->open = lis2de_acc_input_open;
	stat->input_dev->close = lis2de_acc_input_close;
	stat->input_dev->name = LIS2DE_ACC_DEV_NAME;
	/* stat->input_dev->name = "accelerometer"; */
	stat->input_dev->id.bustype = BUS_I2C;
	stat->input_dev->dev.parent = &stat->client->dev;

	input_set_drvdata(stat->input_dev, stat);

	set_bit(EV_ABS, stat->input_dev->evbit);
	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, stat->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, stat->input_dev->absbit);

	input_set_abs_params(stat->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);
	/*	next is used for interruptA sources data if the case */
	input_set_abs_params(stat->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptB sources data if the case */
	input_set_abs_params(stat->input_dev, ABS_WHEEL, INT_MIN,
								INT_MAX, 0, 0);

	err = input_register_device(stat->input_dev);
	if (err) {
		dev_err(&stat->client->dev,
				"unable to register input device %s\n",
				stat->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(stat->input_dev);

err0:
	if (stat->pdata->gpio_int1 < 0)
	{
		hrtimer_cancel(&stat->poll_timer);
		cancel_work_sync(&stat->poll_work);
		destroy_workqueue(stat->poll_wq);
	}
	return err;
}

static void lis2de_acc_input_cleanup(struct lis2de_acc_status *stat)
{
	if (stat->pdata->gpio_int1 < 0)
	{
		hrtimer_cancel(&stat->poll_timer);
		cancel_work_sync(&stat->poll_work);
		destroy_workqueue(stat->poll_wq);
	}
	
	input_unregister_device(stat->input_dev);
	input_free_device(stat->input_dev);
}

static int lis2de_acc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct lis2de_acc_status *stat;

	u32 smbus_func = (I2C_FUNC_SMBUS_BYTE_DATA |
			I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK);

	int err = -1;

	dev_info(&client->dev, "probe start.\n");
	//printk(KERN_ERR "%s:probe start",__func__);

	stat = kzalloc(sizeof(struct lis2de_acc_status), GFP_KERNEL);
	if (stat == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
					"%d\n", err);
		goto exit_check_functionality_failed;
	}
	/* Init stat struct data */
	stat->xyz[0] = 0;
	stat->xyz[1] = 0;
	stat->xyz[2] = 0;

	/* Support for both I2C and SMBUS adapter interfaces. */
	stat->use_smbus = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_warn(&client->dev, "client not i2c capable\n");
		if (i2c_check_functionality(client->adapter, smbus_func)) {
			stat->use_smbus = 1;
			dev_warn(&client->dev, "client using SMBUS\n");
		} else {
			err = -ENODEV;
			dev_err(&client->dev, "client nor SMBUS capable\n");
			goto exit_check_functionality_failed;
		}
	}

	spin_lock_init(&stat->xyz_data_lock);
	mutex_init(&stat->lock);
	mutex_lock(&stat->lock);

	stat->client = client;
	i2c_set_clientdata(client, stat);

	stat->pdata = kmalloc(sizeof(*stat->pdata), GFP_KERNEL);
	if (stat->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for pdata: %d\n",
				err);
		goto err_mutexunlock;
	}

	if (client->dev.platform_data == NULL) {
		default_lis2de_acc_pdata.gpio_int1 = int1_gpio;
		default_lis2de_acc_pdata.gpio_int2 = int2_gpio;
		memcpy(stat->pdata, &default_lis2de_acc_pdata,
							sizeof(*stat->pdata));
		dev_info(&client->dev, "using default plaform_data\n");
	} else {
		memcpy(stat->pdata, client->dev.platform_data,
							sizeof(*stat->pdata));
	}
	
	err = lis2de_acc_validate_pdata(stat);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

	if (stat->pdata->init) {
		err = stat->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}
#if defined LIS2DE_ACC_ENABLE_IRQ1 || defined LIS2DE_ACC_ENABLE_IRQ2
	if (stat->pdata->gpio_int1 >= 0) {
		stat->irq1 = gpio_to_irq(stat->pdata->gpio_int1);
		pr_info("%s: %s has set irq1 to irq: %d, "
							"mapped on gpio:%d\n",
			LIS2DE_ACC_DEV_NAME, __func__, stat->irq1,
							stat->pdata->gpio_int1);

	}
	if (stat->pdata->gpio_int2 >= 0) {
		stat->irq2 = gpio_to_irq(stat->pdata->gpio_int2);
		pr_info("%s: %s has set irq2 to irq: %d, "
							"mapped on gpio:%d\n",
			LIS2DE_ACC_DEV_NAME, __func__, stat->irq2,
							stat->pdata->gpio_int2);
	}
#endif
	memset(stat->resume_state, 0, ARRAY_SIZE(stat->resume_state));

	stat->resume_state[RES_CTRL_REG1] = (ALL_ZEROES |
					LIS2DE_ACC_ENABLE_ALL_AXES);
	stat->resume_state[RES_CTRL_REG4] = (ALL_ZEROES | CTRL_REG4_BDU_ENABLE);

	
	stat->resume_state[RES_CTRL_REG3] |= 0x10;
	stat->resume_state[RES_CTRL_REG6] &= 0xfd; // 0: interrupt active high
	//stat->resume_state[RES_CTRL_REG6] |= 0x02; // 1: interrupt active low

/*
	stat->resume_state[RES_CTRL_REG2] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG3] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG4] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG5] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG6] = ALL_ZEROES;

	stat->resume_state[RES_TEMP_CFG_REG] = ALL_ZEROES;
	stat->resume_state[RES_FIFO_CTRL_REG] = ALL_ZEROES;
	stat->resume_state[RES_INT_CFG1] = ALL_ZEROES;
	stat->resume_state[RES_INT_THS1] = ALL_ZEROES;
	stat->resume_state[RES_INT_DUR1] = ALL_ZEROES;

	stat->resume_state[RES_TT_CFG] = ALL_ZEROES;
	stat->resume_state[RES_TT_THS] = ALL_ZEROES;
	stat->resume_state[RES_TT_LIM] = ALL_ZEROES;
	stat->resume_state[RES_TT_TLAT] = ALL_ZEROES;
	stat->resume_state[RES_TT_TW] = ALL_ZEROES;
*/
    err = lis2de_acc_power_init(stat, true);
	err = lis2de_acc_device_power_on(stat);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}
	atomic_set(&stat->enabled, 1);

	err = lis2de_acc_update_fs_range(stat, stat->pdata->fs_range);
	if (err < 0) {
		dev_err(&client->dev, "update_fs_range failed\n");
		goto  err_power_off;
	}
	
	err = lis2de_acc_update_odr(stat, stat->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	err = lis2de_acc_input_init(stat);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_power_off;
	}

	/*add by shengwang.luo 20141210 begin*/
	stat->cdev = sensors_cdev;
	stat->cdev.sensors_enable = lis2de_set_enable;
	stat->cdev.sensors_poll_delay = lis2de_cdev_poll_delay;
	err = sensors_classdev_register(&stat->input_dev->dev, &stat->cdev);
	if (err) {
		err = -EINVAL;
		goto exit_unregister_acc_class;	
	}
	/*add by shengwang.luo 20141210 end*/

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev,
		   "device LIS2DE_ACC_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}

	lis2de_acc_device_power_off(stat);

	/* As default, do not report information */
	atomic_set(&stat->enabled, 0);
#if defined LIS2DE_ACC_ENABLE_IRQ1 || defined LIS2DE_ACC_ENABLE_IRQ2
	if (stat->pdata->gpio_int1 >= 0) {
		INIT_WORK(&stat->irq1_work, lis2de_acc_irq1_work_func);
		stat->irq1_work_queue =
			create_singlethread_workqueue("lis2de_acc_wq1");
		if (!stat->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue1: %d\n", err);
			goto err_remove_sysfs_int;
		}
 		err = gpio_request(stat->pdata->gpio_int1,"lis2de-int");        
		if(err < 0)
		{
			printk(KERN_ERR "%s: gpio_request, err=%d", __func__, err);
			goto err_destoyworkqueue1;
		}
		err = gpio_direction_input(stat->pdata->gpio_int1);
		if(err < 0)
		{
			printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, err);
			goto err_destoyworkqueue1;
		}	 
		err = request_irq(stat->irq1, lis2de_acc_isr1,
			IRQF_TRIGGER_RISING, "lis2de_acc_irq1", stat);
		if (err < 0) {
			dev_err(&client->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(stat->irq1);
	}
	if (stat->pdata->gpio_int2 >= 0) {
		INIT_WORK(&stat->irq2_work, lis2de_acc_irq2_work_func);
		stat->irq2_work_queue =
			create_singlethread_workqueue("lis2de_acc_wq2");
		if (!stat->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue2: %d\n", err);
			goto err_free_irq1;
		}
		err = request_irq(stat->irq2, lis2de_acc_isr2,
			IRQF_TRIGGER_RISING, "lis2de_acc_irq2", stat);
		if (err < 0) {
			dev_err(&client->dev, "request irq2 failed: %d\n", err);
			goto err_destoyworkqueue2;
		}
		disable_irq_nosync(stat->irq2);
	}
#endif
	mutex_unlock(&stat->lock);

	dev_info(&client->dev, "%s: probe ok!\n", LIS2DE_ACC_DEV_NAME);

	return 0;
	
#if defined LIS2DE_ACC_ENABLE_IRQ1 || defined LIS2DE_ACC_ENABLE_IRQ2
err_destoyworkqueue2:
	if (stat->pdata->gpio_int2 >= 0)
		destroy_workqueue(stat->irq2_work_queue);
err_free_irq1:
	free_irq(stat->irq1, stat);
err_destoyworkqueue1:
	if (stat->pdata->gpio_int1 >= 0)
		destroy_workqueue(stat->irq1_work_queue);
err_remove_sysfs_int:
	remove_sysfs_interfaces(&client->dev);
#endif
err_input_cleanup:
	lis2de_acc_input_cleanup(stat);
exit_unregister_acc_class:
	sensors_classdev_unregister(&stat->cdev);
err_power_off:
	lis2de_acc_device_power_off(stat);
	lis2de_acc_power_init(stat, false);
err_pdata_init:
	if (stat->pdata->exit)
		stat->pdata->exit();
exit_kfree_pdata:
	kfree(stat->pdata);
err_mutexunlock:
	mutex_unlock(&stat->lock);
/* err_freedata: */
	kfree(stat);
exit_check_functionality_failed:
	pr_err("%s: Driver Init failed\n", LIS2DE_ACC_DEV_NAME);
	
	return err;
}

static int lis2de_acc_remove(struct i2c_client *client)
{

	struct lis2de_acc_status *stat = i2c_get_clientdata(client);

	dev_info(&stat->client->dev, "driver removing\n");

	if (stat->pdata->gpio_int1 >= 0) {
		free_irq(stat->irq1, stat);
		gpio_free(stat->pdata->gpio_int1);
		destroy_workqueue(stat->irq1_work_queue);
	}

	if (stat->pdata->gpio_int2 >= 0) {
		free_irq(stat->irq2, stat);
		gpio_free(stat->pdata->gpio_int2);
		destroy_workqueue(stat->irq2_work_queue);
	}

	lis2de_acc_disable(stat);
	lis2de_acc_input_cleanup(stat);

	remove_sysfs_interfaces(&client->dev);

	if (stat->pdata->exit)
		stat->pdata->exit();
	kfree(stat->pdata);
	kfree(stat);

	return 0;
}

#ifdef CONFIG_PM
static int lis2de_acc_resume(struct i2c_client *client)
{
	struct lis2de_acc_status *stat = i2c_get_clientdata(client);

	printk("G-sensor enter resume: %s\n", __func__);
	if (stat->on_before_suspend)
		return lis2de_acc_enable(stat);
	return 0;
}

static int lis2de_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lis2de_acc_status *stat = i2c_get_clientdata(client);

	printk("\nG-sensor enter suspend: %s\n", __func__);
	stat->on_before_suspend = atomic_read(&stat->enabled);
	return lis2de_acc_disable(stat);
}
#else
#define lis2de_acc_suspend	NULL
#define lis2de_acc_resume	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id lis2de_acc_id[]
		= { { LIS2DE_ACC_DEV_NAME, 0 }, { }, };

MODULE_DEVICE_TABLE(i2c, lis2de_acc_id);

static const struct of_device_id lis2de12_of_match[] = {
	{ .compatible = "st,lis2de12", },
	{ },
};

static struct i2c_driver lis2de_acc_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = LIS2DE_ACC_DEV_NAME,
			.of_match_table = lis2de12_of_match,
		  },
	.probe = lis2de_acc_probe,
	.remove = lis2de_acc_remove,
	.suspend = lis2de_acc_suspend,
	.resume = lis2de_acc_resume,
	.id_table = lis2de_acc_id,
};

static int lis2de_acc_init(void)
{
	pr_info("%s accelerometer driver: init\n",
						LIS2DE_ACC_DEV_NAME);
	return i2c_add_driver(&lis2de_acc_driver);
}

static void lis2de_acc_exit(void)
{

	pr_info("%s accelerometer driver exit\n",
						LIS2DE_ACC_DEV_NAME);

	i2c_del_driver(&lis2de_acc_driver);
	return;
}

module_init(lis2de_acc_init);
module_exit(lis2de_acc_exit);

MODULE_DESCRIPTION("lis2de accelerometer sysfs driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, STMicroelectronics");
MODULE_LICENSE("GPL");

