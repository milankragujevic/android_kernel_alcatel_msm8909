/*
 * Copyright (c) 2014, Linux Foundation. All rights reserved.
 * Linux Foundation chooses to take subject only to the GPLv2 license
 * terms, and distributes only under these terms.
 * Copyright (C) 2010 MEMSIC, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/regulator/consumer.h>
#include <linux/input.h>
#include <linux/regmap.h>
#include <linux/sensors.h>
#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/kthread.h>

#include "mmc3524x.h"

#define MMC3524X_DELAY_TM_MS	10

#define MMC3524X_DELAY_SET_MS	75
#define MMC3524X_DELAY_RESET_MS	75

#define MMC3524X_RETRY_COUNT	10
#define MMC3524X_DEFAULT_INTERVAL_MS	100
#define MMC3524X_TIMEOUT_SET_MS	15000

#define MMC3524X_PRODUCT_ID	0x09

/* POWER SUPPLY VOLTAGE RANGE */
#define MMC3524X_VDD_MIN_UV	2000000
#define MMC3524X_VDD_MAX_UV	3300000
#define MMC3524X_VIO_MIN_UV	1750000
#define MMC3524X_VIO_MAX_UV	1950000
#define MMC3524X_MIN_DELAY      10
#define MMC3524X_MAX_DELAY      2000

enum {
	OBVERSE_X_AXIS_FORWARD = 0,
	OBVERSE_X_AXIS_RIGHTWARD,
	OBVERSE_X_AXIS_BACKWARD,
	OBVERSE_X_AXIS_LEFTWARD,
	REVERSE_X_AXIS_FORWARD,
	REVERSE_X_AXIS_RIGHTWARD,
	REVERSE_X_AXIS_BACKWARD,
	REVERSE_X_AXIS_LEFTWARD,
	MMC3524X_DIR_COUNT,
};
static unsigned short otpMatrix[3] = {1000,1000,1350};

static char *mmc3524x_dir[MMC3524X_DIR_COUNT] = {
	[OBVERSE_X_AXIS_FORWARD] = "obverse-x-axis-forward",
	[OBVERSE_X_AXIS_RIGHTWARD] = "obverse-x-axis-rightward",
	[OBVERSE_X_AXIS_BACKWARD] = "obverse-x-axis-backward",
	[OBVERSE_X_AXIS_LEFTWARD] = "obverse-x-axis-leftward",
	[REVERSE_X_AXIS_FORWARD] = "reverse-x-axis-forward",
	[REVERSE_X_AXIS_RIGHTWARD] = "reverse-x-axis-rightward",
	[REVERSE_X_AXIS_BACKWARD] = "reverse-x-axis-backward",
	[REVERSE_X_AXIS_LEFTWARD] = "reverse-x-axis-leftward",
};

static s8 mmc3524x_rotation_matrix[MMC3524X_DIR_COUNT][9] = {
	[OBVERSE_X_AXIS_FORWARD] = {0, -1, 0, 1, 0, 0, 0, 0, 1},
	[OBVERSE_X_AXIS_RIGHTWARD] = {1, 0, 0, 0, 1, 0, 0, 0, 1},
	[OBVERSE_X_AXIS_BACKWARD] = {0, 1, 0, -1, 0, 0, 0, 0, 1},
	[OBVERSE_X_AXIS_LEFTWARD] = {-1, 0, 0, 0, -1, 0, 0, 0, 1},
	[REVERSE_X_AXIS_FORWARD] = {0, 1, 0, 1, 0, 0, 0, 0, -1},
	[REVERSE_X_AXIS_RIGHTWARD] = {1, 0, 0, 0, -1, 0, 0, 0, -1},
	[REVERSE_X_AXIS_BACKWARD] = {0, -1, 0, -1, 0, 0, 0, 0, -1},
	[REVERSE_X_AXIS_LEFTWARD] = {-1, 0, 0, 0, 1, 0, 0, 0, -1},
};

struct mmc3524x_vec {
	int x;
	int y;
	int z;
};

struct mmc3524x_data {
	struct mutex		ecompass_lock;
	struct mutex		ops_lock;
	struct delayed_work	dwork;
	struct workqueue_struct *data_wq;
	struct sensors_classdev	cdev;
	struct mmc3524x_vec	last;

	struct i2c_client	*i2c;
	struct input_dev	*input;
	struct regulator	*vdd;
	struct regulator	*vio;
	struct regmap		*regmap;

	int			dir;
	int			auto_report;
	int			enable;
	atomic_t			poll_interval;
	int			power_enabled;
	unsigned long		timeout;
	struct hrtimer		mag_timer;
	bool			use_hrtimer;
};

static struct sensors_classdev sensors_cdev = {
	.name = "mmc3524x-mag",
	.vendor = "MEMSIC, Inc",
	.version = 1,
	.handle = SENSORS_MAGNETIC_FIELD_HANDLE,
	.type = SENSOR_TYPE_MAGNETIC_FIELD,
	.max_range = "1228.8",
//	.resolution = "0.0488228125",                                    //2048 
    .resolution = "0.09765625",                     //1024
	.sensor_power = "0.35",
	.min_delay = MMC3524X_MIN_DELAY*1000,
        .max_delay = MMC3524X_MAX_DELAY*1000,  //2000ms  0.5hz  
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = MMC3524X_DEFAULT_INTERVAL_MS,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

//static struct i2c_client *this_client;
//static struct regmap *this_regmap;
//static DEFINE_MUTEX(ecompass_ioctl_lock);
static struct mmc3524x_data *mmc3524x_data_struct;

static int mmc3524x_read_xyz(struct mmc3524x_data *memsic,
		struct mmc3524x_vec *vec)
{
	int count = 0;
	unsigned char data[6];
	unsigned int status;
	struct mmc3524x_vec tmp;
	int rc = 0;

	mutex_lock(&memsic->ecompass_lock);
#if 1
	/* mmc3524x need to be set periodly to avoid overflow */
	if (time_after(jiffies, memsic->timeout)) {
		rc = regmap_write(memsic->regmap, MMC3524X_REG_CTRL,
				MMC3524X_CTRL_REFILL);
		if (rc) {
			dev_err(&memsic->i2c->dev, "write reg %d failed at %d.(%d)\n",
					MMC3524X_REG_CTRL, __LINE__, rc);
			goto exit;
		}

		/* Time from refill cap to SET */
		msleep(MMC3524X_DELAY_SET_MS);

		rc = regmap_write(memsic->regmap, MMC3524X_REG_CTRL,
				MMC3524X_CTRL_SET);
		if (rc) {
			dev_err(&memsic->i2c->dev, "write reg %d failed at %d.(%d)\n",
					MMC3524X_REG_CTRL, __LINE__, rc);
			goto exit;
		}

		/* Wait time to complete SET/RESET */
		usleep_range(1000, 1500);
		memsic->timeout = jiffies +
			msecs_to_jiffies(MMC3524X_TIMEOUT_SET_MS);

		dev_dbg(&memsic->i2c->dev, "mmc3524x reset is done\n");

		/* Re-send the TM command */
		rc = regmap_write(memsic->regmap, MMC3524X_REG_CTRL,
				MMC3524X_CTRL_TM);
		if (rc) {
			dev_err(&memsic->i2c->dev, "write reg %d failed at %d.(%d)\n",
					MMC3524X_REG_CTRL, __LINE__, rc);
			goto exit;
		}
	}
#endif
	/* Read MD */
	rc = regmap_read(memsic->regmap, MMC3524X_REG_DS, &status);
	if (rc) {
		dev_err(&memsic->i2c->dev, "read reg %d failed at %d.(%d)\n",
				MMC3524X_REG_DS, __LINE__, rc);
		goto exit;

	}

	while ((!(status & 0x01)) && (count < MMC3524X_RETRY_COUNT)) {
		/* Read MD again*/
		rc = regmap_read(memsic->regmap, MMC3524X_REG_DS, &status);
		if (rc) {
			dev_err(&memsic->i2c->dev, "read reg %d failed at %d.(%d)\n",
					MMC3524X_REG_DS, __LINE__, rc);
			goto exit;

		}

		/* Wait more time to get valid data */
		usleep_range(1000, 1500);
		count++;
	}

	if (count >= MMC3524X_RETRY_COUNT) {
		dev_err(&memsic->i2c->dev, "TM not work!!");
		rc = -EFAULT;
		goto exit;
	}

	/* read xyz raw data */
	rc = regmap_bulk_read(memsic->regmap, MMC3524X_REG_DATA, data, 6);
	if (rc) {
		dev_err(&memsic->i2c->dev, "read reg %d failed at %d.(%d)\n",
				MMC3524X_REG_DS, __LINE__, rc);
		goto exit;
	}

	//tmp.x = (((u8)data[1]) << 8 | (u8)data[0]) - 32768;                                                //3516
	//tmp.y = (((u8)data[3]) << 8 | (u8)data[2]) - 32768;
	//tmp.z = (((u8)data[5]) << 8 | (u8)data[4]) - 32768;

	tmp.x = (((u8)data[1]) << 8 | (u8)data[0]) - 32768;                                                    //3524
	tmp.y = (((u8)data[3]) << 8 | (u8)data[2]) - (((u8)data[5]) << 8 | (u8)data[4]) ;
	tmp.z = (((u8)data[3]) << 8 | (u8)data[2]) +  (((u8)data[5]) << 8 | (u8)data[4]) - 32768 - 32768;

	dev_dbg(&memsic->i2c->dev, "raw data:%d %d %d %d %d %d",
			data[0], data[1], data[2], data[3], data[4], data[5]);
	dev_dbg(&memsic->i2c->dev, "raw x:%d y:%d z:%d\n", tmp.x, tmp.y, tmp.z);

	vec->x = tmp.x;
	vec->y = tmp.y;
	vec->z = -tmp.z;

exit:
	/* send TM cmd before read */
	if (regmap_write(memsic->regmap, MMC3524X_REG_CTRL, MMC3524X_CTRL_TM)) {
		dev_warn(&memsic->i2c->dev, "write reg %d failed at %d.(%d)\n",
				MMC3524X_REG_CTRL, __LINE__, rc);
	}

	mutex_unlock(&memsic->ecompass_lock);
	return rc;
}

static void mmc3524x_report_axis_data(struct mmc3524x_data *memsic,struct mmc3524x_vec report)
{
	ktime_t	timestamp = ktime_get_boottime();
	input_report_abs(memsic->input, ABS_X, report.x);
	input_report_abs(memsic->input, ABS_Y, report.y);
	input_report_abs(memsic->input, ABS_Z, report.z);
	input_event(memsic->input, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(timestamp).tv_sec);
	input_event(memsic->input, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(timestamp).tv_nsec);
	input_sync(memsic->input);
}
static void mmc3524x_poll_work(struct work_struct *work)
{
	struct mmc3524x_data *memsic = container_of((struct delayed_work *)work,
			struct mmc3524x_data, dwork); 
	int ret;
	s8 *tmp;
	struct mmc3524x_vec vec;
	struct mmc3524x_vec report;
	//ktime_t	timestamp = ktime_get_boottime();
	unsigned long delay = msecs_to_jiffies(atomic_read(&memsic->poll_interval));
	

	vec.x = vec.y = vec.z = 0;

	ret = mmc3524x_read_xyz(memsic, &vec);
	if (ret) {
		dev_warn(&memsic->i2c->dev, "read xyz failed\n");
		goto exit;
	}
		//OtpProcess(&vec, rawDataTemp);
		vec.x=vec.x*otpMatrix[0]/1000;
		vec.y=vec.y*otpMatrix[1]/1000;
		vec.z=vec.z*otpMatrix[2]/1000;
		
	tmp = &mmc3524x_rotation_matrix[memsic->dir][0];
	report.x = tmp[0] * vec.x + tmp[1] * vec.y + tmp[2] * vec.z;
	report.y = tmp[3] * vec.x + tmp[4] * vec.y + tmp[5] * vec.z;
	report.z = tmp[6] * vec.x + tmp[7] * vec.y + tmp[8] * vec.z;
	memsic->last=report;

	
exit:
	queue_delayed_work(memsic->data_wq, &memsic->dwork,delay);
	if (!memsic->use_hrtimer)
  	schedule_delayed_work(&memsic->dwork,	msecs_to_jiffies(atomic_read(&memsic->poll_interval)));
}

static void mmc3524x_work_func(struct work_struct *work)
{
	int ret;
	s8 *tmp;
	struct mmc3524x_data *memsic = container_of((struct delayed_work *)work,
			struct mmc3524x_data, dwork); 
	struct mmc3524x_vec vec;
	struct mmc3524x_vec report; 
	unsigned long delay = msecs_to_jiffies(atomic_read(&memsic->poll_interval));
	vec.x = vec.y = vec.z = 0;
	ret = mmc3524x_read_xyz(memsic, &vec);
	if (ret) {
		dev_warn(&memsic->i2c->dev, "read xyz failed\n");
		goto exit;
	}
		vec.x=vec.x*otpMatrix[0]/1000;
		vec.y=vec.y*otpMatrix[1]/1000;
		vec.z=vec.z*otpMatrix[2]/1000;
	tmp = &mmc3524x_rotation_matrix[memsic->dir][0];
	report.x = tmp[0] * vec.x + tmp[1] * vec.y + tmp[2] * vec.z;
	report.y = tmp[3] * vec.x + tmp[4] * vec.y + tmp[5] * vec.z;
	report.z = tmp[6] * vec.x + tmp[7] * vec.y + tmp[8] * vec.z;
	memsic->last=report;
	mmc3524x_report_axis_data(memsic,memsic->last);
		
exit:
	queue_delayed_work(memsic->data_wq, &memsic->dwork,delay);
	if (!memsic->use_hrtimer)
  	schedule_delayed_work(&memsic->dwork,	msecs_to_jiffies(atomic_read(&memsic->poll_interval)));
}
static enum hrtimer_restart mag_timer_handle(struct hrtimer *hrtimer)
{
	struct mmc3524x_data *memsic;
	ktime_t ktime;
	memsic = container_of(hrtimer, struct mmc3524x_data, mag_timer);
	mmc3524x_report_axis_data(memsic,memsic->last);
	ktime = ktime_set(0, atomic_read(&memsic->poll_interval) * NSEC_PER_MSEC);
	hrtimer_forward_now(&memsic->mag_timer, ktime);
	return HRTIMER_RESTART;
}

static int mmc3524x_init_input(struct input_dev **input)
{
	int err = 0;

	*input = input_allocate_device();
	if (!*input)
		return -ENOMEM;

	(*input)->name = "compass";
	(*input)->phys = "mmc3524x/input0";
	(*input)->id.bustype = BUS_I2C;

	set_bit(EV_ABS, (*input)->evbit);

	input_set_abs_params(*input, ABS_X, -2047, 2047, 0, 0);
	input_set_abs_params(*input, ABS_Y, -2047, 2047, 0, 0);
	input_set_abs_params(*input, ABS_Z, -2047, 2047, 0, 0);

	input_set_capability(*input, EV_REL, REL_X);
	input_set_capability(*input, EV_REL, REL_Y);
	input_set_capability(*input, EV_REL, REL_Z);

	err = input_register_device(*input);
	if (err) {
		input_free_device(*input);
		return err;
	}

	return err;
}

static int mmc3524x_power_init(struct mmc3524x_data *data)
{
	int rc;

	data->vdd = devm_regulator_get(&data->i2c->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->i2c->dev,
				"Regualtor get failed vdd rc=%d\n", rc);
		return rc;
	}
	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd,
				MMC3524X_VDD_MIN_UV, MMC3524X_VDD_MAX_UV);
		if (rc) {
			dev_err(&data->i2c->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
			goto exit;
		}
	}

	data->vio = devm_regulator_get(&data->i2c->dev, "vio");
	if (IS_ERR(data->vio)) {
		rc = PTR_ERR(data->vio);
		dev_err(&data->i2c->dev,
				"Regulator get failed vio rc=%d\n", rc);
		goto reg_vdd_set;
	}

	if (regulator_count_voltages(data->vio) > 0) {
		rc = regulator_set_voltage(data->vio,
				MMC3524X_VIO_MIN_UV, MMC3524X_VIO_MAX_UV);
		if (rc) {
			dev_err(&data->i2c->dev,
					"Regulator set failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}
	}

	 /* The minimum time to operate device after VDD valid is 10 ms. */


	return 0;

reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, MMC3524X_VDD_MAX_UV);
exit:
	return rc;

}

static int mmc3524x_power_deinit(struct mmc3524x_data *data)
{
	if (!IS_ERR_OR_NULL(data->vio)) {
		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
					MMC3524X_VIO_MAX_UV);
		devm_regulator_put(data->vio);

	}

	if (!IS_ERR_OR_NULL(data->vdd)) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
					MMC3524X_VDD_MAX_UV);

		devm_regulator_put(data->vdd);
	}

	//data->power_enabled = false;

	return 0;
}

static int mmc3524x_power_set(struct mmc3524x_data *memsic, bool on)
{
	int rc = 0;

		mutex_lock(&memsic->ecompass_lock);

	if (!on && memsic->power_enabled) {
		rc = regulator_disable(memsic->vdd);
		if (rc) {
			dev_err(&memsic->i2c->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			goto err_vdd_disable;
		}

		rc = regulator_disable(memsic->vio);
		if (rc) {
			dev_err(&memsic->i2c->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			goto err_vio_disable;
		}
		memsic->power_enabled = false;

		goto err_vdd_disable;
	} else if (on && !memsic->power_enabled) {

		rc = regulator_enable(memsic->vdd);
		if (rc) {
			dev_err(&memsic->i2c->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_enable;
		}

		rc = regulator_enable(memsic->vio);
		if (rc) {
			dev_err(&memsic->i2c->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_enable;
		}
		memsic->power_enabled = true;


		/* The minimum time to operate after VDD valid is 10 ms */
		usleep_range(15000, 20000);

		goto err_vdd_disable;
	} else {
		dev_warn(&memsic->i2c->dev,
				"Power on=%d. enabled=%d\n",
				on, memsic->power_enabled);
		goto err_vdd_disable;
	}

err_vio_enable:
	regulator_disable(memsic->vio);
err_vdd_enable:
	mutex_unlock(&memsic->ecompass_lock);
	return rc;

err_vio_disable:
	if (regulator_enable(memsic->vdd))
		dev_warn(&memsic->i2c->dev, "Regulator vdd enable failed\n");
err_vdd_disable:
	mutex_unlock(&memsic->ecompass_lock);
	return rc;
}
static int ReadMMC3524x_OTP(struct mmc3524x_data *memsic)
{
	static unsigned char data[4];
	signed short stemp = 0;
	unsigned short utemp = 0;
	int rc = 0;

	rc = regmap_bulk_read(memsic->regmap, 0x1b, data, 4);
	if (rc) {
		dev_err(&memsic->i2c->dev, " ReadMMC3524x_OTP failed.(%d)\n", rc);
		return rc;
	}
	
	otpMatrix[0] = 1000;
	stemp = (signed short)(((data[1]&0x03) << 4) | (data[2] >> 4));
	if(stemp >= 32)
		stemp = 32 - stemp;
	otpMatrix[1] = (unsigned short)(stemp * 6 + 1000);	//*0.006*1000

	stemp = (signed short)(data[3] & 0x3f);
	if(stemp >= 32)
		stemp = 32 - stemp;
     utemp = (unsigned short)(stemp * 6 + 1000);        //magnify 1000 times
    otpMatrix[2] = utemp + (utemp*3)/10 + (utemp*30%100 + utemp*5)/100;

	printk(" read otp sucessful %d %d %d ",otpMatrix[0],otpMatrix[1],otpMatrix[2]);
	return rc;
}
static int mmc3524x_check_device(struct i2c_client *client)
{
	struct 	mmc3524x_data *memsic = i2c_get_clientdata(client);
	unsigned int data;
	int rc;

	rc = regmap_read(memsic->regmap, MMC3524X_REG_PRODUCTID_1, &data);
	if (rc) {
		dev_err(&memsic->i2c->dev, "read reg %d failed.(%d)\n",
				MMC3524X_REG_DS, rc);
		return rc;

	}
    printk("%s,%d,0x%x\n",__FUNCTION__,__LINE__,data);
	if ((data&0x0F)!= MMC3524X_PRODUCT_ID)
		return -ENODEV;

	return 0;
}

static int mmc3524x_parse_dt(struct i2c_client *client,
		struct mmc3524x_data *memsic)
{
	struct device_node *np = client->dev.of_node;
	const char *tmp;
	int rc;
	int i;
        if (of_property_read_bool(np, "memsic,hrtimer"))
            memsic->use_hrtimer = 1;
        else
            memsic->use_hrtimer = 0;

	rc = of_property_read_string(np, "memsic,dir", &tmp);

	/* does not have a value or the string is not null-terminated */
	if (rc && (rc != -EINVAL)) {
		dev_err(&client->dev, "Unable to read memsic,dir\n");
		return rc;
	}

	for (i = 0; i < ARRAY_SIZE(mmc3524x_dir); i++) {
		if (strcmp(mmc3524x_dir[i], tmp) == 0)
			break;
	}

	if (i >= ARRAY_SIZE(mmc3524x_dir)) {
		dev_err(&client->dev, "Invalid memsic,dir property");
		return -EINVAL;
	}

     memsic->dir = i;

        //memsic->dir = 1;

	if (of_property_read_bool(np, "memsic,auto-report"))
		memsic->auto_report = 1;
	else
		memsic->auto_report = 0;

	return 0;
}

static int mmc3524x_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	int rc = 0;
  	int delay_ms;
  	ktime_t ktime;
	struct mmc3524x_data *memsic = container_of(sensors_cdev,
			struct mmc3524x_data, cdev);

	mutex_lock(&memsic->ops_lock);

	if (enable && (!memsic->enable)) {
		rc = mmc3524x_power_set(memsic, true);
		if (rc) {
			dev_err(&memsic->i2c->dev, "Power up failed\n");
			goto exit;
		}

		/* send TM cmd before read */
		rc = regmap_write(memsic->regmap, MMC3524X_REG_CTRL,
				MMC3524X_CTRL_TM);
		if (rc) {
			dev_err(&memsic->i2c->dev, "write reg %d failed.(%d)\n",
					MMC3524X_REG_CTRL, rc);
			goto exit;
		}

		memsic->timeout = jiffies;
		if (memsic->auto_report && !memsic->use_hrtimer) {
			  delay_ms = atomic_read(&memsic->poll_interval);
			  queue_delayed_work(memsic->data_wq, &memsic->dwork,	msecs_to_jiffies(atomic_read(&memsic->poll_interval)));
		} else {
		    queue_delayed_work(memsic->data_wq, &memsic->dwork,	msecs_to_jiffies(atomic_read(&memsic->poll_interval)));
		    ktime = ktime_set(0, atomic_read(&memsic->poll_interval) * NSEC_PER_MSEC);
		    hrtimer_start(&memsic->mag_timer, ktime, HRTIMER_MODE_REL);
		}
	} else if ((!enable) && memsic->enable) {
		if (memsic->auto_report && !memsic->use_hrtimer)
			cancel_delayed_work_sync(&memsic->dwork);
		else
			cancel_delayed_work_sync(&memsic->dwork);
			hrtimer_cancel(&memsic->mag_timer);

		if (mmc3524x_power_set(memsic, false))
			dev_warn(&memsic->i2c->dev, "Power off failed\n");
	} else {
		dev_warn(&memsic->i2c->dev,
				"ignore enable state change from %d to %d\n",
				memsic->enable, enable);
	}
	memsic->enable = enable;

exit:
	mutex_unlock(&memsic->ops_lock);
	return rc;
}

static int mmc3524x_set_poll_delay(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	ktime_t ktime;
	struct mmc3524x_data *memsic = container_of(sensors_cdev,
			struct mmc3524x_data, cdev);

	mutex_lock(&memsic->ops_lock);
	if (atomic_read(&memsic->poll_interval) != delay_msec) {
	    if (delay_msec < MMC3524X_MIN_DELAY) {
                delay_msec = MMC3524X_MIN_DELAY;
	  }
	  if (delay_msec > MMC3524X_MAX_DELAY) {
		delay_msec = MMC3524X_MAX_DELAY;
	  }
		atomic_read(&memsic->poll_interval) = delay_msec;
        }
	if (memsic->auto_report && !memsic->use_hrtimer)
	    mod_delayed_work(system_wq, &memsic->dwork, msecs_to_jiffies(delay_msec));
	else if (memsic->enable && memsic->use_hrtimer) {
	    ktime = ktime_set(0, atomic_read(&memsic->poll_interval) * NSEC_PER_MSEC);
	    hrtimer_start(&memsic->mag_timer, ktime, HRTIMER_MODE_REL);
        }
	mutex_unlock(&memsic->ops_lock);

	return 0;
}

static struct regmap_config mmc3524x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int mmc3524x_device_initial(void)
{
	int rc = -1;

#if 1
	// Do RESET operation
	rc = regmap_write(mmc3524x_data_struct->regmap, MMC3524X_REG_CTRL,
			MMC3524X_CTRL_REFILL);
	if (rc) {
		dev_err(&mmc3524x_data_struct->i2c->dev, "write reg %d failed at %d.(%d)\n",
				MMC3524X_REG_CTRL, __LINE__, rc);
		printk("write reg %d failed at %d.(%d)\n",
				MMC3524X_REG_CTRL, __LINE__, rc);
		//mutex_unlock(&ecompass_ioctl_lock);
		return -EFAULT;
	}

		/* Time from refill cap to SET */
	msleep(MMC3524X_DELAY_SET_MS);
#endif

	rc = regmap_write(mmc3524x_data_struct->regmap, MMC3524X_REG_CTRL,
			MMC3524X_CTRL_RESET);
	if (rc) {
		dev_err(&mmc3524x_data_struct->i2c->dev, "write reg %d failed at %d.(%d)\n",
				MMC3524X_REG_CTRL, __LINE__, rc);
		printk("write reg %d failed at %d.(%d)\n",
				MMC3524X_REG_CTRL, __LINE__, rc);
		//mutex_unlock(&ecompass_ioctl_lock);
		return -EFAULT;
	}
	msleep(1);

	// Do SET operation
	rc = regmap_write(mmc3524x_data_struct->regmap, MMC3524X_REG_CTRL,
			MMC3524X_CTRL_REFILL);
	if (rc) {
		dev_err(&mmc3524x_data_struct->i2c->dev, "write reg %d failed at %d.(%d)\n",
				MMC3524X_REG_CTRL, __LINE__, rc);
		printk("write reg %d failed at %d.(%d)\n",
				MMC3524X_REG_CTRL, __LINE__, rc);
		//mutex_unlock(&ecompass_ioctl_lock);
		return -EFAULT;
	}

		/* Time from refill cap to SET */
	msleep(MMC3524X_DELAY_SET_MS);

	rc = regmap_write(mmc3524x_data_struct->regmap, MMC3524X_REG_CTRL,
			MMC3524X_CTRL_SET);
	if (rc) {
		dev_err(&mmc3524x_data_struct->i2c->dev, "write reg %d failed at %d.(%d)\n",
				MMC3524X_REG_CTRL, __LINE__, rc);
		printk("write reg %d failed at %d.(%d)\n",
				MMC3524X_REG_CTRL, __LINE__, rc);
		//mutex_unlock(&ecompass_ioctl_lock);
		return -EFAULT;
	}
	return 0;
}

static int mmc3524x_open(struct inode *inode, struct file *file)
{

	return nonseekable_open(inode, file);
}

static int mmc3524x_release(struct inode *inode, struct file *file)
{

	return 0;
}

static long mmc3524x_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
/*
	int vec[3] = {0};
	int reg;
	short flag;
	short delay;
*/	
	void __user *pa = (void __user *)arg;
//	int __user *pa_i = (void __user *)arg;
	unsigned char data[16] = {0};
	unsigned char reg_addr;
	unsigned char reg_num;
	unsigned int reg_value;
	int rc = -1;
	printk("%s,%d\n",__FUNCTION__,__LINE__);
//	mutex_lock(&ecompass_ioctl_lock);
	switch (cmd) {
	case MMC3524X_IOC_SET:
		rc = regmap_write(mmc3524x_data_struct->regmap, MMC3524X_REG_CTRL,
				MMC3524X_CTRL_REFILL);
		if (rc) {
			dev_err(&mmc3524x_data_struct->i2c->dev, "write reg %d failed at %d.(%d)\n",
					MMC3524X_REG_CTRL, __LINE__, rc);
			printk("write reg %d failed at %d.(%d)\n",
					MMC3524X_REG_CTRL, __LINE__, rc);
//			mutex_unlock(&ecompass_ioctl_lock);
			return -EFAULT;
		}

		/* Time from refill cap to SET */
		msleep(MMC3524X_DELAY_SET_MS);

		rc = regmap_write(mmc3524x_data_struct->regmap, MMC3524X_REG_CTRL,
				MMC3524X_CTRL_SET);
		if (rc) {
			dev_err(&mmc3524x_data_struct->i2c->dev, "write reg %d failed at %d.(%d)\n",
					MMC3524X_REG_CTRL, __LINE__, rc);
			printk("write reg %d failed at %d.(%d)\n",
					MMC3524X_REG_CTRL, __LINE__, rc);
//			mutex_unlock(&ecompass_ioctl_lock);
			return -EFAULT;
		}
		dev_dbg(&mmc3524x_data_struct->i2c->dev, "mmc3524x reset is done\n");
		printk("mmc3524x reset is done\n");
		break;
		
	case MMC3524X_IOC_RESET:
		rc = regmap_write(mmc3524x_data_struct->regmap, MMC3524X_REG_CTRL,
				MMC3524X_CTRL_REFILL);
		if (rc) {
			dev_err(&mmc3524x_data_struct->i2c->dev, "write reg %d failed at %d.(%d)\n",
					MMC3524X_REG_CTRL, __LINE__, rc);
			printk("write reg %d failed at %d.(%d)\n",
					MMC3524X_REG_CTRL, __LINE__, rc);
//			mutex_unlock(&ecompass_ioctl_lock);
			return -EFAULT;
		}

		/* Time from refill cap to SET */
		msleep(MMC3524X_DELAY_SET_MS);

		rc = regmap_write(mmc3524x_data_struct->regmap, MMC3524X_REG_CTRL,
				MMC3524X_CTRL_RESET);
		if (rc) {
			dev_err(&mmc3524x_data_struct->i2c->dev, "write reg %d failed at %d.(%d)\n",
					MMC3524X_REG_CTRL, __LINE__, rc);
			printk("write reg %d failed at %d.(%d)\n",
					MMC3524X_REG_CTRL, __LINE__, rc);
//			mutex_unlock(&ecompass_ioctl_lock);
			return -EFAULT;
		}
		dev_dbg(&mmc3524x_data_struct->i2c->dev, "mmc3524x reset is done\n");
		printk("mmc3524x reset is done\n");
		break;
		
 	case MMC3524X_IOC_READ_REG:
		if (copy_from_user(&reg_addr, pa, sizeof(reg_addr)))
			return -EFAULT;
		rc = regmap_read(mmc3524x_data_struct->regmap, reg_addr, &reg_value);
		if (rc) {
			dev_err(&mmc3524x_data_struct->i2c->dev, "read reg %d failed at %d.(%d)\n",
					reg_addr, __LINE__, rc);
		}	
		printk("<7>planar Read register No. 0x%0x\n",reg_addr);
		if (copy_to_user(pa, &reg_value, sizeof(reg_value))) {
//			mutex_unlock(&ecompass_lock);
			return -EFAULT;
		}		
		break;       
		
 	case MMC3524X_IOC_WRITE_REG:
		if (copy_from_user(&data, pa, sizeof(data)))
			return -EFAULT;
			
		rc = regmap_write(mmc3524x_data_struct->regmap, data[1],
				data[0]);	
		if (rc) {
			dev_err(&mmc3524x_data_struct->i2c->dev, "write reg %d failed at %d.(%d)\n",
					data[1], __LINE__, rc);
			printk("write reg %d failed at %d.(%d)\n",
					data[1], __LINE__, rc);
//			mutex_unlock(&ecompass_ioctl_lock);
			return -EFAULT;
		}
		printk("<7>planar Write '0x%0x' to  register No. 0x%0x\n", data[0], data[1]);
		break;   
		
 	case MMC3524X_IOC_READ_REGS:
		if (copy_from_user(&data, pa, sizeof(data)))
			return -EFAULT;
		reg_addr = data[0];
		reg_num = data[1];
		printk("<7> planar Read %d registers from 0x%0x\n", reg_num, reg_addr);	
		rc = regmap_bulk_read(mmc3524x_data_struct->regmap, reg_addr, data, reg_num);
		if (rc) {
			dev_err(&mmc3524x_data_struct->i2c->dev, "read reg %d failed at %d.(%d)\n",
					reg_addr, __LINE__, rc);
		}
		
		printk("<7> data: %x %x %x \n%x %x %x\n", data[0], data[1], data[2], data[3], data[4], data[5]);	
		if (copy_to_user(pa, data, sizeof(data))) {
	//		mutex_unlock(&ecompass_lock);
			return -EFAULT;
		}		
		break;    		
		
	default:
		break;
	}

//	mutex_unlock(&ecompass_ioctl_lock);

	return 0;

}
#ifdef CONFIG_COMPAT
static long mmc3524x_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
/*
	int vec[3] = {0};
	int reg;
	short flag;
	short delay;
*/	
	void __user *pa = (void __user *)arg;
//	int __user *pa_i = (void __user *)arg;
	unsigned char data[16] = {0};
	unsigned char reg_addr;
	unsigned char reg_num;
	unsigned int reg_value;
	int rc = -1;

//	mutex_lock(&ecompass_ioctl_lock);
	switch (cmd) {
	case MMC3524X_IOC_SET:
		rc = regmap_write(mmc3524x_data_struct->regmap, MMC3524X_REG_CTRL,
				MMC3524X_CTRL_REFILL);
		if (rc) {
			dev_err(&mmc3524x_data_struct->i2c->dev, "write reg %d failed at %d.(%d)\n",
					MMC3524X_REG_CTRL, __LINE__, rc);
			printk("write reg %d failed at %d.(%d)\n",
					MMC3524X_REG_CTRL, __LINE__, rc);
//			mutex_unlock(&ecompass_ioctl_lock);
			return -EFAULT;
		}

		/* Time from refill cap to SET */
		msleep(MMC3524X_DELAY_SET_MS);

		rc = regmap_write(mmc3524x_data_struct->regmap, MMC3524X_REG_CTRL,
				MMC3524X_CTRL_SET);
		if (rc) {
			dev_err(&mmc3524x_data_struct->i2c->dev, "write reg %d failed at %d.(%d)\n",
					MMC3524X_REG_CTRL, __LINE__, rc);
			printk("write reg %d failed at %d.(%d)\n",
					MMC3524X_REG_CTRL, __LINE__, rc);
//			mutex_unlock(&ecompass_ioctl_lock);
			return -EFAULT;
		}
		dev_dbg(&mmc3524x_data_struct->i2c->dev, "mmc3524x reset is done\n");
		printk("mmc3524x reset is done\n");
		break;
		
	case MMC3524X_IOC_RESET:
		rc = regmap_write(mmc3524x_data_struct->regmap, MMC3524X_REG_CTRL,
				MMC3524X_CTRL_REFILL);
		if (rc) {
			dev_err(&mmc3524x_data_struct->i2c->dev, "write reg %d failed at %d.(%d)\n",
					MMC3524X_REG_CTRL, __LINE__, rc);
			printk("write reg %d failed at %d.(%d)\n",
					MMC3524X_REG_CTRL, __LINE__, rc);
//			mutex_unlock(&ecompass_ioctl_lock);
			return -EFAULT;
		}

		/* Time from refill cap to SET */
		msleep(MMC3524X_DELAY_SET_MS);

		rc = regmap_write(mmc3524x_data_struct->regmap, MMC3524X_REG_CTRL,
				MMC3524X_CTRL_RESET);
		if (rc) {
			dev_err(&mmc3524x_data_struct->i2c->dev, "write reg %d failed at %d.(%d)\n",
					MMC3524X_REG_CTRL, __LINE__, rc);
			printk("write reg %d failed at %d.(%d)\n",
					MMC3524X_REG_CTRL, __LINE__, rc);
//			mutex_unlock(&ecompass_ioctl_lock);
			return -EFAULT;
		}
		dev_dbg(&mmc3524x_data_struct->i2c->dev, "mmc3524x reset is done\n");
		printk("mmc3524x reset is done\n");
		break;
		
 	case MMC3524X_IOC_READ_REG:
		if (copy_from_user(&reg_addr, pa, sizeof(reg_addr)))
			return -EFAULT;
		rc = regmap_read(mmc3524x_data_struct->regmap, reg_addr, &reg_value);
		if (rc) {
			dev_err(&mmc3524x_data_struct->i2c->dev, "read reg %d failed at %d.(%d)\n",
					reg_addr, __LINE__, rc);
		}	
		printk("<7>planar Read register No. 0x%0x\n",reg_addr);
		if (copy_to_user(pa, &reg_value, sizeof(reg_value))) {
//			mutex_unlock(&ecompass_lock);
			return -EFAULT;
		}		
		break;       
		
 	case MMC3524X_IOC_WRITE_REG:
		if (copy_from_user(&data, pa, sizeof(data)))
			return -EFAULT;
			
		rc = regmap_write(mmc3524x_data_struct->regmap, data[1],
				data[0]);	
		if (rc) {
			dev_err(&mmc3524x_data_struct->i2c->dev, "write reg %d failed at %d.(%d)\n",
					data[1], __LINE__, rc);
			printk("write reg %d failed at %d.(%d)\n",
					data[1], __LINE__, rc);
//			mutex_unlock(&ecompass_ioctl_lock);
			return -EFAULT;
		}
		printk("<7>planar Write '0x%0x' to  register No. 0x%0x\n", data[0], data[1]);
		break;   
		
 	case MMC3524X_IOC_READ_REGS:
		if (copy_from_user(&data, pa, sizeof(data)))
			return -EFAULT;
		reg_addr = data[0];
		reg_num = data[1];
		printk("<7> planar Read %d registers from 0x%0x\n", reg_num, reg_addr);	
		rc = regmap_bulk_read(mmc3524x_data_struct->regmap, reg_addr, data, reg_num);
		if (rc) {
			dev_err(&mmc3524x_data_struct->i2c->dev, "read reg %d failed at %d.(%d)\n",
					reg_addr, __LINE__, rc);
		}
		
		printk("<7> data: %x %x %x \n%x %x %x\n", data[0], data[1], data[2], data[3], data[4], data[5]);	
		if (copy_to_user(pa, data, sizeof(data))) {
	//		mutex_unlock(&ecompass_lock);
			return -EFAULT;
		}		
		break;    		
		
	default:
		break;
	}

//	mutex_unlock(&ecompass_ioctl_lock);

	return 0;

}

#endif

static struct file_operations mmc3524x_fops = {
	.owner		= THIS_MODULE,
	.open		= mmc3524x_open,
	.release	= mmc3524x_release,
	.unlocked_ioctl = mmc3524x_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = mmc3524x_compat_ioctl,
#endif
};

static struct miscdevice mmc3524x_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MMC3524X_I2C_NAME,
	.fops = &mmc3524x_fops,
};

static ssize_t mmc3524x_otp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	
	char data[10] = {0};
	unsigned char reg_addr;
	unsigned char reg_num;
	int rc = -1;
	
	reg_addr = 0x1b;
	reg_num = 4;
	rc = regmap_bulk_read(mmc3524x_data_struct->regmap, reg_addr, data, reg_num);
	if (rc) {
			dev_err(&mmc3524x_data_struct->i2c->dev, "read reg %d failed at %d.(%d)\n",
					reg_addr, __LINE__, rc);
	}
	printk("otp data: %x %x %x %x\n", data[0], data[1], data[2], data[3]);	

	return sprintf(buf, "%x,%x,%x,%x\n", data[0],data[1],data[2],data[3]);
}

static ssize_t mmc3524x_value_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	//s8 *tmp;
	struct mmc3524x_vec vec;
	//struct mmc3524x_vec report;
	struct mmc3524x_data *memsic = dev_get_drvdata(dev);

	vec.x = vec.y = vec.z = 0;

	ret = mmc3524x_read_xyz(memsic, &vec);
	if (ret) {
		dev_warn(&memsic->i2c->dev, "read xyz failed\n");
	}
	
	/*
	tmp = &mmc3524x_rotation_matrix[memsic->dir][0];
	report.x = tmp[0] * vec.x + tmp[1] * vec.y + tmp[2] * vec.z;
	report.y = tmp[3] * vec.x + tmp[4] * vec.y + tmp[5] * vec.z;
	report.z = tmp[6] * vec.x + tmp[7] * vec.y + tmp[8] * vec.z;	
	*/

	return sprintf(buf, "%d %d %d\n", vec.x,vec.y,vec.z);
}

// add to create sysfs file node 
static DEVICE_ATTR(otp, 0444, mmc3524x_otp_show, NULL);
static DEVICE_ATTR(value, 0444, mmc3524x_value_show, NULL);

static struct attribute *mmc3524x_attributes[] = {
		&dev_attr_otp.attr,
		&dev_attr_value.attr,
		NULL,
};

static const struct attribute_group mmc3524x_attr_group = {
		.attrs = mmc3524x_attributes,
};


static int mmc3524x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int res = 0;
	struct mmc3524x_data *memsic;

	printk("[%s]: Enter!\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("mmc3524x i2c functionality check failed.\n");
		res = -ENODEV;
		goto out;
	}

	memsic = kzalloc(sizeof(struct mmc3524x_data),GFP_KERNEL);
	if (!memsic) {
		dev_err(&client->dev, "memory allocation failed.\n");
		res = -ENOMEM;
		goto out;
	}

	mmc3524x_data_struct = memsic;
	
	if (client->dev.of_node) {
		res = mmc3524x_parse_dt(client, memsic);
		if (res) {
			dev_err(&client->dev,
				"Unable to parse platform data.(%d)", res);
			goto out;
		}
	} else {
		memsic->dir = 0;
		memsic->auto_report = 1;
	}
	memsic->power_enabled = 0; // add
	memsic->i2c = client;
	dev_set_drvdata(&client->dev, memsic);

	mutex_init(&memsic->ecompass_lock);
	mutex_init(&memsic->ops_lock);

	res = mmc3524x_power_init(memsic);
	if (res) {
		dev_err(&client->dev, "Power init mmc3524x failed\n");
		goto out;
	}
	res = mmc3524x_power_set(memsic,1);
	if (res) {
		dev_err(&client->dev, "Power up mmc3524x failed\n");
		goto out_check_device;
	}
	memsic->regmap = devm_regmap_init_i2c(client, &mmc3524x_regmap_config);
	if (IS_ERR(memsic->regmap)) {
		dev_err(&client->dev, "Init regmap failed.(%ld)",
				PTR_ERR(memsic->regmap));
		res = PTR_ERR(memsic->regmap);
		goto out_power_set;
	}
	
	res = mmc3524x_check_device(client);
	if (res) {
		dev_err(&client->dev, "Check device failed\n");
		goto out_power_set;
	}

	res = mmc3524x_device_initial();
	if (res) {
		pr_err("mmc3524x_device initial failed\n");
	}

	//if(memsic_sensor==MMC35240)
        res = ReadMMC3524x_OTP(memsic);
	   if (res) {
		   dev_info(&client->dev, "read mmc3524x otp  failed\n");
			//default matrix
		   otpMatrix[0] = 1000;
		   otpMatrix[1] = 1000;
		   otpMatrix[2] = 1350;

	   	}
	   
	res = mmc3524x_init_input(&memsic->input);
	if (res) {
		dev_err(&client->dev, "init input device failed\n");
		goto out_init_input;
	}


	if (memsic->auto_report && !memsic->use_hrtimer) {
		dev_info(&client->dev, "auto report is enabled\n");
		INIT_DELAYED_WORK(&memsic->dwork, mmc3524x_work_func);
		memsic->data_wq =	create_freezable_workqueue("mmc3524x_data_work");
		if (!memsic->data_wq) {
			dev_err(&client->dev, "Cannot create workqueue.\n");
			goto out_register_classdev;
		}
	} else {
		hrtimer_init(&memsic->mag_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		memsic->mag_timer.function = mag_timer_handle;
		memsic->data_wq = alloc_workqueue("mmc3524x_data_work",WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
		INIT_DELAYED_WORK(&memsic->dwork, mmc3524x_poll_work);  
	}

	memsic->cdev = sensors_cdev;
	memsic->cdev.sensors_enable = mmc3524x_set_enable;
	memsic->cdev.sensors_poll_delay = mmc3524x_set_poll_delay;
	atomic_read(&memsic->poll_interval) = MMC3524X_DEFAULT_INTERVAL_MS;
	res = sensors_classdev_register(&memsic->input->dev, &memsic->cdev);
	if (res) {
		dev_err(&client->dev, "sensors class register failed.\n");
		goto out_register_classdev;
	}

	/***** misc ****/
	res = misc_register(&mmc3524x_device);
	if (res) {
		pr_err("%s: mmc3524x_device register failed\n", __FUNCTION__);
		goto out_class_unregister;
	}


	
	/* create sysfs group */
	res = sysfs_create_group(&client->dev.kobj, &mmc3524x_attr_group);
	if (res){
		res = -EROFS;
		dev_err(&client->dev,"Unable to creat sysfs group\n");
		goto out_deregister;
	}

	mmc3524x_power_set(memsic, false);
	//dev_info(&client->dev, "mmc3524x successfully probed\n");
	printk("[%s]: successfully probed!\n", __func__);

	return 0;

out_deregister:
	misc_deregister(&mmc3524x_device);
out_class_unregister:
	sensors_classdev_unregister(&memsic->cdev);
out_register_classdev:
	destroy_workqueue(memsic->data_wq);
	input_unregister_device(memsic->input);
out_init_input:
out_power_set:
	mmc3524x_power_set(memsic, false);
out_check_device:
	mmc3524x_power_deinit(memsic);
out:
	printk("[%s]: probed fail !\n", __func__);
	return res;
}

static int mmc3524x_remove(struct i2c_client *client)
{
	struct mmc3524x_data *memsic = dev_get_drvdata(&client->dev);

	if (memsic->use_hrtimer) {
	  hrtimer_cancel(&memsic->mag_timer);
	  destroy_workqueue(memsic->data_wq);
	} else {
		destroy_workqueue(memsic->data_wq);
	}
	sensors_classdev_unregister(&memsic->cdev);
	misc_deregister(&mmc3524x_device);
	mmc3524x_power_deinit(memsic);

	if (memsic->input)
		input_unregister_device(memsic->input);

	sysfs_remove_group(&client->dev.kobj, &mmc3524x_attr_group);	

	return 0;
}

static int mmc3524x_suspend(struct device *dev)
{
	int res = 0;
	struct mmc3524x_data *memsic = dev_get_drvdata(dev);

	dev_dbg(dev, "suspended\n");

	if (memsic->enable) {
		if (memsic->auto_report & !memsic->use_hrtimer)
			cancel_delayed_work_sync(&memsic->dwork);
		else if (memsic->use_hrtimer)
			hrtimer_cancel(&memsic->mag_timer);
		res = mmc3524x_power_set(memsic, false);
		if (res) {
			dev_err(dev, "failed to suspend mmc3524x\n");
			goto exit;
		}
	}
exit:
	return res;
}

static int mmc3524x_resume(struct device *dev)
{
	int res = 0;
	struct mmc3524x_data *memsic = dev_get_drvdata(dev);
	ktime_t ktime;

	dev_dbg(dev, "resumed\n");

	if (memsic->enable) {
		res = mmc3524x_power_set(memsic, true);
		if (res) {
			dev_err(&memsic->i2c->dev, "Power enable failed\n");
			goto exit;
		}

		if (memsic->auto_report & !memsic->use_hrtimer) {
			queue_delayed_work(memsic->data_wq, &memsic->dwork,	msecs_to_jiffies(atomic_read(&memsic->poll_interval)));
		} else if (memsic->use_hrtimer) {
			queue_delayed_work(memsic->data_wq, &memsic->dwork,	msecs_to_jiffies(atomic_read(&memsic->poll_interval)));
			ktime = ktime_set(0, atomic_read(&memsic->poll_interval) * NSEC_PER_MSEC);
			hrtimer_start(&memsic->mag_timer,	ktime, HRTIMER_MODE_REL);
		}
	}

exit:
	return res;
}

static const struct i2c_device_id mmc3524x_id[] = {
	{ MMC3524X_I2C_NAME, 0 },
	{ }
};

static struct of_device_id mmc3524x_match_table[] = {
	{ .compatible = "memsic,mmc3524x", },
	{ },
};

static const struct dev_pm_ops mmc3524x_pm_ops = {
	.suspend = mmc3524x_suspend,
	.resume = mmc3524x_resume,
};

static struct i2c_driver mmc3524x_driver = {
	.probe 		= mmc3524x_probe,
	.remove 	= mmc3524x_remove,
	.id_table	= mmc3524x_id,
	.driver 	= {
		.owner	= THIS_MODULE,
		.name	= MMC3524X_I2C_NAME,
		.of_match_table = mmc3524x_match_table,
		.pm = &mmc3524x_pm_ops,
	},
};

static int __init mmc3524x_init(void)
{
	printk(KERN_ERR "mmc3524x_init");
	return i2c_add_driver(&mmc3524x_driver);
}

static void __exit mmc3524x_exit(void)
{
	i2c_del_driver(&mmc3524x_driver);
}

//module_i2c_driver(mmc3524x_driver);
late_initcall(mmc3524x_init);
module_exit(mmc3524x_exit);

MODULE_DESCRIPTION("MEMSIC MMC3524X Magnetic Sensor Driver");
MODULE_LICENSE("GPL");

