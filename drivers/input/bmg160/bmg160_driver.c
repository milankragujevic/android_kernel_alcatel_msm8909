/*!
 * @section LICENSE
 * $license$
 *
 * @filename $filename$
 * @date     2014/04/22 13:44
 * @id       $id$
 * @version  1.5.9
 *
 * @brief    BMG160 Linux Driver
 */
#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/string.h>
#else
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#endif

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>

//[Feature]-BEGIN by TCTSZ.yongzhong.cheng@tcl.com,2015/9/16,for 538697:bmg160 getdata ok
#include <linux/sensors.h>

#define POWER_REGULATOR_BMG
#define PINCTRL_BMG

#ifdef POWER_REGULATOR_BMG
#include <linux/regulator/consumer.h>
#define BMG_VDD_MIN_UV	2600000
#define BMG_VDD_MAX_UV	3300000
#define BMG_VIO_MIN_UV	1800000
#define BMG_VIO_MAX_UV	1800000
#endif
//[Feature]-END by TCTSZ.yongzhong.cheng@tcl.com, 2015/9/16,for 538697

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "bmg160.h"
//#include "bs_log.h"

/* sensor specific */
#define SENSOR_NAME "bmg160"

#define SENSOR_CHIP_ID_BMG (0x0f)
#define CHECK_CHIP_ID_TIME_MAX   5

#define BMG_REG_NAME(name) BMG160_##name
#define BMG_VAL_NAME(name) BMG160_##name
#define BMG_CALL_API(name) bmg160_##name

#define BMG_I2C_WRITE_DELAY_TIME 1

/* generic */
#define BMG_MAX_RETRY_I2C_XFER (100)
#define BMG_MAX_RETRY_WAKEUP (5)
#define BMG_MAX_RETRY_WAIT_DRDY (100)

#define BMG_DELAY_MIN (10)
#define BMG_DELAY_DEFAULT (100)

#define BMG_VALUE_MAX (32767)
#define BMG_VALUE_MIN (-32768)

#define BYTES_PER_LINE (16)

#define BMG_SELF_TEST 0

#define BMG_SOFT_RESET_VALUE                0xB6


#ifdef BMG_USE_FIFO
#define MAX_FIFO_F_LEVEL 100
#define MAX_FIFO_F_BYTES 8
#define BMG160_FIFO_DAT_SEL_X                     1
#define BMG160_FIFO_DAT_SEL_Y                     2
#define BMG160_FIFO_DAT_SEL_Z                     3
#endif

/*!
 * @brief:BMI058 feature
 *  macro definition
*/
#ifdef CONFIG_SENSORS_BMI058
/*! BMI058 X AXIS definition*/
#define BMI058_X_AXIS	BMG160_Y_AXIS
/*! BMI058 Y AXIS definition*/
#define BMI058_Y_AXIS	BMG160_X_AXIS

#define C_BMI058_One_U8X	1
#define C_BMI058_Two_U8X	2
#endif

/*! Bosch sensor unknown place*/
#define BOSCH_SENSOR_PLACE_UNKNOWN (-1)
/*! Bosch sensor remapping table size P0~P7*/
#define MAX_AXIS_REMAP_TAB_SZ 8


struct bosch_sensor_specific {
	char *name;
	/* 0 to 7 */
	int place;
	int irq;
	int (*irq_gpio_cfg)(void);
};

//[Feature]-BEGIN by TCTSZ.yongzhong.cheng@tcl.com,2015/9/16,for 538697:bmg160 getdata ok
extern  int bmg_enable_set(struct sensors_classdev *sensors_cdev,unsigned int enabled);
extern  int bmg_poll_delay_set(struct sensors_classdev *sensors_cdev,unsigned int delay_msec);

static struct sensors_classdev sensors_bmg_cdev = {
	.name = SENSOR_NAME,
	.vendor = "BOSCH",
	.version = 1,
	.handle = SENSORS_GYROSCOPE_HANDLE,
	.type = SENSOR_TYPE_GYROSCOPE,
	.max_range = "35.0",
	.resolution = "1.0",
	.sensor_power = "0.2",
	.min_delay = 10000,//CTS request less than 10000
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = bmg_enable_set,
	.sensors_poll_delay = bmg_poll_delay_set,
};
//[Feature]-END by TCTSZ.yongzhong.cheng@tcl.com, 2015/9/16,for 538697

/*!
 * we use a typedef to hide the detail,
 * because this type might be changed
 */
struct bosch_sensor_axis_remap {
	/* src means which source will be mapped to target x, y, z axis */
	/* if an target OS axis is remapped from (-)x,
	 * src is 0, sign_* is (-)1 */
	/* if an target OS axis is remapped from (-)y,
	 * src is 1, sign_* is (-)1 */
	/* if an target OS axis is remapped from (-)z,
	 * src is 2, sign_* is (-)1 */
	int src_x:3;
	int src_y:3;
	int src_z:3;

	int sign_x:2;
	int sign_y:2;
	int sign_z:2;
};


struct bosch_sensor_data {
	union {
		int16_t v[3];
		struct {
			int16_t x;
			int16_t y;
			int16_t z;
		};
	};
};

struct bmg_client_data {
	struct bmg160_t device;
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend_handler;
#endif

	atomic_t delay;

	struct bmg160_data_t value;
	u8 enable:1;
	unsigned int fifo_count;
	unsigned char fifo_datasel;

	/* controls not only reg, but also workqueue */
	struct mutex mutex_op_mode;
	struct mutex mutex_enable;
	struct bosch_sensor_specific *bst_pd;

//[Feature]-BEGIN by TCTSZ.yongzhong.cheng@tcl.com,2015/9/16,for 538697:bmg160 getdata ok	
	struct sensors_classdev bmg_cdev;
#ifdef POWER_REGULATOR_BMG
	struct regulator *vdd;
	struct regulator *vio;
	bool power_enabled; 
	bool bmg_enabled;
#endif
//[Feature]-END by TCTSZ.yongzhong.cheng@tcl.com, 2015/9/16,for 538697
#ifdef PINCTRL_BMG
struct pinctrl *pinctrl;

#ifdef JRD_PROJECT_PIXI4554G
struct pinctrl_state *bmi_int11_init;
struct pinctrl_state *bmi_int11_sleep;
struct pinctrl_state *bmi_int91_init;
struct pinctrl_state *bmi_int91_sleep;
#else
struct pinctrl_state *bmg_int98_init;
struct pinctrl_state *bmg_int98_sleep;
#endif
#endif

u8 on_before_suspend;
struct work_struct poll_work;
struct hrtimer	poll_timer;
struct workqueue_struct *data_wq;

};

static struct i2c_client *bmg_client;
/* i2c operation for API */
static void bmg_i2c_delay(BMG160_U16 msec);
static int bmg_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len);
static int bmg_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len);

static void bmg_dump_reg(struct i2c_client *client);
static int bmg_check_chip_id(struct i2c_client *client);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bmg_early_suspend(struct early_suspend *handler);
static void bmg_late_resume(struct early_suspend *handler);
#endif
/*!
* BMG160 sensor remapping function
* need to give some parameter in BSP files first.
*/
static const struct bosch_sensor_axis_remap
	bst_axis_remap_tab_dft[MAX_AXIS_REMAP_TAB_SZ] = {
	/* src_x src_y src_z  sign_x  sign_y  sign_z */
	{  0,	 1,    2,	  1,	  1,	  1 }, /* P0 */
	{  1,	 0,    2,	  1,	 -1,	  1 }, /* P1 */
	{  0,	 1,    2,	 -1,	 -1,	  1 }, /* P2 */
	{  1,	 0,    2,	 -1,	  1,	  1 }, /* P3 */

	{  0,	 1,    2,	 -1,	  1,	 -1 }, /* P4 */
	{  1,	 0,    2,	 -1,	 -1,	 -1 }, /* P5 */
	{  0,	 1,    2,	  1,	 -1,	 -1 }, /* P6 */
	{  1,	 0,    2,	  1,	  1,	 -1 }, /* P7 */
};

static void bst_remap_sensor_data(struct bosch_sensor_data *data,
			const struct bosch_sensor_axis_remap *remap)
{
	struct bosch_sensor_data tmp;

	tmp.x = data->v[remap->src_x] * remap->sign_x;
	tmp.y = data->v[remap->src_y] * remap->sign_y;
	tmp.z = data->v[remap->src_z] * remap->sign_z;

	memcpy(data, &tmp, sizeof(*data));
}

static void bst_remap_sensor_data_dft_tab(struct bosch_sensor_data *data,
			int place)
{
/* sensor with place 0 needs not to be remapped */
	if ((place <= 0) || (place >= MAX_AXIS_REMAP_TAB_SZ))
		return;
	bst_remap_sensor_data(data, &bst_axis_remap_tab_dft[place]);
}

static void bmg160_remap_sensor_data(struct bmg160_data_t *val,
		struct bmg_client_data *client_data)
{
	struct bosch_sensor_data bsd;
	int place;

	if ((NULL == client_data->bst_pd) || (BOSCH_SENSOR_PLACE_UNKNOWN
			 == client_data->bst_pd->place))
		place = BOSCH_SENSOR_PLACE_UNKNOWN;
	else
		place = client_data->bst_pd->place;

#ifdef CONFIG_SENSORS_BMI058
/*x,y need to be invesed becase of HW Register for BMI058*/
	bsd.y = val->datax;
	bsd.x = val->datay;
	bsd.z = val->dataz;
#else
	bsd.x = val->datax;
	bsd.y = val->datay;
	bsd.z = val->dataz;
#endif

	bst_remap_sensor_data_dft_tab(&bsd, place);

	val->datax = bsd.x;
	val->datay = bsd.y;
	val->dataz = bsd.z;

}

static int bmg_check_chip_id(struct i2c_client *client)
{
	int err = -1;
	u8 chip_id = 0;
	u8 read_count = 0;

	while (read_count++ < CHECK_CHIP_ID_TIME_MAX) {
		bmg_i2c_read(client, BMG_REG_NAME(CHIP_ID_ADDR), &chip_id, 1);
		printk("read chip id result: %#x\n", chip_id);

		if ((chip_id & 0xff) != SENSOR_CHIP_ID_BMG) {
			mdelay(1);
		} else {
			err = 0;
			break;
		}
	}
	return err;
}

static void bmg_i2c_delay(BMG160_U16 msec)
{
	mdelay(msec);
}

static void bmg_dump_reg(struct i2c_client *client)
{
	int i;
	u8 dbg_buf[64];
	u8 dbg_buf_str[64 * 3 + 1] = "";

	for (i = 0; i < BYTES_PER_LINE; i++) {
		dbg_buf[i] = i;
		sprintf(dbg_buf_str + i * 3, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	dev_dbg(&client->dev, "%s\n", dbg_buf_str);

	bmg_i2c_read(client, BMG_REG_NAME(CHIP_ID_ADDR), dbg_buf, 64);
	for (i = 0; i < 64; i++) {
		sprintf(dbg_buf_str + i * 3, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	dev_dbg(&client->dev, "%s\n", dbg_buf_str);
}

/*i2c read routine for API*/
static int bmg_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
#if !defined BMG_USE_BASIC_I2C_FUNC
	s32 dummy;
	if (NULL == client)
		return -1;

	while (0 != len--) {
#ifdef BMG_SMBUS
		dummy = i2c_smbus_read_byte_data(client, reg_addr);
		if (dummy < 0) {
			dev_err(&client->dev, "i2c bus read error");
			return -1;
		}
		*data = (u8)(dummy & 0xff);
#else
		dummy = i2c_master_send(client, (char *)&reg_addr, 1);
		if (dummy < 0)
			return -1;

		dummy = i2c_master_recv(client, (char *)data, 1);
		if (dummy < 0)
			return -1;
#endif
		reg_addr++;
		data++;
	}
	return 0;
#else
	int retry;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg_addr,
		},

		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	for (retry = 0; retry < BMG_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			mdelay(BMG_I2C_WRITE_DELAY_TIME);
	}

	if (BMG_MAX_RETRY_I2C_XFER <= retry) {
		dev_err(&client->dev, "I2C xfer error");
		return -EIO;
	}

	return 0;
#endif
}

#ifdef BMG_USE_FIFO
static int bmg_i2c_burst_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u16 len)
{
	int retry;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg_addr,
		},

		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	for (retry = 0; retry < BMG_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			mdelay(BMG_I2C_WRITE_DELAY_TIME);
	}

	if (BMG_MAX_RETRY_I2C_XFER <= retry) {
		dev_err(&client->dev, "I2C xfer error");
		return -EIO;
	}

	return 0;
}
#endif

/*i2c write routine for */
static int bmg_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
#if !defined BMG_USE_BASIC_I2C_FUNC
	s32 dummy;

#ifndef BMG_SMBUS
	u8 buffer[2];
#endif

	if (NULL == client)
		return -EPERM;

	while (0 != len--) {
#ifdef BMG_SMBUS
		dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
#else
		buffer[0] = reg_addr;
		buffer[1] = *data;
		dummy = i2c_master_send(client, (char *)buffer, 2);
#endif
		reg_addr++;
		data++;
		if (dummy < 0) {
			dev_err(&client->dev, "error writing i2c bus");
			return -EPERM;
		}

	}
	return 0;
#else
	u8 buffer[2];
	int retry;
	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 2,
		 .buf = buffer,
		 },
	};

	while (0 != len--) {
		buffer[0] = reg_addr;
		buffer[1] = *data;
		for (retry = 0; retry < BMG_MAX_RETRY_I2C_XFER; retry++) {
			if (i2c_transfer(client->adapter, msg,
						ARRAY_SIZE(msg)) > 0) {
				break;
			} else {
				mdelay(BMG_I2C_WRITE_DELAY_TIME);
			}
		}
		if (BMG_MAX_RETRY_I2C_XFER <= retry) {
			dev_err(&client->dev, "I2C xfer error");
			return -EIO;
		}
		reg_addr++;
		data++;
	}

	return 0;
#endif
}

static int bmg_i2c_read_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	int err;
	err = bmg_i2c_read(bmg_client, reg_addr, data, len);
	return err;
}

static int bmg_i2c_write_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	int err;
	err = bmg_i2c_write(bmg_client, reg_addr, data, len);
	return err;
}

#define BMG_DEBUG
//#define BMG160_USE_DELAY_WORK

#ifdef BMG160_USE_DELAY_WORK
static void bmg_work_func(struct work_struct *work)
{
	ktime_t ts;
	struct bmg160_data_t gyro_data;
	struct bmg_client_data *client_data =
		container_of((struct delayed_work *)work,
			struct bmg_client_data, work);

	unsigned long delay =
		msecs_to_jiffies(atomic_read(&client_data->delay));
	ts = ktime_get_boottime();

	BMG_CALL_API(get_dataXYZ)(&gyro_data);
	bmg160_remap_sensor_data(&gyro_data, client_data);

//printk("x=%d,y=%d,z=%d,delay=%d\n",gyro_data.datax,gyro_data.datay,gyro_data.dataz,atomic_read(&client_data->delay));
	input_report_abs(client_data->input, ABS_RX, gyro_data.datax);
	input_report_abs(client_data->input, ABS_RY, gyro_data.datay);
	input_report_abs(client_data->input, ABS_RZ, gyro_data.dataz);
	input_event(client_data->input,EV_SYN,SYN_TIME_SEC,ktime_to_timespec(ts).tv_sec);
	input_event(client_data->input,EV_SYN,SYN_TIME_NSEC,ktime_to_timespec(ts).tv_nsec);

	input_sync(client_data->input);

	schedule_delayed_work(&client_data->work, delay);
}
#else
struct bmg160_data_t gyro_data;
static void bmg_work_func_getdata(struct work_struct *work)
{
	struct bmg_client_data *client_data =container_of(work,struct bmg_client_data, poll_work);

	mutex_lock(&client_data->mutex_enable);
	BMG_CALL_API(get_dataXYZ)(&gyro_data);
	bmg160_remap_sensor_data(&gyro_data, client_data);
	mutex_unlock(&client_data->mutex_enable);
}

static enum hrtimer_restart poll_timer_report(struct hrtimer *timer)
{
	ktime_t ts;
	struct bmg_client_data *client_data= container_of(timer, struct bmg_client_data, poll_timer);;
	queue_work(client_data->data_wq, &client_data->poll_work);
	ts = ktime_get_boottime();

	//printk("x=%d,y=%d,z=%d,delay=%d\n",gyro_data.datax,gyro_data.datay,gyro_data.dataz,atomic_read(&client_data->delay));
	input_report_abs(client_data->input, ABS_RX, gyro_data.datax);
	input_report_abs(client_data->input, ABS_RY, gyro_data.datay);
	input_report_abs(client_data->input, ABS_RZ, gyro_data.dataz);
	input_event(client_data->input,EV_SYN,SYN_TIME_SEC,ktime_to_timespec(ts).tv_sec);
	input_event(client_data->input,EV_SYN,SYN_TIME_NSEC,ktime_to_timespec(ts).tv_nsec);	
	input_sync(client_data->input);

	hrtimer_forward_now(&client_data->poll_timer,
	ns_to_ktime(atomic_read(&client_data->delay) * 1000000UL));	
	return HRTIMER_RESTART;
}
#endif

static int bmg_set_soft_reset(struct i2c_client *client)
{
	int err = 0;
	unsigned char data = BMG_SOFT_RESET_VALUE;
	err = bmg_i2c_write(client, BMG160_BGW_SOFTRESET_ADDR, &data, 1);
	return err;
}

static ssize_t bmg_show_chip_id(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", SENSOR_CHIP_ID_BMG);
}

static ssize_t bmg_show_op_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	u8 op_mode = 0xff;

	mutex_lock(&client_data->mutex_op_mode);
	BMG_CALL_API(get_mode)(&op_mode);
	mutex_unlock(&client_data->mutex_op_mode);

	ret = sprintf(buf, "%d\n", op_mode);

	return ret;
}

static ssize_t bmg_store_op_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	long op_mode;

	err = kstrtoul(buf, 10, &op_mode);
	if (err)
		return err;

	mutex_lock(&client_data->mutex_op_mode);

	err = BMG_CALL_API(set_mode)(op_mode);

	mutex_unlock(&client_data->mutex_op_mode);

	if (err)
		return err;
	else
		return count;
}



static ssize_t bmg_show_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	int count;

	struct bmg160_data_t value_data;
	BMG_CALL_API(get_dataXYZ)(&value_data);
	/*BMG160 sensor raw data remapping*/
	bmg160_remap_sensor_data(&value_data, client_data);

	count = sprintf(buf, "%hd %hd %hd\n",
				value_data.datax,
				value_data.datay,
				value_data.dataz);

	return count;
}

static ssize_t bmg_show_range(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char range = 0;
	BMG_CALL_API(get_range_reg)(&range);
	err = sprintf(buf, "%d\n", range);
	return err;
}

static ssize_t bmg_store_range(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long range;
	err = kstrtoul(buf, 10, &range);
	if (err)
		return err;
	BMG_CALL_API(set_range_reg)(range);
	return count;
}

static ssize_t bmg_show_bandwidth(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char bandwidth = 0;
	BMG_CALL_API(get_bw)(&bandwidth);
	err = sprintf(buf, "%d\n", bandwidth);
	return err;
}

static ssize_t bmg_store_bandwidth(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long bandwidth;
	err = kstrtoul(buf, 10, &bandwidth);
	if (err)
		return err;
	BMG_CALL_API(set_bw)(bandwidth);
	return count;
}


static ssize_t bmg_show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	int err;

	mutex_lock(&client_data->mutex_enable);
	err = sprintf(buf, "%d\n", client_data->enable);
	mutex_unlock(&client_data->mutex_enable);
	return err;
}

static ssize_t bmg_store_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	data = data ? 1 : 0;
#if 0	
	mutex_lock(&client_data->mutex_enable);
	if (data != client_data->enable) {
		if (data) {
			#ifdef BMG160_USE_DELAY_WORK
			schedule_delayed_work(
					&client_data->work,
					msecs_to_jiffies(atomic_read(
							&client_data->delay)));
			#else
			hrtimer_start(&client_data->poll_timer,ns_to_ktime(atomic_read(
				&client_data->delay) * 1000000UL),HRTIMER_MODE_REL);	
			#endif
		} else {
		#ifdef BMG160_USE_DELAY_WORK
			cancel_delayed_work_sync(&client_data->work);
		#else
			hrtimer_cancel(&client_data->poll_timer);
			cancel_work_sync(&client_data->poll_work);
		#endif
		}

		client_data->enable = data;
	}
	mutex_unlock(&client_data->mutex_enable);
#endif

	bmg_enable_set(&client_data->bmg_cdev,data);
	
	return count;
}

//[Feature]-BEGIN by TCTSZ.yongzhong.cheng@tcl.com,2015/9/16,for 538697:bmg160 getdata ok
#ifdef POWER_REGULATOR_BMG
static int bmg_power_ctl(struct bmg_client_data *data, bool on)
{
	int ret = 0;
	printk("func=%s,on=%d,last power_enabled=%d\n",__func__,on,data->power_enabled);
	if (!on && data->power_enabled) {
		ret = regulator_disable(data->vdd);
		if (ret) {
			printk("Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(data->vio);
		if (ret) {
			printk("Regulator vio disable failed ret=%d\n", ret);
			return ret;
		}
		data->power_enabled = on;
		dev_dbg(&data->client->dev, "bmg_power_ctl on=%d\n",
				on);
		
	} else if (on && !data->power_enabled) {
		ret = regulator_enable(data->vdd);
		if (ret) {
			printk("Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_enable(data->vio);
		if (ret) {
			printk("Regulator vio enable failed ret=%d\n", ret);
			regulator_disable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
		dev_dbg(&data->client->dev, "bmg_power_ctl on=%d\n",
				on);
	} else {
		dev_warn(&data->client->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
	}

	return ret;
}

static int bmg_power_init(struct bmg_client_data *data, bool on)
{
	int ret;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd,
					0, BMG_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio,
					0, BMG_VIO_MAX_UV);

		regulator_put(data->vio);
	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			ret = PTR_ERR(data->vdd);
			printk("Regulator get failed vdd ret=%d\n", ret);
			return ret;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			ret = regulator_set_voltage(data->vdd,
					BMG_VDD_MIN_UV,
					BMG_VDD_MAX_UV);
			if (ret) {
				printk("Regulator set failed vdd ret=%d\n",ret);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			ret = PTR_ERR(data->vio);
			printk("Regulator get failed vio ret=%d\n", ret);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			ret = regulator_set_voltage(data->vio,
					BMG_VIO_MIN_UV,
					BMG_VIO_MAX_UV);
			if (ret) {
				printk("Regulator set failed vio ret=%d\n", ret);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, BMG_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return ret;
}

#endif


 int bmg_enable_set(struct sensors_classdev *sensors_cdev,unsigned int enabled)
{    
	#ifdef POWER_REGULATOR_BMG 
		int ret;
	#endif

     struct bmg_client_data *client_data = (struct bmg_client_data *)container_of(
            sensors_cdev, struct bmg_client_data, bmg_cdev);

     #ifdef POWER_REGULATOR_BMG    
		if(enabled) {
			ret = bmg_power_ctl(client_data, enabled);
			if (ret){
				printk("func=%s,power on bmg160 power error\n",__func__);
				return ret;}
		}
    #endif
	
	mutex_lock(&client_data->mutex_enable);
	client_data->enable=enabled;
	
	if (client_data->enable) {
		BMG_CALL_API(set_mode)(BMG_VAL_NAME(MODE_NORMAL));
		BMG_CALL_API(set_bw)(0x2);
	}else{
		BMG_CALL_API(set_mode)(BMG_VAL_NAME(MODE_SUSPEND));
	}

	if (enabled){
		#ifdef BMG160_USE_DELAY_WORK
		schedule_delayed_work(&client_data->work,
				msecs_to_jiffies(atomic_read(&client_data->delay)));
		#else
		hrtimer_start(&client_data->poll_timer,ns_to_ktime(atomic_read(
			&client_data->delay) * 1000000UL),HRTIMER_MODE_REL);
		#endif
	} else {
	#ifdef BMG160_USE_DELAY_WORK
		cancel_delayed_work_sync(&client_data->work);
	#else
		hrtimer_cancel(&client_data->poll_timer);
		//cancel_work_sync(&client_data->poll_work);//important CTS
	#endif
	}
	mutex_unlock(&client_data->mutex_enable);
	
	#ifdef POWER_REGULATOR_BMG    
		if(!enabled) {
			ret = bmg_power_ctl(client_data, enabled);
			if (ret){
				printk("func=%s,shutdown bmg160 power error\n",__func__);
				return ret;}
		}
    #endif
	return 0;
}


int bmg_poll_delay_set(struct sensors_classdev *sensors_cdev,unsigned int delay_msec)
{
	struct bmg_client_data *client_data = (struct bmg_client_data *)container_of(
				sensors_cdev, struct bmg_client_data, bmg_cdev);
				
	printk("func=%s,delay_msec=%d\n",__func__,delay_msec);
	atomic_set(&client_data->delay, delay_msec);

	return 0;
}
//[Feature]-END by TCTSZ.yongzhong.cheng@tcl.com, 2015/9/16,for 538697

#ifdef PINCTRL_BMG

#ifdef JRD_PROJECT_PIXI4554G
static int bmg_pinctrl_init_pixi4554g(struct bmg_client_data *data)
{
	data->pinctrl = devm_pinctrl_get(&data->client->dev);
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		printk("bmg_pinctrl_init:Failed to get pinctrl\n");
		return PTR_ERR(data->pinctrl);
	}
	
	data->bmi_int11_init= pinctrl_lookup_state(data->pinctrl, "bmi_int11_init");
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		printk("bmg_pinctrl_init:Failed to look up default state\n");
		return PTR_ERR(data->pinctrl);
	}

	data->bmi_int11_sleep= pinctrl_lookup_state(data->pinctrl, "bmi_int11_sleep");
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		printk("bmg_pinctrl_init:Failed to look up default state\n");
		return PTR_ERR(data->pinctrl);
	}

	data->bmi_int91_init= pinctrl_lookup_state(data->pinctrl, "bmi_int91_init");
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		printk("bmg_pinctrl_init:Failed to look up default state\n");
		return PTR_ERR(data->pinctrl);
	}

	data->bmi_int91_sleep= pinctrl_lookup_state(data->pinctrl, "bmi_int91_sleep");
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		printk("bmg_pinctrl_init:Failed to look up default state\n");
		return PTR_ERR(data->pinctrl);
	}
	printk("bmg_pinctrl_init success\n");
	return 0;
}
static int bmg_pinctrl_state_pixi4554g(struct bmg_client_data *client_data,bool active)
{
	int err=0;
	printk("bmg_pinctrl_state active=%d\n", active);
	
	if (active) {
		if (!IS_ERR_OR_NULL(client_data->bmi_int11_init) || !IS_ERR_OR_NULL(client_data->bmi_int91_init)) {
			err += pinctrl_select_state(client_data->pinctrl, client_data->bmi_int11_init);			
			err += pinctrl_select_state(client_data->pinctrl, client_data->bmi_int91_init);
		}else{
			err=1;}
	} else {
		if (!IS_ERR_OR_NULL(client_data->bmi_int11_sleep) || !IS_ERR_OR_NULL(client_data->bmi_int91_sleep)) {
			err += pinctrl_select_state(client_data->pinctrl, client_data->bmi_int11_sleep);
			err += pinctrl_select_state(client_data->pinctrl, client_data->bmi_int91_sleep);
		}else{
			err=1;}
	}
	if (err)
		printk("Error pinctrl_select_state IS_ERR_OR_NULL err:%d\n",err);
	return err;
}
#else
static int bmg_pinctrl_init(struct bmg_client_data *data)
{
	data->pinctrl = devm_pinctrl_get(&data->client->dev);
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		printk("bmg_pinctrl_init:Failed to get pinctrl\n");
		return PTR_ERR(data->pinctrl);
	}
	
	data->bmg_int98_init= pinctrl_lookup_state(data->pinctrl, "bmg_int98_init");
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		printk("bmg_pinctrl_init:Failed to look up default state\n");
		return PTR_ERR(data->pinctrl);
	}

	data->bmg_int98_sleep= pinctrl_lookup_state(data->pinctrl, "bmg_int98_sleep");
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		printk("bmg_pinctrl_init:Failed to look up default state\n");
		return PTR_ERR(data->pinctrl);
	}
	return 0;
}
static int bmg_pinctrl_state(struct bmg_client_data *client_data,bool active)
{
	int err=0;
	printk("bmg_pinctrl_state active=%d\n", active);
	
	if (active) {
		if (!IS_ERR_OR_NULL(client_data->bmg_int98_init)) {
			err += pinctrl_select_state(client_data->pinctrl, client_data->bmg_int98_init);			
		}else{
			err=1;}
	} else {
		if (!IS_ERR_OR_NULL(client_data->bmg_int98_sleep) ) {
			err += pinctrl_select_state(client_data->pinctrl, client_data->bmg_int98_sleep);
		}else{
			err=1;}
	}
	if (err)
		printk("Error pinctrl_select_state IS_ERR_OR_NULL err:%d\n",err);
	return err;
}
#endif
#endif

static ssize_t bmg_show_delay(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", atomic_read(&client_data->delay));

}

static ssize_t bmg_store_delay(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	if (data == 0) {
		err = -EINVAL;
		return err;
	}

	if (data < BMG_DELAY_MIN)
		data = BMG_DELAY_MIN;

	atomic_set(&client_data->delay, data);

	return count;
}


static ssize_t bmg_store_fastoffset_en(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long fastoffset_en;
	err = kstrtoul(buf, 10, &fastoffset_en);
	if (err)
		return err;
	if (fastoffset_en) {

#ifdef CONFIG_SENSORS_BMI058
		BMG_CALL_API(set_fast_offset_en_ch)(BMI058_X_AXIS, 1);
		BMG_CALL_API(set_fast_offset_en_ch)(BMI058_Y_AXIS, 1);
#else
		BMG_CALL_API(set_fast_offset_en_ch)(BMG160_X_AXIS, 1);
		BMG_CALL_API(set_fast_offset_en_ch)(BMG160_Y_AXIS, 1);
#endif

		BMG_CALL_API(set_fast_offset_en_ch)(BMG160_Z_AXIS, 1);
		BMG_CALL_API(enable_fast_offset)();
	}
	return count;
}

static ssize_t bmg_store_slowoffset_en(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long slowoffset_en;
	err = kstrtoul(buf, 10, &slowoffset_en);
	if (err)
		return err;
	if (slowoffset_en) {
		BMG_CALL_API(set_slow_offset_th)(3);
		BMG_CALL_API(set_slow_offset_dur)(0);
#ifdef CONFIG_SENSORS_BMI058
		BMG_CALL_API(set_slow_offset_en_ch)(BMI058_X_AXIS, 1);
		BMG_CALL_API(set_slow_offset_en_ch)(BMI058_Y_AXIS, 1);
#else
		BMG_CALL_API(set_slow_offset_en_ch)(BMG160_X_AXIS, 1);
		BMG_CALL_API(set_slow_offset_en_ch)(BMG160_Y_AXIS, 1);
#endif
		BMG_CALL_API(set_slow_offset_en_ch)(BMG160_Z_AXIS, 1);
	} else {
#ifdef CONFIG_SENSORS_BMI058
	BMG_CALL_API(set_slow_offset_en_ch)(BMI058_X_AXIS, 0);
	BMG_CALL_API(set_slow_offset_en_ch)(BMI058_Y_AXIS, 0);
#else
	BMG_CALL_API(set_slow_offset_en_ch)(BMG160_X_AXIS, 0);
	BMG_CALL_API(set_slow_offset_en_ch)(BMG160_Y_AXIS, 0);
#endif
	BMG_CALL_API(set_slow_offset_en_ch)(BMG160_Z_AXIS, 0);
	}

	return count;
}

static ssize_t bmg_show_selftest(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char selftest;
	BMG_CALL_API(selftest)(&selftest);
	err = sprintf(buf, "%d\n", selftest);
	return err;
}

static ssize_t bmg_show_sleepdur(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char sleepdur;
	BMG_CALL_API(get_sleepdur)(&sleepdur);
	err = sprintf(buf, "%d\n", sleepdur);
	return err;
}

static ssize_t bmg_store_sleepdur(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long sleepdur;
	err = kstrtoul(buf, 10, &sleepdur);
	if (err)
		return err;
	BMG_CALL_API(set_sleepdur)(sleepdur);
	return count;
}

static ssize_t bmg_show_autosleepdur(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char autosleepdur;
	BMG_CALL_API(get_autosleepdur)(&autosleepdur);
	err = sprintf(buf, "%d\n", autosleepdur);
	return err;
}

static ssize_t bmg_store_autosleepdur(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long autosleepdur;
	unsigned char bandwidth;
	err = kstrtoul(buf, 10, &autosleepdur);
	if (err)
		return err;
	BMG_CALL_API(get_bw)(&bandwidth);
	BMG_CALL_API(set_autosleepdur)(autosleepdur, bandwidth);
	return count;
}

static ssize_t bmg_show_place(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	int place = BOSCH_SENSOR_PLACE_UNKNOWN;

	if (NULL != client_data->bst_pd)
		place = client_data->bst_pd->place;

	return sprintf(buf, "%d\n", place);
}


#ifdef BMG_DEBUG
static ssize_t bmg_store_softreset(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long softreset;
	err = kstrtoul(buf, 10, &softreset);
	if (err)
		return err;
	BMG_CALL_API(set_soft_reset)();
	return count;
}

static ssize_t bmg_show_dumpreg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	u8 reg[0x40];
	int i;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	for (i = 0; i < 0x40; i++) {
		bmg_i2c_read(client_data->client, i, reg+i, 1);

		count += sprintf(&buf[count], "0x%x: 0x%x\n", i, reg[i]);
	}
	return count;
}
#endif

#ifdef BMG_USE_FIFO
static ssize_t bmg_show_fifo_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_mode;
	BMG_CALL_API(get_fifo_mode)(&fifo_mode);
	err = sprintf(buf, "%d\n", fifo_mode);
	return err;
}

static ssize_t bmg_store_fifo_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long fifo_mode;
	err = kstrtoul(buf, 10, &fifo_mode);
	if (err)
		return err;
	BMG_CALL_API(set_fifo_mode)(fifo_mode);
	return count;
}

static ssize_t bmg_show_fifo_framecount(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_framecount;
	BMG_CALL_API(get_fifo_framecount)(&fifo_framecount);
	err = sprintf(buf, "%d\n", fifo_framecount);
	return err;
}

static ssize_t bmg_store_fifo_framecount(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	client_data->fifo_count = (unsigned int) data;

	return count;
}

static ssize_t bmg_show_fifo_overrun(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_overrun;
	BMG_CALL_API(get_fifo_overrun)(&fifo_overrun);
	err = sprintf(buf, "%d\n", fifo_overrun);
	return err;
}

/*!
 * brief: bmg single axis data remaping
 * @param[i] fifo_datasel   fifo axis data select setting
 * @param[i/o] remap_dir   remapping direction
 * @param[i] client_data   to transfer sensor place
 *
 * @return none
 */
static void bmg_single_axis_remaping(unsigned char fifo_datasel,
		unsigned char *remap_dir, struct bmg_client_data *client_data)
{
	if ((NULL == client_data->bst_pd) ||
			(BOSCH_SENSOR_PLACE_UNKNOWN
			 == client_data->bst_pd->place))
		return;
	else {
		signed char place = client_data->bst_pd->place;
		/* sensor with place 0 needs not to be remapped */
		if ((place <= 0) ||
			(place >= MAX_AXIS_REMAP_TAB_SZ))
			return;

		if (fifo_datasel < 1 || fifo_datasel > 3)
			return;
		else {
			switch (fifo_datasel) {
			/*P2, P3, P4, P5 X axis(andorid) need to reverse*/
			case BMG160_FIFO_DAT_SEL_X:
				if (-1 == bst_axis_remap_tab_dft[place].sign_x)
					*remap_dir = 1;
				else
					*remap_dir = 0;
				break;
			/*P1, P2, P5, P6 Y axis(andorid) need to reverse*/
			case BMG160_FIFO_DAT_SEL_Y:
				if (-1 == bst_axis_remap_tab_dft[place].sign_y)
					*remap_dir = 1;
				else
					*remap_dir = 0;
				break;
			case BMG160_FIFO_DAT_SEL_Z:
			/*P4, P5, P6, P7 Z axis(andorid) need to reverse*/
				if (-1 == bst_axis_remap_tab_dft[place].sign_z)
					*remap_dir = 1;
				else
					*remap_dir = 0;
				break;
			default:
				break;
			}
		}
	}

	return;
}

static ssize_t bmg_show_fifo_data_frame(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err, i, len;
	signed char fifo_data_out[MAX_FIFO_F_LEVEL * MAX_FIFO_F_BYTES] = {0};
	unsigned char f_len = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	struct bmg160_data_t gyro_lsb;
	unsigned char axis_dir_remap = 0;
	s16 value;

	if (client_data->fifo_count == 0)
		return -ENOENT;

	if (client_data->fifo_datasel)
		/*Select one axis data output for every fifo frame*/
		f_len = 2;
	else
		/*Select X Y Z axis data output for every fifo frame*/
		f_len = 6;

	bmg_i2c_burst_read(client_data->client, BMG160_FIFO_DATA_ADDR,
			fifo_data_out, client_data->fifo_count * f_len);
	err = 0;

	if (f_len == 6) {
		/* Select X Y Z axis data output for every frame */
		for (i = 0; i < client_data->fifo_count; i++) {
			gyro_lsb.datax =
			((unsigned char)fifo_data_out[i * f_len + 1] << 8
				| (unsigned char)fifo_data_out[i * f_len + 0]);

			gyro_lsb.datay =
			((unsigned char)fifo_data_out[i * f_len + 3] << 8
				| (unsigned char)fifo_data_out[i * f_len + 2]);

			gyro_lsb.dataz =
			((unsigned char)fifo_data_out[i * f_len + 5] << 8
				| (unsigned char)fifo_data_out[i * f_len + 4]);

			bmg160_remap_sensor_data(&gyro_lsb, client_data);
			len = sprintf(buf, "%d %d %d ",
				gyro_lsb.datax, gyro_lsb.datay, gyro_lsb.dataz);
			buf += len;
			err += len;
		}
	} else {
		/* single axis data output for every frame */
		bmg_single_axis_remaping(client_data->fifo_datasel,
				&axis_dir_remap, client_data);
		for (i = 0; i < client_data->fifo_count * f_len / 2; i++) {
			value = ((unsigned char)fifo_data_out[2 * i + 1] << 8 |
					(unsigned char)fifo_data_out[2 * i]);
			if (axis_dir_remap)
				value = 0 - value;
			len = sprintf(buf, "%d ", value);
			buf += len;
			err += len;
		}
	}

	return err;
}

/*!
 * @brief show fifo_data_sel axis definition(Android definition, not sensor HW reg).
 * 0--> x, y, z axis fifo data for every frame
 * 1--> only x axis fifo data for every frame
 * 2--> only y axis fifo data for every frame
 * 3--> only z axis fifo data for every frame
 */
static ssize_t bmg_show_fifo_data_sel(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_data_sel;
	struct i2c_client *client = to_i2c_client(dev);
	struct bmg_client_data *client_data = i2c_get_clientdata(client);
	signed char place = BOSCH_SENSOR_PLACE_UNKNOWN;

	BMG_CALL_API(get_fifo_data_sel)(&fifo_data_sel);

	/*remapping fifo_dat_sel if define virtual place in BSP files*/
	if ((NULL != client_data->bst_pd) &&
		(BOSCH_SENSOR_PLACE_UNKNOWN != client_data->bst_pd->place)) {
		place = client_data->bst_pd->place;
		/* sensor with place 0 needs not to be remapped */
		if ((place > 0) && (place < MAX_AXIS_REMAP_TAB_SZ)) {
			if (BMG160_FIFO_DAT_SEL_X == fifo_data_sel)
				/* BMG160_FIFO_DAT_SEL_X: 1, Y:2, Z:3;
				*bst_axis_remap_tab_dft[i].src_x:0, y:1, z:2
				*so we need to +1*/
				fifo_data_sel =
					bst_axis_remap_tab_dft[place].src_x + 1;

			else if (BMG160_FIFO_DAT_SEL_Y == fifo_data_sel)
				fifo_data_sel =
					bst_axis_remap_tab_dft[place].src_y + 1;
		}

	}

	err = sprintf(buf, "%d\n", fifo_data_sel);
	return err;
}

/*!
 * @brief store fifo_data_sel axis definition(Android definition, not sensor HW reg).
 * 0--> x, y, z axis fifo data for every frame
 * 1--> only x axis fifo data for every frame
 * 2--> only y axis fifo data for every frame
 * 3--> only z axis fifo data for every frame
 */
static ssize_t bmg_store_fifo_data_sel(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)

{
	int err;
	unsigned long fifo_data_sel;

	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	signed char place;

	err = kstrtoul(buf, 10, &fifo_data_sel);
	if (err)
		return err;

	/*save fifo_data_sel(android axis definition)*/
	client_data->fifo_datasel = (unsigned char) fifo_data_sel;

	/*remaping fifo_dat_sel if define virtual place*/
	if ((NULL != client_data->bst_pd) &&
		(BOSCH_SENSOR_PLACE_UNKNOWN != client_data->bst_pd->place)) {
		place = client_data->bst_pd->place;
		/* sensor with place 0 needs not to be remapped */
		if ((place > 0) && (place < MAX_AXIS_REMAP_TAB_SZ)) {
			/*Need X Y axis revesal sensor place: P1, P3, P5, P7 */
			/* BMG160_FIFO_DAT_SEL_X: 1, Y:2, Z:3;
			  * but bst_axis_remap_tab_dft[i].src_x:0, y:1, z:2
			  * so we need to +1*/
			if (BMG160_FIFO_DAT_SEL_X == fifo_data_sel)
				fifo_data_sel =
					bst_axis_remap_tab_dft[place].src_x + 1;

			else if (BMG160_FIFO_DAT_SEL_Y == fifo_data_sel)
				fifo_data_sel =
					bst_axis_remap_tab_dft[place].src_y + 1;
		}
	}

	if (BMG_CALL_API(set_fifo_data_sel)(fifo_data_sel) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bmg_show_fifo_tag(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_tag;
	BMG_CALL_API(get_fifo_tag)(&fifo_tag);
	err = sprintf(buf, "%d\n", fifo_tag);
	return err;
}

static ssize_t bmg_store_fifo_tag(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)

{
	int err;
	unsigned long fifo_tag;
	err = kstrtoul(buf, 10, &fifo_tag);
	if (err)
		return err;
	BMG_CALL_API(set_fifo_tag)(fifo_tag);
	return count;
}
#endif

static DEVICE_ATTR(chip_id, S_IRUGO,
		bmg_show_chip_id, NULL);
static DEVICE_ATTR(op_mode, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bmg_show_op_mode, bmg_store_op_mode);
static DEVICE_ATTR(value, S_IRUGO,
		bmg_show_value, NULL);
static DEVICE_ATTR(range, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bmg_show_range, bmg_store_range);
static DEVICE_ATTR(bandwidth, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bmg_show_bandwidth, bmg_store_bandwidth);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bmg_show_enable, bmg_store_enable);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bmg_show_delay, bmg_store_delay);
static DEVICE_ATTR(fastoffset_en, S_IWUSR|S_IWGRP|S_IWOTH,
		NULL, bmg_store_fastoffset_en);
static DEVICE_ATTR(slowoffset_en, S_IWUSR|S_IWGRP|S_IWOTH,
		NULL, bmg_store_slowoffset_en);
static DEVICE_ATTR(selftest, S_IRUGO,
		bmg_show_selftest, NULL);
static DEVICE_ATTR(sleepdur, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bmg_show_sleepdur, bmg_store_sleepdur);
static DEVICE_ATTR(autosleepdur, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bmg_show_autosleepdur, bmg_store_autosleepdur);
static DEVICE_ATTR(place, S_IRUGO,
		bmg_show_place, NULL);

#ifdef BMG_DEBUG
static DEVICE_ATTR(softreset, S_IWUSR|S_IWGRP|S_IWOTH,
		NULL, bmg_store_softreset);
static DEVICE_ATTR(regdump, S_IRUGO,
		bmg_show_dumpreg, NULL);
#endif
#ifdef BMG_USE_FIFO
static DEVICE_ATTR(fifo_mode, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bmg_show_fifo_mode, bmg_store_fifo_mode);
static DEVICE_ATTR(fifo_framecount, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bmg_show_fifo_framecount, bmg_store_fifo_framecount);
static DEVICE_ATTR(fifo_overrun, S_IRUGO,
		bmg_show_fifo_overrun, NULL);
static DEVICE_ATTR(fifo_data_frame, S_IRUGO,
		bmg_show_fifo_data_frame, NULL);
static DEVICE_ATTR(fifo_data_sel, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bmg_show_fifo_data_sel, bmg_store_fifo_data_sel);
static DEVICE_ATTR(fifo_tag, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bmg_show_fifo_tag, bmg_store_fifo_tag);
#endif

static struct attribute *bmg_attributes[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_op_mode.attr,
	&dev_attr_value.attr,
	&dev_attr_range.attr,
	&dev_attr_bandwidth.attr,
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_fastoffset_en.attr,
	&dev_attr_slowoffset_en.attr,
	&dev_attr_selftest.attr,
	&dev_attr_sleepdur.attr,
	&dev_attr_autosleepdur.attr,
	&dev_attr_place.attr,
#ifdef BMG_DEBUG
	&dev_attr_softreset.attr,
	&dev_attr_regdump.attr,
#endif
#ifdef BMG_USE_FIFO
	&dev_attr_fifo_mode.attr,
	&dev_attr_fifo_framecount.attr,
	&dev_attr_fifo_overrun.attr,
	&dev_attr_fifo_data_frame.attr,
	&dev_attr_fifo_data_sel.attr,
	&dev_attr_fifo_tag.attr,
#endif
	NULL
};

static struct attribute_group bmg_attribute_group = {
	.attrs = bmg_attributes
};

static struct device_attribute bmg160_attrs[] = {
	__ATTR(selftest, 0444, bmg_show_selftest, NULL),
};

static int bmg_input_init(struct bmg_client_data *client_data)
{
	struct input_dev *dev;
	int err = 0;

	dev = input_allocate_device();
	if (NULL == dev)
		return -ENOMEM;

	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_MISC);
//[Feature]-BEGIN by TCTSZ.yongzhong.cheng@tcl.com,2015/9/16,for 538697:bmg160 getdata ok	
	input_set_abs_params(dev, ABS_RX, BMG_VALUE_MIN, BMG_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, ABS_RY, BMG_VALUE_MIN, BMG_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, ABS_RZ, BMG_VALUE_MIN, BMG_VALUE_MAX, 0, 0);
//[Feature]-END by TCTSZ.yongzhong.cheng@tcl.com, 2015/9/16,for 538697
	
	input_set_drvdata(dev, client_data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	client_data->input = dev;

	return 0;
}

static void bmg_input_destroy(struct bmg_client_data *client_data)
{
	struct input_dev *dev = client_data->input;

	input_unregister_device(dev);
	input_free_device(dev);
}



static int bmg_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct bmg_client_data *client_data = NULL;

	printk("func=%s\n",__func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("i2c_check_functionality error!\n");
		err = -EIO;
		goto exit_err_clean;
	}

	if (NULL == bmg_client) {
		bmg_client = client;
	} else {
		printk("this driver does not support multiple clients\n");
		err = -EINVAL;
		goto exit_err_clean;
	}

	/* do soft reset */
	mdelay(5);

	err = bmg_set_soft_reset(client);

	if (err < 0) {
		printk("erro soft reset!\n");
		err = -EINVAL;
		goto exit_err_clean;
	}
	mdelay(30);

	/* check chip id */
	err = bmg_check_chip_id(client);
	if (!err) {
		printk("Bosch Sensortec Device %s detected\n", SENSOR_NAME);
	} else {
		printk("Bosch Sensortec Device not found, chip id mismatch\n");
		err = -1;
		goto exit_err_clean;
	}

	client_data = kzalloc(sizeof(struct bmg_client_data), GFP_KERNEL);
	if (NULL == client_data) {
		printk("no memory available\n");
		err = -ENOMEM;
		goto exit_err_clean;
	}

	i2c_set_clientdata(client, client_data);
	client_data->client = client;

	mutex_init(&client_data->mutex_op_mode);
	mutex_init(&client_data->mutex_enable);

	/* input device init */
	err = bmg_input_init(client_data);
	if (err < 0)
		goto exit_err_clean;

	/* sysfs node creation */
	err = sysfs_create_group(&client_data->input->dev.kobj,
			&bmg_attribute_group);

    err = device_create_file(&client->dev,bmg160_attrs); 

	if (err < 0)
		goto exit_err_sysfs;

	if (NULL != client->dev.platform_data) {
		client_data->bst_pd = kzalloc(sizeof(*client_data->bst_pd),
				GFP_KERNEL);

		if (NULL != client_data->bst_pd) {
			memcpy(client_data->bst_pd, client->dev.platform_data,
					sizeof(*client_data->bst_pd));
			printk("%s sensor driver set place: p%d\n",
					SENSOR_NAME,
					client_data->bst_pd->place);
		}
	}

	/* workqueue init */
#ifdef BMG160_USE_DELAY_WORK	
	INIT_DELAYED_WORK(&client_data->work, bmg_work_func);
#else
	hrtimer_init(&client_data->poll_timer, CLOCK_MONOTONIC,HRTIMER_MODE_REL);
	client_data->poll_timer.function = poll_timer_report;
	INIT_WORK(&client_data->poll_work, bmg_work_func_getdata);
	client_data->data_wq = alloc_workqueue("bmg160_data_work",
		WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
#endif

	atomic_set(&client_data->delay, BMG_DELAY_DEFAULT);

	/* h/w init */
	client_data->device.bus_read = bmg_i2c_read_wrapper;
	client_data->device.bus_write = bmg_i2c_write_wrapper;
	client_data->device.delay_msec = bmg_i2c_delay;
	BMG_CALL_API(init)(&client_data->device);

//[Feature]-BEGIN by TCTSZ.yongzhong.cheng@tcl.com,2015/9/16,for 538697:bmg160 getdata ok
	client_data->bmg_cdev=sensors_bmg_cdev;
	client_data->bmg_cdev.sensors_enable = bmg_enable_set;
	client_data->bmg_cdev.sensors_poll_delay = bmg_poll_delay_set;
	err = sensors_classdev_register(&client_data->input->dev, &client_data->bmg_cdev);
	if (err)
		goto exit_err_clean;
//[Feature]-END by TCTSZ.yongzhong.cheng@tcl.com, 2015/9/16,for 538697

	bmg_dump_reg(client);

	client_data->enable = 0;
	client_data->fifo_datasel = 0;
	client_data->fifo_count = 0;

	/* now it's power on which is considered as resuming from suspend */
	err = BMG_CALL_API(set_mode)(
			BMG_VAL_NAME(MODE_SUSPEND));

	if (err < 0)
		goto exit_err_sysfs;


#ifdef CONFIG_HAS_EARLYSUSPEND
	client_data->early_suspend_handler.suspend = bmg_early_suspend;
	client_data->early_suspend_handler.resume = bmg_late_resume;
	register_early_suspend(&client_data->early_suspend_handler);
#endif

	printk("sensor %s probed successfully\n", SENSOR_NAME);

	dev_dbg(&client->dev,
		"i2c_client: %p client_data: %p i2c_device: %p input: %p",
		client, client_data, &client->dev, client_data->input);

//[Feature]-BEGIN by TCTSZ.yongzhong.cheng@tcl.com,2015/9/16,for 538697:bmg160 getdata ok
#ifdef POWER_REGULATOR_BMG
	printk("bmg160 defined POWER_REGULATOR_BMG\n");
			err = bmg_power_init(client_data, true);
		if (err)
			goto err_power_init;
	
		err = bmg_power_ctl(client_data, true);
		if (err)
			goto err_power_on;
	
		client_data->bmg_enabled = false;
		
		err = bmg_power_ctl(client_data, false);
		if (err)
			goto err_init_all_setting_power;
#else
	printk("bmg160 not defined POWER_REGULATOR_BMG\n");
#endif

#ifdef PINCTRL_BMG
	#ifdef JRD_PROJECT_PIXI4554G
		if (bmg_pinctrl_init_pixi4554g(client_data)) {
			printk("func=%s:bmg_pinctrl_init_pixi4554g error\n",__func__);
		}else{
			err += bmg_pinctrl_state_pixi4554g(client_data,true);
			if (err) {
				printk("Can't select pinctrl_pixi4554g state\n");
				return err;
			}
		}
	#else	
		if (bmg_pinctrl_init(client_data)) {
			printk("func=%s:bmg_pinctrl_init error\n",__func__);
		}else{
			err += bmg_pinctrl_state(client_data,true);
			if (err) {
				printk("Can't select pinctrl state\n");
				return err;
			}
		}
	#endif	
#endif

	return 0;

#ifdef POWER_REGULATOR_BMG
err_init_all_setting_power:
	bmg_power_ctl(client_data, false);
err_power_on:
	bmg_power_init(client_data, false);
err_power_init: 
#endif
//[Feature]-END by TCTSZ.yongzhong.cheng@tcl.com, 2015/9/16,for 538697

exit_err_sysfs:
	if (err)
		bmg_input_destroy(client_data);

exit_err_clean:
	if (err) {
		if (client_data != NULL) {
			kfree(client_data);
			client_data = NULL;
		}

		bmg_client = NULL;
	}

	return err;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bmg_early_suspend(struct early_suspend *handler)
{
	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)container_of(handler,
			struct bmg_client_data, early_suspend_handler);
	struct i2c_client *client = client_data->client;

	printk("func=%s\n",__func__);

	mutex_lock(&client_data->mutex_op_mode);
	if (client_data->enable) {
		err = bmg_pre_suspend(client);
		err = BMG_CALL_API(set_mode)(
				BMG_VAL_NAME(MODE_SUSPEND));
	}
	mutex_unlock(&client_data->mutex_op_mode);
}

static void bmg_late_resume(struct early_suspend *handler)
{

	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)container_of(handler,
			struct bmg_client_data, early_suspend_handler);
	struct i2c_client *client = client_data->client;

	printk("func=%s\n",__func__);

	mutex_lock(&client_data->mutex_op_mode);

	if (client_data->enable)
		err = BMG_CALL_API(set_mode)(BMG_VAL_NAME(MODE_NORMAL));

	/* post resume operation */
	bmg_post_resume(client);

	mutex_unlock(&client_data->mutex_op_mode);
}
#else
static int bmg_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);

	mutex_lock(&client_data->mutex_op_mode);
	client_data->on_before_suspend = client_data->enable;
	printk("func=%s,on_before_suspend=%d\n",__func__,client_data->on_before_suspend);
	
	if((client_data->enable)||(1==regulator_is_enabled(client_data->vdd))){
		bmg_enable_set(&client_data->bmg_cdev,0);
	}

	#ifdef PINCTRL_BMG
	#ifdef JRD_PROJECT_PIXI4554G
		bmg_pinctrl_state_pixi4554g(client_data,false);
	#else	
		bmg_pinctrl_state(client_data,false);
	#endif	
	#endif
	mutex_unlock(&client_data->mutex_op_mode);
	return err;
}

static int bmg_resume(struct i2c_client *client)
{
	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);

	mutex_lock(&client_data->mutex_op_mode);
	#ifdef PINCTRL_BMG
	#ifdef JRD_PROJECT_PIXI4554G
		bmg_pinctrl_state_pixi4554g(client_data,true);
	#else	
		bmg_pinctrl_state(client_data,true);
	#endif	
	#endif
	
	printk("func=%s,on_before_suspend=%d\n",__func__,client_data->on_before_suspend);
	if(client_data->on_before_suspend) {
		bmg_enable_set(&client_data->bmg_cdev,1);
	}
	mutex_unlock(&client_data->mutex_op_mode);
	return err;
}
#endif

void bmg_shutdown(struct i2c_client *client)
{
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);

	mutex_lock(&client_data->mutex_op_mode);
	BMG_CALL_API(set_mode)(
		BMG_VAL_NAME(MODE_DEEPSUSPEND));
	mutex_unlock(&client_data->mutex_op_mode);
}

static int bmg_remove(struct i2c_client *client)
{
	int err = 0;
	u8 op_mode;

	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);

	if (NULL != client_data) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&client_data->early_suspend_handler);
#endif
		mutex_lock(&client_data->mutex_op_mode);
		BMG_CALL_API(get_mode)(&op_mode);
		if (BMG_VAL_NAME(MODE_NORMAL) == op_mode) {
		#ifdef BMG160_USE_DELAY_WORK	
			cancel_delayed_work_sync(&client_data->work);
			printk("cancel work\n");
		#else
			hrtimer_cancel(&client_data->poll_timer);
			cancel_work_sync(&client_data->poll_work);
		#endif
		}
		mutex_unlock(&client_data->mutex_op_mode);

		err = BMG_CALL_API(set_mode)(
				BMG_VAL_NAME(MODE_SUSPEND));
		mdelay(BMG_I2C_WRITE_DELAY_TIME);

		sysfs_remove_group(&client_data->input->dev.kobj,
				&bmg_attribute_group);
		bmg_input_destroy(client_data);
		kfree(client_data);

		bmg_client = NULL;
	}

	return err;
}

static const struct i2c_device_id bmg_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bmg_id);

//[Feature]-BEGIN by TCTSZ.yongzhong.cheng@tcl.com,2015/9/16,for 538697:bmg160 getdata ok
static struct of_device_id bmg160_match_table[] = {
	{ .compatible = "bosch,bmg160",},
	{ },
};
//[Feature]-END by TCTSZ.yongzhong.cheng@tcl.com, 2015/9/16,for 538697

static struct i2c_driver bmg_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SENSOR_NAME,
		.of_match_table = bmg160_match_table,
	},
	.class = I2C_CLASS_HWMON,
	.id_table = bmg_id,
	.probe = bmg_probe,
	.remove = bmg_remove,
	.shutdown = bmg_shutdown,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = bmg_suspend,
	.resume = bmg_resume,
#endif
};

static int __init BMG_init(void)
{
	return i2c_add_driver(&bmg_driver);
}

static void __exit BMG_exit(void)
{
	i2c_del_driver(&bmg_driver);
}

MODULE_AUTHOR("contact@bosch-sensortec.com>");
MODULE_DESCRIPTION("BMG GYROSCOPE SENSOR DRIVER");
MODULE_LICENSE("GPL v2");

module_init(BMG_init);
module_exit(BMG_exit);
