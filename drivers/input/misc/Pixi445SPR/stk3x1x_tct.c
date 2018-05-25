/*
 *  stk3x1x.c - Linux kernel modules for sensortek stk301x, stk321x, stk331x 
 *  , and stk3410 proximity/ambient light sensor
 *
 *  Copyright (C) 2012~2013 Lex Hsieh / sensortek <lex_hsieh@sensortek.com.tw>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/errno.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/fs.h>   
#include <asm/uaccess.h> 
#include <linux/sensors.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
//#include <linux/earlysuspend.h>
#endif

//modify(add) by junfeng.zhou.sz for add power supply begin . 20140214
//modify(add) by junfeng.zhou.sz for [Psensor]Psensor insensitive with 607508 begin . 20140306
#define POWER_REGULATOR
//modify(add) by junfeng.zhou.sz for [Psensor]Psensor insensitive with 607508 end .
#ifdef POWER_REGULATOR
#include <linux/regulator/consumer.h>
/* POWER SUPPLY VOLTAGE RANGE */
#define STK3X1X_VDD_MIN_UV	2000000
#define STK3X1X_VDD_MAX_UV	3300000
#define STK3X1X_VIO_MIN_UV	1750000
#define STK3X1X_VIO_MAX_UV	1950000
#endif
//modify(add) by junfeng.zhou.sz for add power supply end . 
#define TUNE_LARGE_DATA
//modify(add) by junfeng.zhou.sz for add print time begin . 20140214
#define PRINT_TIME
#ifdef PRINT_TIME
#include <linux/rtc.h>
void print_local_time(char *param)
{
    	struct timespec ts;
	struct rtc_time tm;

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec+28800, &tm);
	if(param == NULL)
	    return ;
	printk(KERN_INFO "%s:%s %d-%02d-%02d %02d:%02d:%02d\n",__func__,param,
			 tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,tm.tm_hour, tm.tm_min, tm.tm_sec);
}
#endif
//modify(add) by junfeng.zhou.sz for add print time end .


//modify(add) by junfeng.zhou.sz for update the version with 533451 begin . 20140115
#define DRIVER_VERSION  "3.6.0 20140324"
//modify(add) by junfeng.zhou.sz for update the version with 533451 end . 
//modify(add) by junfeng.zhou.sz for evolution begin . 20140210
int call_flag_stk_ps = 0;  //flag for calling timing
#define STK_CHK_REG //check the register
//modify(add) by junfeng.zhou.sz for evolution end . 
/* Driver Settings */
#define CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
#ifdef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
//modify by junfeng.zhou . for change the report event duration
#define STK_ALS_CHANGE_THD	10	/* The threshold to trigger ALS interrupt, unit: lux */	
#define STK_ALS_DARKCODE_THD	5
#endif	/* #ifdef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD */
#define STK_INT_PS_MODE			1	/* 1, 2, or 3	*/
//#define STK_POLL_PS
#define STK_POLL_ALS		/* ALS interrupt is valid only when STK_PS_INT_MODE = 1	or 4*/
#define STK_TUNE0
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 begin . 20140115
#define CALI_EVERY_TIME
//#define STK_DEBUG_PRINTF
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 end . 
//#define SPREADTRUM_PLATFORM
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 begin . 20140115
#define STK_ALS_FIR
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 end . 
//#define STK_IRS
//#define STK_CHK_REG
//#define STK_GES
#include "stk3x1x_tct.h"
#ifdef SPREADTRUM_PLATFORM
	#include "stk3x1x_tct.h"
#else
	//#include "linux/stk3x1x.h"
#endif
/* Define Register Map */
#define STK_STATE_REG 			0x00
#define STK_PSCTRL_REG 			0x01
#define STK_ALSCTRL_REG 			0x02
#define STK_LEDCTRL_REG 			0x03
#define STK_INT_REG 				0x04
#define STK_WAIT_REG 			0x05
#define STK_THDH1_PS_REG 		0x06
#define STK_THDH2_PS_REG 		0x07
#define STK_THDL1_PS_REG 		0x08
#define STK_THDL2_PS_REG 		0x09
#define STK_THDH1_ALS_REG 		0x0A
#define STK_THDH2_ALS_REG 		0x0B
#define STK_THDL1_ALS_REG 		0x0C
#define STK_THDL2_ALS_REG 		0x0D
#define STK_FLAG_REG 			0x10
#define STK_DATA1_PS_REG	 	0x11
#define STK_DATA2_PS_REG 		0x12
#define STK_DATA1_ALS_REG 		0x13
#define STK_DATA2_ALS_REG 		0x14
#define STK_DATA1_OFFSET_REG 	0x15
#define STK_DATA2_OFFSET_REG 	0x16
#define STK_DATA1_IR_REG 		0x17
#define STK_DATA2_IR_REG 		0x18
#define STK_PDT_ID_REG 			0x3E
#define STK_RSRVD_REG 			0x3F
#define STK_SW_RESET_REG		0x80

#define STK_GSCTRL_REG			0x1A
#define STK_FLAG2_REG			0x1C

/* Define state reg */
#define STK_STATE_EN_IRS_SHIFT  	7
#define STK_STATE_EN_AK_SHIFT  	6
#define STK_STATE_EN_ASO_SHIFT  	5
#define STK_STATE_EN_IRO_SHIFT  	4
#define STK_STATE_EN_WAIT_SHIFT  	2
#define STK_STATE_EN_ALS_SHIFT  	1
#define STK_STATE_EN_PS_SHIFT  	0

#define STK_STATE_EN_IRS_MASK	0x80
#define STK_STATE_EN_AK_MASK	0x40
#define STK_STATE_EN_ASO_MASK	0x20
#define STK_STATE_EN_IRO_MASK	0x10
#define STK_STATE_EN_WAIT_MASK	0x04
#define STK_STATE_EN_ALS_MASK	0x02
#define STK_STATE_EN_PS_MASK	0x01

/* Define PS ctrl reg */
#define STK_PS_PRS_SHIFT  		6
#define STK_PS_GAIN_SHIFT  		4
#define STK_PS_IT_SHIFT  			0

#define STK_PS_PRS_MASK			0xC0
#define STK_PS_GAIN_MASK			0x30
#define STK_PS_IT_MASK			0x0F

/* Define ALS ctrl reg */
#define STK_ALS_PRS_SHIFT  		6
#define STK_ALS_GAIN_SHIFT  		4
#define STK_ALS_IT_SHIFT  			0

#define STK_ALS_PRS_MASK		0xC0
#define STK_ALS_GAIN_MASK		0x30
#define STK_ALS_IT_MASK			0x0F
	
/* Define LED ctrl reg */
#define STK_LED_IRDR_SHIFT  		6
#define STK_LED_DT_SHIFT  		0

#define STK_LED_IRDR_MASK		0xC0
#define STK_LED_DT_MASK			0x3F
	
/* Define interrupt reg */
#define STK_INT_CTRL_SHIFT  		7
#define STK_INT_OUI_SHIFT  		4
#define STK_INT_ALS_SHIFT  		3
#define STK_INT_PS_SHIFT  			0

#define STK_INT_CTRL_MASK		0x80
#define STK_INT_OUI_MASK			0x10
#define STK_INT_ALS_MASK			0x08
#define STK_INT_PS_MASK			0x07

#define STK_INT_ALS				0x08

/* Define flag reg */
#define STK_FLG_ALSDR_SHIFT  		7
#define STK_FLG_PSDR_SHIFT  		6
#define STK_FLG_ALSINT_SHIFT  		5
#define STK_FLG_PSINT_SHIFT  		4
#define STK_FLG_OUI_SHIFT  		2
#define STK_FLG_IR_RDY_SHIFT  		1
#define STK_FLG_NF_SHIFT  		0

#define STK_FLG_ALSDR_MASK		0x80
#define STK_FLG_PSDR_MASK		0x40
#define STK_FLG_ALSINT_MASK		0x20
#define STK_FLG_PSINT_MASK		0x10
#define STK_FLG_OUI_MASK			0x04
#define STK_FLG_IR_RDY_MASK		0x02
#define STK_FLG_NF_MASK			0x01
	
/* Define flag2 reg */
#define STK_FLG2_INT_GS_SHIFT		6
#define STK_FLG2_GS10_SHIFT		5
#define STK_FLG2_GS01_SHIFT		4

#define STK_FLG2_INT_GS_MASK	0x40
#define STK_FLG2_GS10_MASK		0x20
#define STK_FLG2_GS01_MASK		0x10

	
/* misc define */
#define MIN_ALS_POLL_DELAY_NS	60000000


#ifdef STK_TUNE0
#define STK_MAX_CT_VALUE	1200
//modify(add) by junfeng.zhou.sz for update the version with 533451 begin . 20140115
	#define STK_MAX_MIN_DIFF	300
	//modify by junfeng.zhou.sz for tune the p-sensor param with 577118 begin ,20131223
	#define STK_LT_N_CT	130
	#define STK_HT_N_CT	270
	#define STK_LOW_PS_LT_N_CT	60
	#define STK_LOW_PS_HT_N_CT	110
	#define STK_HI_PS_LT_N_CT	120
	#define STK_HI_PS_HT_N_CT	240
	//modify by junfeng.zhou.sz for tune the p-sensor param with 577118 end
//modify(add) by junfeng.zhou.sz for update the version with 533451 end . 
#endif	/* #ifdef STK_TUNE0 */

#define STK_IRC_MAX_ALS_CODE		20000
#define STK_IRC_MIN_ALS_CODE		25
#define STK_IRC_MIN_IR_CODE		50
#define STK_IRC_ALS_DENOMI		2		
#define STK_IRC_ALS_NUMERA		5
#define STK_IRC_ALS_CORREC		748

#define DEVICE_NAME		"stk3x1x"
#define ALS_NAME "stk3x1x-als"
#define PS_NAME "stk3x1x-ps"

#ifdef SPREADTRUM_PLATFORM
extern int sprd_3rdparty_gpio_pls_irq;

static struct stk3x1x_platform_data stk3x1x_pfdata={ 
  .state_reg = 0x0,    /* disable all */ 
  .psctrl_reg = 0x71,    /* ps_persistance=4, ps_gain=64X, PS_IT=0.391ms */ 
  .alsctrl_reg = 0x38, 	/* als_persistance=1, als_gain=64X, ALS_IT=50ms */
  .ledctrl_reg = 0xFF,   /* 100mA IRDR, 64/64 LED duty */ 
  .wait_reg = 0x07,    /* 50 ms */   
  .ps_thd_h =1700, 
  .ps_thd_l = 1500, 
  .int_pin = sprd_3rdparty_gpio_pls_irq,  
  .transmittance = 500, 
}; 
#endif

#if 0
static struct stk3x1x_platform_data stk3x1x_pfdata={ 
  .state_reg = 0x0,    /* disable all */ 
  .psctrl_reg = 0x71,    /* ps_persistance=4, ps_gain=64X, PS_IT=0.391ms */ 
  .alsctrl_reg = 0x38, 	/* als_persistance=1, als_gain=64X, ALS_IT=50ms */
  .ledctrl_reg = 0xFF,   /* 100mA IRDR, 64/64 LED duty */ 
  .wait_reg = 0x07,    /* 50 ms */   
  .ps_thd_h =1000, 
  .ps_thd_l = 800, 
  .int_pin = 0,  
  .transmittance = 500, 
};
#endif

#define TRACE_TAG 0x4646
#define DEF_TH_TAG 0x7878
#define DEF_TH_OFFSET_TAG 0x5454
#define CALI_DATA_TAG 0x9595
#define DEF_TH_LIMIT_TAG 0x3131
#define DEF_MMI_DATA 0x1234


struct factory_thd_offset {
	u16 tag;
	u16 low_ps_cali_th_offset_h;
	u16 low_ps_cali_th_offset_l;
	u16 hi_ps_cali_th_offset_h;
	u16 hi_ps_cali_th_offset_l;
	u16 reserve;
}__attribute__ ((__packed__));

struct factory_cali_cfg {
	u16 tag;
	u8 psctrl;
	u8 ledctrl;
}__attribute__ ((__packed__));

struct factory_thd_limit {
	u16 tag;
	u16 mmi_thd_h_limit;
	u16 mmi_thd_l_limit;
}__attribute__ ((__packed__));

struct factory_mmi_data {
	u16 tag;
	u16 mmi_data_offset;
	u16 mmi_data_rawoff;
	u16 mmi_data_rawdata;
}__attribute__ ((__packed__));


//************************************************
//  Note: this strust must not be over 40 bytes
//************************************************
struct trace_data {
	u16 tag;
	struct factory_thd_offset factory_def_thd_offset;
	struct factory_cali_cfg   factory_cali_data;
	struct factory_thd_limit  factory_def_thd_limit;
	struct factory_mmi_data   factory_def_mmi_data;
}__attribute__ ((__packed__));

int dark_code_flag = 0;
#ifdef STK_ALS_FIR
//modify by junfeng.zhou.sz for add power supply begin ,20140218
//#define STK_FIR_LEN	16
#define STK_FIR_LEN	32
//modify by junfeng.zhou.sz for add power supply end .
#define MAX_FIR_LEN 64

static struct sensors_classdev sensors_light_cdev = {
	.name = ALS_NAME,
	.vendor = "Sensortek",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "6500",
	.resolution = "0.0625",
	.sensor_power = "0.09",
	.min_delay = (MIN_ALS_POLL_DELAY_NS / 1000),	/* us */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = PS_NAME,
	.vendor = "Sensortek",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5.0",
	.resolution = "5.0",
	.sensor_power = "0.1",
	.min_delay = 0,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.flags = 1,	// add by ning.wei p-sensor is a wake up sensor ,flags must set 1
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

struct data_filter {
    	u16 raw[MAX_FIR_LEN];
    	int sum;
    	int number;
    	int idx;
};
#endif

#ifdef STK_GES
union stk_ges_operation{
	uint8_t ops[4];
	struct {
		uint8_t rw_len_retry;
		uint8_t reg;
		uint8_t reg_value_retry_crit;
		uint8_t sleep_10ns;
	}action;
};

union stk_ges_operation stk_ges_op[10] =
{
	{.ops={0xc1, 0x24, 0, 0}},
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}}
};
#endif
//modify(add) by junfeng.zhou.sz for add i2c lock with 656597 begin . 20140421
struct mutex stk_i2c_lock;
//modify(add) by junfeng.zhou.sz for add i2c lock with 656597 end . 
struct stk3x1x_data {
	struct i2c_client *client;
	struct stk3x1x_platform_data *pdata;
	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;
#if (!defined(STK_POLL_PS) || !defined(STK_POLL_ALS))
	int32_t irq;
    struct work_struct stk_work;
	struct workqueue_struct *stk_wq;	
#endif	
	uint16_t ir_code;
	uint16_t als_correct_factor;
	uint8_t alsctrl_reg;
	uint8_t psctrl_reg;
	uint8_t ledctrl_reg;
	uint8_t state_reg;
	int		int_pin;
	uint8_t wait_reg;
	uint8_t int_reg;
#ifdef CONFIG_HAS_EARLYSUSPEND
	//struct early_suspend stk_early_suspend;
#endif	
	uint16_t ps_thd_h;
	uint16_t ps_thd_l;
	struct mutex io_lock;
    struct mutex io_als_lock;
	struct input_dev *ps_input_dev;
	int32_t ps_distance_last;
	bool ps_enabled;
	bool re_enable_ps;
	struct wake_lock ps_wakelock;	
#ifdef STK_POLL_PS		
	struct hrtimer ps_timer;	
    struct work_struct stk_ps_work;
	struct workqueue_struct *stk_ps_wq;
	struct wake_lock ps_nosuspend_wl;		
#endif
	struct input_dev *als_input_dev;
	int32_t als_lux_last;
	uint32_t als_transmittance;	
	bool als_enabled;
	bool re_enable_als;
	ktime_t ps_poll_delay;
	ktime_t als_poll_delay;
#ifdef STK_POLL_ALS		
	struct work_struct stk_als_work;
	struct hrtimer als_timer;	
	struct workqueue_struct *stk_als_wq;
#endif	
	bool first_boot;
#ifdef STK_TUNE0
	uint16_t psa;
	uint16_t psi;	
	uint16_t psi_set;	
	//modify(add) by junfeng.zhou.sz for update the version with 533451 begin . 20140115
	uint16_t ps_high_thd_boot;
	uint16_t ps_low_thd_boot;	
	uint16_t ps_high_thd_def;
	uint16_t ps_low_thd_def;	
	//modify(add) by junfeng.zhou.sz for update the version with 533451 end . 
	struct hrtimer ps_tune0_timer;	
	struct workqueue_struct *stk_ps_tune0_wq;
    struct work_struct stk_ps_tune0_work;
	ktime_t ps_tune0_delay;
	bool tune_zero_init_proc;
	uint32_t ps_stat_data[3];
	int data_count;
#endif	
#ifdef STK_ALS_FIR
	struct data_filter      fir;
	atomic_t                firlength;	
#endif
	atomic_t	recv_reg;

#ifdef STK_GES		
	struct input_dev *ges_input_dev;
	int ges_enabled;
	int re_enable_ges;	
	atomic_t gesture2;
#endif

	bool use_fir;	
//modify(add) by junfeng.zhou.sz for add power supply begin . 20140214
#ifdef POWER_REGULATOR
	struct regulator *vdd;
	struct regulator *vio;
	bool power_enabled; 
#endif
//modify(add) by junfeng.zhou.sz for add power supply end . 
//modify(add) by junfeng.zhou.sz for fix i2c timeout error with 661727 begin . 20140501
    	int in_suspend;
    	int als_enable_pre;
    	int ps_enable_pre;
//modify(add) by junfeng.zhou.sz for fix i2c timeout error with 661727 end . 

// add by ning.wei if can not calibrate use before good calibration 
	bool is_have_good_cali;
	uint16_t  last_good_psi;
	uint16_t  min_psi;
	
	struct trace_data factory_data;

// [Feature]Add-BEGIN by TCTSZ. ning.wei@tcl.com, 2015/08/03, for add stk p/l sensor pull up&down ALM389755
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_sleep;
// [Feature]-Add-END by TCTSZ. ning.wei@tcl.com, 2015/08/03, for add stk p/l sensor pull up&down ALM389755	
};

//#define ALS_REPORT_LUX_TABLE
#ifdef ALS_REPORT_LUX_TABLE
static uint32_t lux_report_table[] =
{
	30,         //30b
	60,         //40b
	90,         //50b
	110,        //60b
	130,        //70b    
	150,        //80b
	180,        //90b
	230,        //110b
	1000,       //120b
	2000,       //250b
};
static uint32_t lux_adc_table[] =
{
	0,
	8,
	80,
	100,
	120,
	140,
	160,
	200,
	250,
	3500,
};
#endif
#if( !defined(CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD))
static uint32_t lux_threshold_table[] =
{
	3,
	10,
	40,
	65,
	145,
	300,
	550,
	930,
	1250,
	1700,
};

#define LUX_THD_TABLE_SIZE (sizeof(lux_threshold_table)/sizeof(uint32_t)+1)
static uint16_t code_threshold_table[LUX_THD_TABLE_SIZE+1];
#endif 	

static int32_t stk3x1x_enable_ps(struct stk3x1x_data *ps_data, uint8_t enable, uint8_t validate_reg);
static int32_t stk3x1x_enable_als(struct stk3x1x_data *ps_data, uint8_t enable);
static int32_t stk3x1x_set_ps_thd_l(struct stk3x1x_data *ps_data, uint16_t thd_l);
static int32_t stk3x1x_set_ps_thd_h(struct stk3x1x_data *ps_data, uint16_t thd_h);
static int32_t stk3x1x_set_als_thd_l(struct stk3x1x_data *ps_data, uint16_t thd_l);
static int32_t stk3x1x_set_als_thd_h(struct stk3x1x_data *ps_data, uint16_t thd_h);
//modify(add) by junfeng.zhou.sz for add power supply begin . 20140214
#ifdef POWER_REGULATOR
static int stk3x1x_device_ctl(struct stk3x1x_data *ps_data, bool enable);
#endif
//modify(add) by junfeng.zhou.sz for add power supply end . 
static int32_t stk3x1x_get_ir_reading(struct stk3x1x_data *ps_data);
#ifdef STK_TUNE0
static int stk_ps_tune_zero_func_fae(struct stk3x1x_data *ps_data);
#endif
#ifdef STK_CHK_REG
static int stk3x1x_validate_n_handle(struct i2c_client *client);
#endif
static int stk_ps_val(struct stk3x1x_data *ps_data);
static void stk3x1x_report_value(struct input_dev *dev, unsigned int code, int value);

static int stk3x1x_i2c_read_data(struct i2c_client *client, unsigned char command, int length, unsigned char *values)
{
	uint8_t retry;	
	int err;
	struct i2c_msg msgs[] = 
	{
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &command,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = values,
		},
	};
	
	mutex_lock(&stk_i2c_lock);
	for (retry = 0; retry < 5; retry++) 
	{
		err = i2c_transfer(client->adapter, msgs, 2);
		if (err == 2)
			break;
		else
			mdelay(5);
	}
	mutex_unlock(&stk_i2c_lock);
	//modify(add) by junfeng.zhou.sz for add i2c lock with 656597 end . 
	if (retry >= 5) 
	{
		printk(KERN_ERR "%s: i2c read fail, err=%d\n", __func__, err);
		return -EIO;
	} 
	return 0;		
}

static int stk3x1x_i2c_write_data(struct i2c_client *client, unsigned char command, int length, unsigned char *values)
{
	int retry;
	int err;	
	unsigned char data[11];
	struct i2c_msg msg;
	int index ;

    	mutex_lock(&stk_i2c_lock);
    	if (!client)
	{
	    	mutex_unlock(&stk_i2c_lock);
		return -EINVAL;
	}
    	else if (length >= 10) 
	{        
        	printk(KERN_ERR "%s:length %d exceeds 10\n", __func__, length);
        	mutex_unlock(&stk_i2c_lock);
        	return -EINVAL;
    	}   	
	
	data[0] = command;
	for (index=1;index<=length;index++)
		data[index] = values[index-1];	
	
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = length+1;
	msg.buf = data;

	for (retry = 0; retry < 5; retry++) 
	{

		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			break;
		else
			mdelay(5);
	}
	mutex_unlock(&stk_i2c_lock);
	//modify(add) by junfeng.zhou.sz for add i2c lock with 656597 end . 
	if (retry >= 5) 
	{
		printk(KERN_ERR "%s: i2c write fail, err=%d\n", __func__, err);		
		return -EIO;
	}
	return 0;
}

static int stk3x1x_i2c_smbus_read_byte_data(struct i2c_client *client, unsigned char command)
{
	unsigned char value;
	int err;
	err = stk3x1x_i2c_read_data(client, command, 1, &value);
	if(err < 0)
		return err;
	return value;
}

static int stk3x1x_i2c_smbus_write_byte_data(struct i2c_client *client, unsigned char command, unsigned char value)
{
	int err;
	err = stk3x1x_i2c_write_data(client, command, 1, &value);
	return err;
}

uint32_t stk_alscode2lux(struct stk3x1x_data *ps_data, uint32_t alscode)
{
	alscode += ((alscode<<7)+(alscode<<3)+(alscode>>1));   
    	alscode<<=3; 
    	alscode/=ps_data->als_transmittance;
	return alscode;
}

uint32_t stk_lux2alscode(struct stk3x1x_data *ps_data, uint32_t lux)
{
    	lux*=ps_data->als_transmittance;
    	lux/=1100;
    	if (unlikely(lux>=(1<<16)))
        	lux = (1<<16) -1;
    	return lux;
}

#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
static void stk_init_code_threshold_table(struct stk3x1x_data *ps_data)
{
    	uint32_t i,j;
    	uint32_t alscode;

    	code_threshold_table[0] = 0;
#ifdef STK_DEBUG_PRINTF	
    	printk(KERN_INFO "alscode[0]=%d\n",0);
#endif	
    	for (i=1,j=0;i<LUX_THD_TABLE_SIZE;i++,j++)
    	{
        	alscode = stk_lux2alscode(ps_data, lux_threshold_table[j]);
        	printk(KERN_INFO "alscode[%d]=%d\n",i,alscode);
        	code_threshold_table[i] = (uint16_t)(alscode);
    	}
    	code_threshold_table[i] = 0xffff;
    	printk(KERN_INFO "alscode[%d]=%d\n",i,alscode);
}

static uint32_t stk_get_lux_interval_index(uint16_t alscode)
{
    	uint32_t i;
    	for (i=1;i<=LUX_THD_TABLE_SIZE;i++)
    	{
        	if ((alscode>=code_threshold_table[i-1])&&(alscode<code_threshold_table[i]))
        	{
            		return i;
        	}
    	}
    	return LUX_THD_TABLE_SIZE;
}
#else
void stk_als_set_new_thd(struct stk3x1x_data *ps_data, uint16_t alscode)
{
    	int32_t high_thd,low_thd;
    	high_thd = alscode + stk_lux2alscode(ps_data, STK_ALS_CHANGE_THD);
    	low_thd = alscode - stk_lux2alscode(ps_data, STK_ALS_CHANGE_THD);
    	if (high_thd >= (1<<16))
        	high_thd = (1<<16) -1;
    	if (low_thd <0)
        	low_thd = 0;
    	stk3x1x_set_als_thd_h(ps_data, (uint16_t)high_thd);
    	stk3x1x_set_als_thd_l(ps_data, (uint16_t)low_thd);
}
#endif // CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD


static void stk3x1x_proc_plat_data(struct stk3x1x_data *ps_data, struct stk3x1x_platform_data *plat_data)
{
	uint8_t w_reg;
	
	ps_data->state_reg = plat_data->state_reg;
	ps_data->psctrl_reg = plat_data->psctrl_reg;
#ifdef STK_POLL_PS		
	ps_data->psctrl_reg &= 0x3F;
#endif		
	ps_data->alsctrl_reg = plat_data->alsctrl_reg;
	ps_data->ledctrl_reg = plat_data->ledctrl_reg;
	
	ps_data->wait_reg = plat_data->wait_reg;	
	if(ps_data->wait_reg < 2)
	{
		printk(KERN_WARNING "%s: wait_reg should be larger than 2, force to write 2\n", __func__);
		ps_data->wait_reg = 2;
	}
	else if (ps_data->wait_reg > 0xFF)
	{
		printk(KERN_WARNING "%s: wait_reg should be less than 0xFF, force to write 0xFF\n", __func__);
		ps_data->wait_reg = 0xFF;		
	}
#ifndef STK_TUNE0		
	ps_data->ps_thd_h = plat_data->ps_thd_h;
	ps_data->ps_thd_l = plat_data->ps_thd_l;		
#endif	

	w_reg = 0;
#ifndef STK_POLL_PS	
	w_reg |= STK_INT_PS_MODE;	
#else
	w_reg |= 0x01;		
#endif	

#if (!defined(STK_POLL_ALS) && (STK_INT_PS_MODE != 0x02) && (STK_INT_PS_MODE != 0x03))
	w_reg |= STK_INT_ALS;
#endif	
	ps_data->int_reg = w_reg;
	return;
}

static int32_t stk3x1x_init_all_reg(struct stk3x1x_data *ps_data)
{
	int32_t ret;
	
    	ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, ps_data->state_reg);
    	if (ret < 0)
    	{
        	printk(KERN_ERR "%s: write i2c error\n", __func__);
        	return ret;
    	}		
    	ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client, STK_PSCTRL_REG, ps_data->psctrl_reg);
    	if (ret < 0)
    	{
        	printk(KERN_ERR "%s: write i2c error\n", __func__);
        	return ret;
    	}	
    	ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client, STK_ALSCTRL_REG, ps_data->alsctrl_reg);
    	if (ret < 0)
    	{
        	printk(KERN_ERR "%s: write i2c error\n", __func__);
        	return ret;
    	}		
    	ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client, STK_LEDCTRL_REG, ps_data->ledctrl_reg);
    	if (ret < 0)
    	{
        	printk(KERN_ERR "%s: write i2c error\n", __func__);
        	return ret;
    	}	
    	ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client, STK_WAIT_REG, ps_data->wait_reg);
    	if (ret < 0)
    	{
        	printk(KERN_ERR "%s: write i2c error\n", __func__);
        	return ret;
    	}	
#ifdef STK_TUNE0	
	ps_data->psa = 0x0;
	ps_data->psi = 0xFFFF;	
#else
	stk3x1x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
	stk3x1x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);	
#endif	

    	ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client, STK_INT_REG, ps_data->int_reg);
    	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}		
	
	/*
    ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client, 0x87, 0x60);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}		
	*/
		
	return 0;	
}
	

static int32_t stk3x1x_check_pid(struct stk3x1x_data *ps_data)
{
	unsigned char value[2], pid_msb;
	int err;
	
	err = stk3x1x_i2c_read_data(ps_data->client, STK_PDT_ID_REG, 2, &value[0]);
	if(err < 0)
	{
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, err);
		return err;
	}
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 begin . 20140115
#ifdef STK_DEBUG_PRINTF	
	printk(KERN_INFO "%s: PID=0x%x, RID=0x%x\n", __func__, value[0], value[1]);
#endif
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 end . 
	if(value[1] == 0xC0)
		printk(KERN_INFO "%s: RID=0xC0!!!!!!!!!!!!!\n", __func__);	
		
	if(value[0] == 0)
	{
		printk(KERN_ERR "PID=0x0, please make sure the chip is stk3x1x!\n");
		return -2;			
	}
	
	pid_msb = value[0] & 0xF0;
	switch(pid_msb)
	{
		case 0x10:
		case 0x20:
		case 0x30:
			return 0;
		default:
			printk(KERN_ERR "%s: invalid PID(%#x)\n", __func__, value[0]);	
			return -1;
	}
	return 0;
}


static int32_t stk3x1x_software_reset(struct stk3x1x_data *ps_data)
{
    	int32_t r;
    	uint8_t w_reg;
	
    	w_reg = 0x7F;
    	r = stk3x1x_i2c_smbus_write_byte_data(ps_data->client,STK_WAIT_REG,w_reg);
    	if (r<0)
    	{
        	printk(KERN_ERR "%s: software reset: write i2c error, ret=%d\n", __func__, r);
        	return r;
    	}
    	r = stk3x1x_i2c_smbus_read_byte_data(ps_data->client,STK_WAIT_REG);
    	if (w_reg != r)
    	{
        	printk(KERN_ERR "%s: software reset: read-back value is not the same\n", __func__);
        	return -1;
    	}
	
    	r = stk3x1x_i2c_smbus_write_byte_data(ps_data->client,STK_SW_RESET_REG,0);
    	if (r<0)
    	{
        	printk(KERN_ERR "%s: software reset: read error after reset\n", __func__);
        	return r;
    	}
	usleep_range(13000, 15000);
    	return 0;
}


static int32_t stk3x1x_set_als_thd_l(struct stk3x1x_data *ps_data, uint16_t thd_l)
{
	unsigned char val[2];
	int ret;
	val[0] = (thd_l & 0xFF00) >> 8;
	val[1] = thd_l & 0x00FF;
	ret = stk3x1x_i2c_write_data(ps_data->client, STK_THDL1_ALS_REG, 2, val);
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);	
	return ret;		
}
static int32_t stk3x1x_set_als_thd_h(struct stk3x1x_data *ps_data, uint16_t thd_h)
{
	unsigned char val[2];
	int ret;
	val[0] = (thd_h & 0xFF00) >> 8;
	val[1] = thd_h & 0x00FF;
	ret = stk3x1x_i2c_write_data(ps_data->client, STK_THDH1_ALS_REG, 2, val);		
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);	
	return ret;	
}

static int32_t stk3x1x_set_ps_thd_l(struct stk3x1x_data *ps_data, uint16_t thd_l)
{
	unsigned char val[2];
	int ret;
	val[0] = (thd_l & 0xFF00) >> 8;
	val[1] = thd_l & 0x00FF;
	ret = stk3x1x_i2c_write_data(ps_data->client, STK_THDL1_PS_REG, 2, val);		
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
#ifdef STK_DEBUG_PRINTF	
	printk(KERN_ERR "%s: STK_THDL1_PS_REG =%d\n", __func__, thd_l);	
#endif
	return ret;	
}
static int32_t stk3x1x_set_ps_thd_h(struct stk3x1x_data *ps_data, uint16_t thd_h)
{	
	unsigned char val[2];
	int ret;
	val[0] = (thd_h & 0xFF00) >> 8;
	val[1] = thd_h & 0x00FF;
	ret = stk3x1x_i2c_write_data(ps_data->client, STK_THDH1_PS_REG, 2, val);		
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
#ifdef STK_DEBUG_PRINTF		
	printk(KERN_ERR "%s: STK_THDH1_PS_REG = %d\n", __func__, thd_h);
#endif
	return ret;
}

static uint32_t stk3x1x_get_ps_reading(struct stk3x1x_data *ps_data)
{	
	unsigned char value[2];
	int err;
	err = stk3x1x_i2c_read_data(ps_data->client, STK_DATA1_PS_REG, 2, &value[0]);
	if(err < 0)
	{
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, err);
		return err;
	}
	return ((value[0]<<8) | value[1]);	
}


static int32_t stk3x1x_set_flag(struct stk3x1x_data *ps_data, uint8_t org_flag_reg, uint8_t clr)
{
	uint8_t w_flag;
	int ret;
	
	w_flag = org_flag_reg | (STK_FLG_ALSINT_MASK | STK_FLG_PSINT_MASK | STK_FLG_OUI_MASK | STK_FLG_IR_RDY_MASK);
	w_flag &= (~clr);
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 begin . 20140115
#ifdef STK_DEBUG_PRINTF	
	printk(KERN_INFO "%s: org_flag_reg=0x%x, w_flag = 0x%x\n", __func__, org_flag_reg, w_flag);		
#endif
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 end . 
    	ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client,STK_FLAG_REG, w_flag);	
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}

static int32_t stk3x1x_get_flag(struct stk3x1x_data *ps_data)
{	
	int ret;
    	ret = stk3x1x_i2c_smbus_read_byte_data(ps_data->client,STK_FLAG_REG);	
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}

#ifdef STK_GES			
static int32_t stk3x1x_set_flag2(struct stk3x1x_data *ps_data, uint8_t org_flag2_reg, uint8_t clr)
{
	uint8_t w_flag2;
	int ret;
	
	w_flag2 = org_flag2_reg | 0x72;
	w_flag2 &= (~clr);
	//printk(KERN_INFO "%s: org_flag2_reg=0x%x, w_flag2 = 0x%x\n", __func__, org_flag2_reg, w_flag2);		
    	ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client,STK_FLAG2_REG, w_flag2);	
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}

static int32_t stk3x1x_get_flag2(struct stk3x1x_data *ps_data)
{	
	int ret;
    	ret = stk3x1x_i2c_smbus_read_byte_data(ps_data->client,STK_FLAG2_REG);	
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}
#endif

static int32_t stk3x1x_set_state(struct stk3x1x_data *ps_data, uint8_t state)
{
	int ret;		
    	ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client,STK_STATE_REG, state);	
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}

static int32_t stk3x1x_get_state(struct stk3x1x_data *ps_data)
{	
	int ret;
    	ret = stk3x1x_i2c_smbus_read_byte_data(ps_data->client,STK_STATE_REG);	
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}

#ifdef STK_GES		
static int32_t stk3x1x_set_gsctrl(struct stk3x1x_data *ps_data, uint8_t state)
{
	int ret;		
    	ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client,STK_GSCTRL_REG, state);	
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}

static int32_t stk3x1x_get_gsctrl(struct stk3x1x_data *ps_data)
{	
	int ret;
    	ret = stk3x1x_i2c_smbus_read_byte_data(ps_data->client,STK_GSCTRL_REG);	
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}

#if 0
static uint32_t stk3x1x_get_ges_reading(struct stk3x1x_data *ps_data, unsigned int *ges0,
				unsigned int *ges1, unsigned int *ges2)
{
	int retry, len, no = 0;
	uint8_t reg_val_crit;
	int err;
	unsigned char value[10];
	
	while(stk_ges_op[no].ops[1] != 0)
	{
		retry = stk_ges_op[no].action.rw_len_retry & 0x0F;
		len = (stk_ges_op[no].action.rw_len_retry & 0x70) >> 4;
		reg_val_crit = stk_ges_op[no].action.reg_value_retry_crit;
		if(stk_ges_op[no].action.rw_len_retry & 0x80)
		{
			while(retry != 0)
			{
				err = stk3x1x_i2c_read_data(ps_data->client, 
								stk_ges_op[no].action.reg, len, value);								
				if(err < 0)
				{
					printk(KERN_ERR "%s: fail, ret=%d\n", __func__, err);
					return err;
				}
				if(reg_val_crit)
				{
					if(value[0] & reg_val_crit)
						break;
				}
				if(stk_ges_op[no].action.sleep_10ns != 0)
					usleep_range(stk_ges_op[no].action.sleep_10ns*10, 
									stk_ges_op[no].action.sleep_10ns*10+300);
				retry--;
			}			
		}
		else
		{
			while(retry != 0)
			{
				err = stk3x1x_i2c_write_data(ps_data->client, stk_ges_op[no].action.reg, 
								len, &reg_val_crit);		
				if(err < 0)
				{
					printk(KERN_ERR "%s: fail, err=%d\n", __func__, err);			
					return err;
				}

				if(stk_ges_op[no].action.sleep_10ns != 0)
					usleep_range(stk_ges_op[no].action.sleep_10ns*10, 
									stk_ges_op[no].action.sleep_10ns*10+300);
				retry--;
			}			
		}
		
		if(stk_ges_op[no].action.reg == 0x24)
		{
			*ges0 = (value[0]<<8) | value[1];	
			*ges1 = (value[2]<<8) | value[3];
		}
		else if(stk_ges_op[no].action.reg == stk_ges_op[9].ops[0])
		{
			*ges2 = (value[0]<<8) | value[1];
		}
		
		no++;
	}
	return 0;
}

#else
static uint32_t stk3x1x_get_ges_reading(struct stk3x1x_data *ps_data, unsigned int *ges0,unsigned int *ges1,unsigned int *ges2)
{
	unsigned char value[4];
	int err, retry = 10;
	
	do {
		err = stk3x1x_get_flag(ps_data);	
		if(err < 0)
			return err;
		if(err & STK_FLG_PSDR_MASK)
			break;
		//printk(KERN_INFO "ges: not ready, %d\n", retry);
		retry--;
		usleep_range(350, 1000);
	} while(retry > 0);
	err = stk3x1x_i2c_read_data(ps_data->client, STK_DATA1_PS_REG, 2, &value[0]);
	
	err = stk3x1x_i2c_read_data(ps_data->client, 0x24, 4, &value[0]);
	if(err < 0)
	{
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, err);
		return err;
	}
	*ges0 = (value[0]<<8) | value[1];	
	*ges1 = (value[2]<<8) | value[3];	
	//printk(KERN_INFO "%s: ges=%d,%d\n",__func__, *ges0, *ges1);	
	return 0;
}
#endif




static int32_t stk3x1x_enable_ges(struct stk3x1x_data *ps_data, uint8_t enable, uint8_t mode)
{
    	int32_t ret;
	uint8_t w_state_reg, gsctrl_reg;
	uint8_t org_mode = 0;
		
	if(enable == ps_data->ges_enabled)
		return 0;	
	
	if(enable)
	{
		if(ps_data->ps_enabled)
		{
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 begin . 20140115
#ifdef STK_DEBUG_PRINTF	
			printk(KERN_INFO "%s: force disable PS\n", __func__);
#endif
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 end . 
			stk3x1x_enable_ps(ps_data, 0, 1);
			ps_data->re_enable_ps = true;
		}
		if(ps_data->als_enabled) 
		{
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 begin . 20140115
#ifdef STK_DEBUG_PRINTF	
			printk(KERN_INFO "%s: force disable ALS\n", __func__);		
#endif	
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 end . 
			stk3x1x_enable_als(ps_data, 0);
			ps_data->re_enable_als = true;
		}		
		
		ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client, STK_WAIT_REG, 0);
		if (ret < 0)
		{
			printk(KERN_ERR "%s: write i2c error\n", __func__);
			return ret;
		}			
		
		ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client, STK_INT_REG, 0);
		if (ret < 0)
		{
			printk(KERN_ERR "%s: write i2c error\n", __func__);
			return ret;
		}				
			
		w_state_reg = STK_STATE_EN_WAIT_MASK | STK_STATE_EN_PS_MASK; 
		ret = stk3x1x_set_state(ps_data, w_state_reg);
		if(ret < 0)
			return ret;
		
		if(mode == 2)
		{
			ret = stk3x1x_get_gsctrl(ps_data);
			if(ret < 0)
				return ret;		
			gsctrl_reg = ret & 0xF3;
#ifdef STK_POLL_PS
			gsctrl_reg |= 0x04;
#else
			gsctrl_reg |= 0x0C;
#endif			
			ret = stk3x1x_set_gsctrl(ps_data, gsctrl_reg);
			if(ret < 0)
				return ret;			
#ifdef STK_POLL_PS
			hrtimer_start(&ps_data->ps_timer, ps_data->ps_poll_delay, HRTIMER_MODE_REL);	
#else
			enable_irq(ps_data->irq);
#endif		
		}
		ps_data->ges_enabled = mode;
	}
	else
	{	
		org_mode = ps_data->ges_enabled;
		if(org_mode == 2)
		{	
#ifdef STK_POLL_PS
			hrtimer_cancel(&ps_data->ps_timer);
			cancel_work_sync(&ps_data->stk_ps_work);
#else	
			disable_irq(ps_data->irq);
#endif		
		}
	
		ret = stk3x1x_set_state(ps_data, 0);
		if(ret < 0)
			return ret;		
		
		ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client, STK_WAIT_REG, ps_data->wait_reg);
		if (ret < 0)
		{
			printk(KERN_ERR "%s: write i2c error\n", __func__);
			return ret;
		}	
		
		ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client, STK_INT_REG, ps_data->int_reg);
		if (ret < 0)
		{
			printk(KERN_ERR "%s: write i2c error\n", __func__);
			return ret;
		}				
		if(org_mode == 2)
		{			
			ret = stk3x1x_get_gsctrl(ps_data);
			if(ret < 0)
				return ret;		
			gsctrl_reg = ret & (~0x0C);
			ret = stk3x1x_set_gsctrl(ps_data, gsctrl_reg);		
			if(ret < 0)
				return ret;		
		}
		ps_data->ges_enabled = 0;
		if(ps_data->re_enable_ps)
		{
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 begin . 20140115
#ifdef STK_DEBUG_PRINTF	
			printk(KERN_INFO "%s: re-enable PS\n", __func__);
#endif
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 end . 
			stk3x1x_enable_ps(ps_data, 1, 1);
			ps_data->re_enable_ps = false;
		}
		if(ps_data->re_enable_als) 
		{
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 begin . 20140115
#ifdef STK_DEBUG_PRINTF	
			printk(KERN_INFO "%s: re-enable ALS\n", __func__);
#endif
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 end .
			stk3x1x_enable_als(ps_data, 1);
			ps_data->re_enable_als = false;
		}
	}	

	return 0;
}
#endif /* #ifdef STK_GES */

static void stk3x1x_report_value(struct input_dev *dev, unsigned int code, int value)
{
	ktime_t timestamp;
	timestamp = ktime_get_boottime();

	input_report_abs(dev, code ,value);
	input_event(dev, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(timestamp).tv_sec);
	input_event(dev, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(timestamp).tv_nsec);
	input_sync(dev);
}

static int32_t stk3x1x_enable_ps(struct stk3x1x_data *ps_data, uint8_t enable, uint8_t validate_reg)
{
    	int32_t ret;
	uint8_t w_state_reg;
	uint8_t curr_ps_enable;	
	uint32_t reading;
	int32_t near_far_state;

	curr_ps_enable = ps_data->ps_enabled?1:0;	
	if(curr_ps_enable == enable)
		return 0;
//modify(add) by junfeng.zhou.sz for add power supply begin . 20140214		
#ifdef POWER_REGULATOR
    	if (enable) {
		ret = stk3x1x_device_ctl(ps_data, enable);
		if (ret)
			return ret;
	} 
#endif
//modify(add) by junfeng.zhou.sz for add power supply end . 
#ifdef STK_CHK_REG
	if(validate_reg)
	{
		ret = stk3x1x_validate_n_handle(ps_data->client);
		if(ret < 0)	
			printk(KERN_ERR "stk3x1x_validate_n_handle fail: %d\n", ret); 
	}			
#endif /* #ifdef STK_CHK_REG */	
#ifdef STK_GES		
	if(ps_data->ges_enabled)
	{
		printk(KERN_INFO "%s: since ges is enabled, PS is disabled\n", __func__);
		ps_data->re_enable_ps = enable ? true : false;
		return 0;
	}
#endif
#ifdef STK_TUNE0
	if (!(ps_data->psi_set) && !enable)
	{

		hrtimer_cancel(&ps_data->ps_tune0_timer);					
		cancel_work_sync(&ps_data->stk_ps_tune0_work);

	}
#endif		
	if(ps_data->first_boot == true)
	{		
		ps_data->first_boot = false;
	}

	ret = stk3x1x_get_state(ps_data);
	if(ret < 0)
		return ret;
	w_state_reg = ret;
	
	
	w_state_reg &= ~(STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK | STK_STATE_EN_AK_MASK); 
	if(enable)	
	{
		w_state_reg |= STK_STATE_EN_PS_MASK;	
		if(!(ps_data->als_enabled))
			w_state_reg |= STK_STATE_EN_WAIT_MASK;			
	}	
	ret = stk3x1x_set_state(ps_data, w_state_reg);
	if(ret < 0)
		return ret;	

    if(enable)
	{
	    //modify(add) by junfeng.zhou.sz for evolution begin . 20140210
	    call_flag_stk_ps = 1;
	    //modify(add) by junfeng.zhou.sz for evolution end .
//modify(add) by junfeng.zhou.sz for cali the psensor eeverytime call with 533451 begin . 20140115
#ifdef STK_TUNE0
	#ifndef CALI_EVERY_TIME
		if (!(ps_data->psi_set))
			hrtimer_start(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay, HRTIMER_MODE_REL);
	#else
		if (1)
		{
			ps_data->psi_set = 0;
			ps_data->psa = 0x0;
			ps_data->psi = 0xFFFF;	
			ps_data->ps_high_thd_boot = ps_data->ps_high_thd_def;
            		ps_data->ps_low_thd_boot = ps_data->ps_low_thd_def;
			ps_data->ps_thd_h = ps_data->ps_high_thd_boot;
			ps_data->ps_thd_l = ps_data->ps_low_thd_boot;
#ifdef TUNE_LARGE_DATA
			ps_data->ps_thd_h = 0xFFFF; //change thd_h to 0xFFFF
			ps_data->ps_thd_l = 0xFFFF;	//change thd_l to 0xFFFF force report far when enable  
		//	ps_data->ps_thd_h = 1700; //change thd_h to 0xFFF
		//	ps_data->ps_thd_l = 1500;	//change thd_l to 0x0000 	
#endif
			stk3x1x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
			stk3x1x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
			
			hrtimer_start(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay, HRTIMER_MODE_REL);					
		}
	#endif
#endif
//modify(add) by junfeng.zhou.sz for cali the psensor eeverytime call with 533451 end .
	
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 begin . 20140115
#ifdef STK_DEBUG_PRINTF	
		printk(KERN_INFO "%s: HT=%d,LT=%d\n", __func__, ps_data->ps_thd_h,  ps_data->ps_thd_l);				
#endif
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 end .

#ifdef STK_POLL_PS		
		hrtimer_start(&ps_data->ps_timer, ps_data->ps_poll_delay, HRTIMER_MODE_REL);	
		ps_data->ps_distance_last = -1;	
#endif		
#ifndef STK_POLL_PS
	#ifndef STK_POLL_ALS		
		if(!(ps_data->als_enabled))
	#endif	/* #ifndef STK_POLL_ALS	*/
			enable_irq(ps_data->irq);
#endif	/* #ifndef STK_POLL_PS */						
		ps_data->ps_enabled = true;
#ifdef STK_CHK_REG		
		if(!validate_reg)		
		{
			ps_data->ps_distance_last = 1;
			//input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, 1);
			//input_sync(ps_data->ps_input_dev);
			stk3x1x_report_value(ps_data->ps_input_dev, ABS_DISTANCE, 1);
			wake_lock_timeout(&ps_data->ps_wakelock, HZ/2);
			reading = stk3x1x_get_ps_reading(ps_data);
			printk(KERN_INFO "%s: force report ps input event=1, ps code = %d\n",__func__, reading);				
		}
		else
#endif /* #ifdef STK_CHK_REG */
		{
			usleep_range(4000, 5000);
			ret = stk3x1x_get_flag(ps_data);
			if (ret < 0)
				return ret;
			near_far_state = ret & STK_FLG_NF_MASK;			
			//report a far event to cover the dirty data from dirver or android 
			// 0 near  and 1 far
			ps_data->ps_distance_last = near_far_state;
			//input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, near_far_state);
			//input_sync(ps_data->ps_input_dev);
			stk3x1x_report_value(ps_data->ps_input_dev, ABS_DISTANCE, near_far_state);
			wake_lock_timeout(&ps_data->ps_wakelock, HZ/2);
			reading = stk3x1x_get_ps_reading(ps_data);
			printk(KERN_INFO "%s: ps input event=%d, ps code = %d\n",__func__, near_far_state, reading);	
		}
	}
	else  //turn off the psensor
	{		
#ifdef STK_POLL_PS
		hrtimer_cancel(&ps_data->ps_timer);
		cancel_work_sync(&ps_data->stk_ps_work);
#else		
#ifndef STK_POLL_ALS
		if(!(ps_data->als_enabled))	
#endif				
			disable_irq(ps_data->irq);
#endif
		ps_data->ps_enabled = false;		
		//modify(add) by junfeng.zhou.sz for evolution begin . 20140210
		call_flag_stk_ps = 0;		
		//modify(add) by junfeng.zhou.sz for evolution end . 
		//modify by junfeng.zhou	for fix the register old thd begin .
		//after turn off the psensor ,set the thd register to 0xFFFF and 0x0
		ps_data->ps_thd_h = 0xFFFF; //change thd_h to 0xFFF
		ps_data->ps_thd_l = 0xFFFF;	//change thd_l to 0x0000 
		//ps_data->ps_thd_h = 1700; //change thd_h to 0xFFF
		//ps_data->ps_thd_l = 1500;	//change thd_l to 0x0000 
		stk3x1x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
		stk3x1x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
		//modify by junfeng.zhou	for fix the register old thd end .
		printk(KERN_INFO "%s: disable p-sensor\n",__func__);
	}
//modify(add) by junfeng.zhou.sz for add power supply begin . 20140214	
#ifdef POWER_REGULATOR
	if (!enable) {
		ret = stk3x1x_device_ctl(ps_data, enable);
		if (ret)
			return ret;
	}
#endif
//modify(add) by junfeng.zhou.sz for add power supply end . 
	return ret;
}

static int32_t stk3x1x_enable_als(struct stk3x1x_data *ps_data, uint8_t enable)
{
    	int32_t ret;
	uint8_t w_state_reg;
	uint8_t curr_als_enable = (ps_data->als_enabled)?1:0;

	printk("%s:enable=%d\n", __func__, enable);

	if(curr_als_enable == enable)
		return 0;
//modify(add) by junfeng.zhou.sz for add power supply begin . 20140214
#ifdef POWER_REGULATOR    
	if (enable) {
		ret = stk3x1x_device_ctl(ps_data, enable);
		if (ret)
			return ret;
	}
#endif
//modify(add) by junfeng.zhou.sz for add power supply end . 
//modify(add) by junfeng.zhou.sz for evolution begin . 20140210
#ifdef STK_CHK_REG
	ret = stk3x1x_validate_n_handle(ps_data->client);
	if(ret < 0)	
		printk(KERN_ERR "stk3x1x_validate_n_handle fail: %d\n", ret); 			
#endif /* #ifdef STK_CHK_REG */	
//modify(add) by junfeng.zhou.sz for evolution end . 
#ifdef STK_GES			
	if(ps_data->ges_enabled)
	{
		printk(KERN_INFO "%s: since ges is enabled, ALS is disabled\n", __func__);
		ps_data->re_enable_als = enable ? true : false;
		return 0;
	}	
#endif	/* #ifdef STK_GES */	

#ifdef STK_IRS
	if(enable && !(ps_data->ps_enabled))
	{		
		ret = stk3x1x_get_ir_reading(ps_data);
		if(ret > 0)
			ps_data->ir_code = ret;
	}		
#endif
	
#ifndef STK_POLL_ALS			
    	if (enable)
	{				
        	stk3x1x_set_als_thd_h(ps_data, 0x0000);
        	stk3x1x_set_als_thd_l(ps_data, 0xFFFF);		
	}
#endif	

	ret = stk3x1x_get_state(ps_data);
	if(ret < 0)
		return ret;
	
	w_state_reg = (uint8_t)(ret & (~(STK_STATE_EN_ALS_MASK | STK_STATE_EN_WAIT_MASK))); 
	if(enable)	
		w_state_reg |= STK_STATE_EN_ALS_MASK;	
	else if (ps_data->ps_enabled)		
		w_state_reg |= STK_STATE_EN_WAIT_MASK;	

	ret = stk3x1x_set_state(ps_data, w_state_reg);
	if(ret < 0)
		return ret;	

    	if (enable)
    	{						
		ps_data->als_enabled = true;
#ifdef STK_POLL_ALS
		// delay add by ning.wei for wait als data convert compelet pr:		
		//mdelay(100);
		hrtimer_start(&ps_data->als_timer, ns_to_ktime(210 * NSEC_PER_MSEC), HRTIMER_MODE_REL);		
#else
#ifndef STK_POLL_PS
		if(!(ps_data->ps_enabled))
#endif		
			enable_irq(ps_data->irq);
#endif		
    	}
	else
	{
		ps_data->als_enabled = false;
#ifdef STK_POLL_ALS			
		hrtimer_cancel(&ps_data->als_timer);
		cancel_work_sync(&ps_data->stk_als_work);
#else
#ifndef STK_POLL_PS
		if(!(ps_data->ps_enabled))	
#endif		
			disable_irq(ps_data->irq);		
#endif		
	}
//modify(add) by junfeng.zhou.sz for add power supply begin . 20140214
#ifdef POWER_REGULATOR
	if (!enable) {
		ret = stk3x1x_device_ctl(ps_data, enable);
		if (ret)
			return ret;
	}
#endif
//modify(add) by junfeng.zhou.sz for add power supply end . 
    return ret;
}

static int32_t stk3x1x_get_als_reading(struct stk3x1x_data *ps_data)
{
    	int32_t word_data;
#ifdef STK_ALS_FIR
	int index;   
	int firlen = atomic_read(&ps_data->firlength);   
#endif	
	unsigned char value[2];
	int ret;
	
	ret = stk3x1x_i2c_read_data(ps_data->client, STK_DATA1_ALS_REG, 2, &value[0]);
	if(ret < 0)
	{
		printk(KERN_ERR "%s fail, ret=0x%x\n", __func__, ret);
		return ret;
	}
	word_data = (value[0]<<8) | value[1];	
	
#ifdef STK_ALS_FIR
	if(ps_data->fir.number < firlen)
	{                
		ps_data->fir.raw[ps_data->fir.number] = word_data;
		ps_data->fir.sum += word_data;
		ps_data->fir.number++;
		ps_data->fir.idx++;
	}
	else
	{
		index = ps_data->fir.idx % firlen;
		ps_data->fir.sum -= ps_data->fir.raw[index];
		ps_data->fir.raw[index] = word_data;
		ps_data->fir.sum += word_data;
		ps_data->fir.idx++;
		word_data = ps_data->fir.sum/firlen;
	}	
#endif	
	
	return word_data;
}

static int32_t stk3x1x_set_irs_it_slp(struct stk3x1x_data *ps_data, uint16_t *slp_time)
{
	uint8_t irs_alsctrl;
	int32_t ret;
		
	irs_alsctrl = (ps_data->alsctrl_reg & 0x0F) - 2;		
	switch(irs_alsctrl)
	{
		case 6:
			*slp_time = 12;
			break;
		case 7:
			*slp_time = 24;			
			break;
		case 8:
			*slp_time = 48;			
			break;
		case 9:
			*slp_time = 96;			
			break;				
		default:
			printk(KERN_ERR "%s: unknown ALS IT=0x%x\n", __func__, irs_alsctrl);
			ret = -EINVAL;	
			return ret;
	}
	irs_alsctrl |= (ps_data->alsctrl_reg & 0xF0);
	ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client, STK_ALSCTRL_REG, irs_alsctrl);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;		
	}		
	return 0;
}

static int32_t stk3x1x_get_ir_reading(struct stk3x1x_data *ps_data)
{
    	int32_t word_data, ret;
	uint8_t w_reg, retry = 0;	
	uint16_t irs_slp_time = 100;
	bool re_enable_ps = false;
	unsigned char value[2];
	
	if(ps_data->ps_enabled)
	{
#ifdef STK_TUNE0		
		if (!(ps_data->psi_set))
		{
			hrtimer_cancel(&ps_data->ps_tune0_timer);					
			cancel_work_sync(&ps_data->stk_ps_tune0_work);
		}	
#endif		
		stk3x1x_enable_ps(ps_data, 0, 1);
		re_enable_ps = true;
	}
	
	ret = stk3x1x_set_irs_it_slp(ps_data, &irs_slp_time);
	if(ret < 0)
		goto irs_err_i2c_rw;
	
	ret = stk3x1x_get_state(ps_data);
	if(ret < 0)
		goto irs_err_i2c_rw;
	
	w_reg = ret | STK_STATE_EN_IRS_MASK;		
	ret = stk3x1x_set_state(ps_data, w_reg);
	if(ret < 0)
		goto irs_err_i2c_rw;
	msleep(irs_slp_time);	
	
	do
	{
		usleep_range(3000, 4000);
		//msleep(3);
		ret = stk3x1x_get_flag(ps_data);
		if (ret < 0)
			goto irs_err_i2c_rw;
		retry++;
	}while(retry < 10 && ((ret&STK_FLG_IR_RDY_MASK) == 0));
	
	if(retry == 10)
	{
		printk(KERN_ERR "%s: ir data is not ready for 300ms\n", __func__);
		ret = -EINVAL;
		goto irs_err_i2c_rw;
	}

	ret = stk3x1x_set_flag(ps_data, ret, STK_FLG_IR_RDY_MASK);
    	if (ret < 0)
		goto irs_err_i2c_rw;
	
	ret = stk3x1x_i2c_read_data(ps_data->client, STK_DATA1_IR_REG, 2, &value[0]);
	if(ret < 0)
	{
		printk(KERN_ERR "%s fail, ret=0x%x\n", __func__, ret);
		goto irs_err_i2c_rw;
	}
	word_data = ((value[0]<<8) | value[1]);	

	ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client, STK_ALSCTRL_REG, ps_data->alsctrl_reg );
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		goto irs_err_i2c_rw;
	}
	if(re_enable_ps)
		stk3x1x_enable_ps(ps_data, 1, 1);			
	return word_data;

irs_err_i2c_rw:	
	if(re_enable_ps)
		stk3x1x_enable_ps(ps_data, 1, 1);		
	return ret;
}

#ifdef STK_CHK_REG
static int stk3x1x_chk_reg_valid(struct stk3x1x_data *ps_data) 
{
	unsigned char value[9];
	int err;
	/*
	uint8_t cnt;
		
	for(cnt=0;cnt<9;cnt++)
	{
		value[cnt] = stk3x1x_i2c_smbus_read_byte_data(ps_data->client, (cnt+1));
		if(value[cnt] < 0)
		{
			printk(KERN_ERR "%s fail, ret=%d", __func__, value[cnt]);	
			return value[cnt];
		}
	}	
	*/
	err = stk3x1x_i2c_read_data(ps_data->client, STK_PSCTRL_REG, 9, &value[0]);
	if(err < 0)
	{
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, err);
		return err;
	}	

	if(value[0] != ps_data->psctrl_reg)
	{
		printk(KERN_ERR "%s: invalid reg 0x01=0x%2x\n", __func__, value[0]);
		return 0xFF;
	}	
	if(value[1] != ps_data->alsctrl_reg)
	{
		printk(KERN_ERR "%s: invalid reg 0x02=0x%2x\n", __func__, value[1]);
		return 0xFF;
	}
	if(value[2] != ps_data->ledctrl_reg)
	{
		printk(KERN_ERR "%s: invalid reg 0x03=0x%2x\n", __func__, value[2]);
		return 0xFF;
	}	
	if(value[3] != ps_data->int_reg)
	{
		printk(KERN_ERR "%s: invalid reg 0x04=0x%2x\n", __func__, value[3]);
		return 0xFF;
	}
	if(value[4] != ps_data->wait_reg)
	{
		printk(KERN_ERR "%s: invalid reg 0x05=0x%2x\n", __func__, value[4]);
		return 0xFF;
	}		
	if(value[5] != ((ps_data->ps_thd_h & 0xFF00) >> 8))
	{
		printk(KERN_ERR "%s: invalid reg 0x06=0x%2x\n", __func__, value[5]);
		return 0xFF;
	}		
	if(value[6] != (ps_data->ps_thd_h & 0x00FF))
	{
		printk(KERN_ERR "%s: invalid reg 0x07=0x%2x\n", __func__, value[6]);
		return 0xFF;
	}		
	if(value[7] != ((ps_data->ps_thd_l & 0xFF00) >> 8))
	{
		printk(KERN_ERR "%s: invalid reg 0x08=0x%2x\n", __func__, value[7]);
		return 0xFF;
	}		
	if(value[8] != (ps_data->ps_thd_l & 0x00FF))
	{
		printk(KERN_ERR "%s: invalid reg 0x09=0x%2x\n", __func__, value[8]);
		return 0xFF;
	}		
	
	return 0;
}

static int stk3x1x_validate_n_handle(struct i2c_client *client) 
{
	struct stk3x1x_data *ps_data = i2c_get_clientdata(client);
	int err;
	
	err = stk3x1x_chk_reg_valid(ps_data);
	if(err < 0)
	{
		printk(KERN_ERR "stk3x1x_chk_reg_valid fail: %d\n", err);        
		return err;
	}
	
	if(err == 0xFF)
	{		
		printk(KERN_ERR "%s: Re-init chip\n", __func__);				
		err = stk3x1x_software_reset(ps_data); 
		if(err < 0)
			return err;			
		err = stk3x1x_init_all_reg(ps_data);
		if(err < 0)
			return err;			
		
		//ps_data->psa = 0;
		//ps_data->psi = 0xFFFF;		
		stk3x1x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
		stk3x1x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);		
#ifdef STK_ALS_FIR
		memset(&ps_data->fir, 0x00, sizeof(ps_data->fir));  
#endif		
		return 0xFF;
	}
	return 0;
}
#endif /* #ifdef STK_CHK_REG */
static ssize_t stk_als_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);		
	int32_t reading;
	//reading = stk3x1x_get_als_reading(ps_data);
	unsigned char value[2];
	int ret;
	
	ret = stk3x1x_i2c_read_data(ps_data->client, STK_DATA1_ALS_REG, 2, &value[0]);
	if(ret < 0)
	{
		printk(KERN_ERR "%s fail, ret=0x%x\n", __func__, ret);
		return ret;
	}
	reading = (value[0]<<8) | value[1];
    	return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}

static int stk_als_enable_set(struct sensors_classdev *sensors_cdev,
						unsigned int enabled)
{
	struct stk3x1x_data *ps_data = container_of(sensors_cdev,
						struct stk3x1x_data, als_cdev);
	int err;

	mutex_lock(&ps_data->io_lock);
	err = stk3x1x_enable_als(ps_data, enabled);
	mutex_unlock(&ps_data->io_lock);

	if (err < 0)
		return err;
	return 0;
}

static ssize_t stk_als_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
    int32_t ret;
	
	ret = stk3x1x_get_state(ps_data);
	if(ret < 0)
		return ret;		
    ret = (ret & STK_STATE_EN_ALS_MASK)?1:0;
	
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);	
}

static ssize_t stk_als_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data = dev_get_drvdata(dev);
	uint8_t en;
	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else 
	{
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}	
    	printk(KERN_INFO "%s: Enable ALS : %d\n", __func__, en);
    	mutex_lock(&ps_data->io_lock);
    	stk3x1x_enable_als(ps_data, en);
    	mutex_unlock(&ps_data->io_lock);
    	return size;
}

static ssize_t stk_als_lux_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data = dev_get_drvdata(dev);
    	int32_t als_reading;
	uint32_t als_lux;
    	als_reading = stk3x1x_get_als_reading(ps_data);    
	als_lux = stk_alscode2lux(ps_data, als_reading);
    	return scnprintf(buf, PAGE_SIZE, "%d lux\n", als_lux);
}

static ssize_t stk_als_lux_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
	unsigned long value = 0;
	int ret;
	ret = strict_strtoul(buf, 16, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=0x%x\n", __func__, ret);	
		return ret;	
	}
    	ps_data->als_lux_last = value;
	//input_report_abs(ps_data->als_input_dev, ABS_MISC, value);
	//input_sync(ps_data->als_input_dev);
	stk3x1x_report_value(ps_data->als_input_dev, ABS_MISC, value);
	printk(KERN_INFO "%s: als input event %ld lux\n",__func__, value);	

    return size;
}


static ssize_t stk_als_transmittance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
    	int32_t transmittance;
    	transmittance = ps_data->als_transmittance;
    	return scnprintf(buf, PAGE_SIZE, "%d\n", transmittance);
}


static ssize_t stk_als_transmittance_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
	unsigned long value = 0;
	int ret;
	ret = strict_strtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	    
	}
    	ps_data->als_transmittance = value;
    	return size;
}

static ssize_t stk_als_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	int64_t delay;
	mutex_lock(&ps_data->io_lock);
	delay = ktime_to_ns(ps_data->als_poll_delay);
	mutex_unlock(&ps_data->io_lock);
	return scnprintf(buf, PAGE_SIZE, "%lld\n", delay);
}

static int stk_als_poll_delay_set(struct sensors_classdev *sensors_cdev,
						unsigned int delay_msec)
{
	struct stk3x1x_data *als_data = container_of(sensors_cdev,
						struct stk3x1x_data, als_cdev);
	uint64_t value = 0;

	value = delay_msec * 1000000;

	if (value < MIN_ALS_POLL_DELAY_NS)
		value = MIN_ALS_POLL_DELAY_NS;

	mutex_lock(&als_data->io_lock);
	if (value != ktime_to_ns(als_data->als_poll_delay))
		als_data->als_poll_delay = ns_to_ktime(value);

#ifdef STK_ALS_FIR		
	als_data->fir.number = 0;
	als_data->fir.idx = 0;
	als_data->fir.sum = 0;
#endif	

	mutex_unlock(&als_data->io_lock);

	return 0;
}

static ssize_t stk_als_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    	uint64_t value = 0;
	int ret;	
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
	ret = strict_strtoull(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:strict_strtoull failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}	
#ifdef STK_DEBUG_PRINTF		
	printk(KERN_INFO "%s: set als poll delay=%lld\n", __func__, value);
#endif	
	if(value < MIN_ALS_POLL_DELAY_NS)	
	{
		printk(KERN_ERR "%s: delay is too small\n", __func__);
		value = MIN_ALS_POLL_DELAY_NS;
	}
	mutex_lock(&ps_data->io_lock);
	if(value != ktime_to_ns(ps_data->als_poll_delay))
		ps_data->als_poll_delay = ns_to_ktime(value);	
#ifdef STK_ALS_FIR		
	ps_data->fir.number = 0;
	ps_data->fir.idx = 0;
	ps_data->fir.sum = 0;
#endif	
	mutex_unlock(&ps_data->io_lock);
	return size;
}

static ssize_t stk_als_ir_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);		
    	int32_t reading;
    	reading = stk3x1x_get_ir_reading(ps_data);
    	return scnprintf(buf, PAGE_SIZE, "%d\n", reading);	
}

#ifdef STK_ALS_FIR
static ssize_t stk_als_firlen_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	int len = atomic_read(&ps_data->firlength);
	
	printk(KERN_INFO "%s: len = %2d, idx = %2d\n", __func__, len, ps_data->fir.idx);			
	printk(KERN_INFO "%s: sum = %5d, ave = %5d\n", __func__, ps_data->fir.sum, ps_data->fir.sum/len);
	
	return scnprintf(buf, PAGE_SIZE, "%d\n", len);		
}


static ssize_t stk_als_firlen_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    	uint64_t value = 0;	
	int ret;	
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
	ret = strict_strtoull(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:strict_strtoull failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}	
	
	if(value > MAX_FIR_LEN)
	{
		printk(KERN_ERR "%s: firlen exceed maximum filter length\n", __func__);
	}
	else if (value < 1)
	{
		atomic_set(&ps_data->firlength, 1);
		memset(&ps_data->fir, 0x00, sizeof(ps_data->fir));
	}
	else
	{ 
		atomic_set(&ps_data->firlength, value);
		memset(&ps_data->fir, 0x00, sizeof(ps_data->fir));
	}	
	return size;	
}
#endif  /* #ifdef STK_ALS_FIR */

#ifdef STK_GES		
static ssize_t stk_ges_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
    	int ret;
	unsigned int gest0 = 0, gest1 = 0, gest2 = 0;

    	ret = stk3x1x_get_ges_reading(ps_data, &gest0, &gest1, &gest2);
	if(ret < 0)
		return ret;
	else if(ret == 0xFFFF)
		atomic_set(&ps_data->gesture2, 0);
		
    	return scnprintf(buf, PAGE_SIZE, "%5d,%5d,%5d\n", gest0, gest1, atomic_read(&ps_data->gesture2));
}

static ssize_t stk_ges_code_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	uint8_t ges;
	unsigned long value = 0;
	int ret;
	
	ret = strict_strtoul(buf, 16, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=%d\n", __func__, ret);	
		return ret;	
	}	
	
	switch(value)
	{
	case 3:
		//printk(KERN_INFO "%s: ges input event, not detected\n",__func__);			
	case 0:
		return size;
	case 1:
		ges = KEY_PAGEUP;	
		atomic_set(&ps_data->gesture2, 0);
		printk(KERN_INFO "%s: ges input event >>>\n",__func__);		
		break;
	case 2:
		ges = KEY_PAGEDOWN;	
		atomic_set(&ps_data->gesture2, 0);
		printk(KERN_INFO "%s: ges input event <<<\n",__func__);		
		break;
	case 32:
		ges = KEY_VOLUMEDOWN;
		printk(KERN_INFO "%s: ges input event near\n",__func__);		
		break;
	case 48:
		ges = KEY_VOLUMEUP;
		printk(KERN_INFO "%s: ges input event far\n",__func__);		
		break;
	default:
		printk(KERN_ERR "%s, invalid value %d\n", __func__, (int)value);
		return -EINVAL;
	}
	
	input_report_key(ps_data->ges_input_dev, ges, 1);
	input_report_key(ps_data->ges_input_dev, ges, 0);
	input_sync(ps_data->ges_input_dev);
    	return size;
}

static ssize_t stk_ges_poll_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len = 0, ii = 0, jj = 0;
	
	while(stk_ges_op[ii].ops[0] != 0)
	{
		len += scnprintf(buf + len, PAGE_SIZE - len, "%x ", ii);
		for(jj=0;jj<4;jj++)
			len += scnprintf(buf + len, PAGE_SIZE - len, "%x ", stk_ges_op[ii].ops[jj]);
		len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
		ii++;
	}
	return len;
}


static ssize_t stk_ges_poll_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int32_t ret, i = 0, index = 0;	
	char *token;
	unsigned long value = 0;
		
	while(buf != '\0')
	{
		token = strsep((char **)&buf, " ");
		if((ret = strict_strtoul(token, 16, &value)) < 0)
		{
			printk(KERN_ERR "%s:strict_strtoul failed, ret=0x%x\n", __func__, ret);
			return ret;	
		}
		
		if(i == 0)
		{
			if(value >= 10)
			{
				memset(stk_ges_op, 0, sizeof(stk_ges_op));				
				break;
			}
			else
				index = value;
		}
		else
		{
			stk_ges_op[index].ops[i-1] = value;
		}
		i++;
		if(i == 5)
			break;
	}
	if(i != 5)
	{
		printk(KERN_ERR "%s: invalid length(%d)\n", __func__, i);
		memset(&(stk_ges_op[index]), 0, sizeof(union stk_ges_operation));				
	}
	return size;
}
		
static ssize_t stk_ges_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
		
	ret = strict_strtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=%d\n", __func__, ret);	
		return ret;	
	}	
    	printk(KERN_INFO "%s: Enable GES : %d\n", __func__, (int)value);	
	
	switch(value)
	{
	case 0:
		mutex_lock(&ps_data->io_lock);
		if(ps_data->ges_enabled == 1)
			stk3x1x_enable_ges(ps_data, 0, 1);
		else
			stk3x1x_enable_ges(ps_data, 0, 2);	
		mutex_unlock(&ps_data->io_lock);		
		break;
	case 1:
		mutex_lock(&ps_data->io_lock);
		stk3x1x_enable_ges(ps_data, 1, 1);
		mutex_unlock(&ps_data->io_lock);		
		break;
	case 2:
		mutex_lock(&ps_data->io_lock);
		stk3x1x_enable_ges(ps_data, 1, 2);
		mutex_unlock(&ps_data->io_lock);		
		break;
	default:
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;	
		break;
	}

    	return size;
}

static ssize_t stk_ges_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%d\n", ps_data->ges_enabled);		
}

#endif	/* #ifdef STK_GES */

static ssize_t stk_ps_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
    	uint32_t reading;
    	reading = stk3x1x_get_ps_reading(ps_data);
    	return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}

static ssize_t stk_ps_prx_raw_off_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
    	uint32_t reading;
    	reading = stk3x1x_get_ps_reading(ps_data);
    	return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}

static ssize_t stk_ps_prx_raw_org_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
    	uint32_t reading;
    	reading = stk3x1x_get_ps_reading(ps_data);
    	return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}


static int stk_ps_poll_delay_set(struct sensors_classdev *sensors_cdev,
						unsigned int delay_msec)
{
	return 0;
}

static int stk_ps_enable_set(struct sensors_classdev *sensors_cdev,
						unsigned int enabled)
{
	struct stk3x1x_data *ps_data = container_of(sensors_cdev,
						struct stk3x1x_data, ps_cdev);
	int err;

	mutex_lock(&ps_data->io_lock);
	err = stk3x1x_enable_ps(ps_data, enabled, 1);
	mutex_unlock(&ps_data->io_lock);

	if (err < 0)
		return err;
	return 0;
}

static ssize_t stk_ps_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    	int32_t ret;
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);

	ret = stk3x1x_get_state(ps_data);
	if(ret < 0)
		return ret;	
    	ret = (ret & STK_STATE_EN_PS_MASK)?1:0;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);	
}

static ssize_t stk_ps_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	uint8_t en;
	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else 
	{
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}
    	printk(KERN_INFO "%s: Enable PS : %d\n", __func__, en);
    	mutex_lock(&ps_data->io_lock);
    	stk3x1x_enable_ps(ps_data, en, 1);
    	mutex_unlock(&ps_data->io_lock);
    	return size;
}

static ssize_t stk_ps_enable_aso_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    	int32_t ret;
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	
    	ret = stk3x1x_i2c_smbus_read_byte_data(ps_data->client,STK_STATE_REG);
    	ret = (ret & STK_STATE_EN_ASO_MASK)?1:0;
	
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);		
}

static ssize_t stk_ps_enable_aso_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	uint8_t en;
    int32_t ret;
	uint8_t w_state_reg;
	
	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else 
	{
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}
    printk(KERN_INFO "%s: Enable PS ASO : %d\n", __func__, en);
    
    ret = stk3x1x_i2c_smbus_read_byte_data(ps_data->client, STK_STATE_REG);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
    }		
	w_state_reg = (uint8_t)(ret & (~STK_STATE_EN_ASO_MASK)); 
	if(en)	
		w_state_reg |= STK_STATE_EN_ASO_MASK;	
	
    ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, w_state_reg);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}	
	
	return size;	
}


static ssize_t stk_ps_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
    int32_t word_data;		
	unsigned char value[2];
	int ret;
	
	ret = stk3x1x_i2c_read_data(ps_data->client, STK_DATA1_OFFSET_REG, 2, &value[0]);
	if(ret < 0)
	{
		printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
		return ret;
	}
	word_data = (value[0]<<8) | value[1];					
		
	return scnprintf(buf, PAGE_SIZE, "%d\n", word_data);
}
 
static ssize_t stk_ps_offset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
	unsigned long offset = 0;
	int ret;
	unsigned char val[2];
	
	ret = strict_strtoul(buf, 10, &offset);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=0x%x\n", __func__, ret);	
		return ret;	
	}
	if(offset > 65535)
	{
		printk(KERN_ERR "%s: invalid value, offset=%ld\n", __func__, offset);
		return -EINVAL;
	}
	
	val[0] = (offset & 0xFF00) >> 8;
	val[1] = offset & 0x00FF;
	ret = stk3x1x_i2c_write_data(ps_data->client, STK_DATA1_OFFSET_REG, 2, val);	
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}
	
	return size;
}


static ssize_t stk_ps_distance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
    	int32_t dist=1, ret;

    	ret = stk3x1x_get_flag(ps_data);
	if(ret < 0)
		return ret;
    	dist = (ret & STK_FLG_NF_MASK)?1:0;	
	
    	ps_data->ps_distance_last = dist;
	//input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, dist);
	//input_sync(ps_data->ps_input_dev);
	stk3x1x_report_value(ps_data->ps_input_dev, ABS_DISTANCE, dist);
	wake_lock_timeout(&ps_data->ps_wakelock, HZ/2);
	printk(KERN_INFO "%s: ps input event %d cm\n",__func__, dist);		
    	return scnprintf(buf, PAGE_SIZE, "%d\n", dist);
}


static ssize_t stk_ps_distance_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
	unsigned long value = 0;
	int ret;
	ret = strict_strtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=0x%x\n", __func__, ret);	
		return ret;	
	}
    	ps_data->ps_distance_last = value;
	//input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, value);
	//input_sync(ps_data->ps_input_dev);
	stk3x1x_report_value(ps_data->ps_input_dev, ABS_DISTANCE, value);
	wake_lock_timeout(&ps_data->ps_wakelock, HZ/2);	
	printk(KERN_INFO "%s: ps input event %ld cm\n",__func__, value);	
    	return size;
}


static ssize_t stk_ps_code_thd_l_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    	int32_t ps_thd_l1_reg, ps_thd_l2_reg;
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
    	ps_thd_l1_reg = stk3x1x_i2c_smbus_read_byte_data(ps_data->client,STK_THDL1_PS_REG);
    	if(ps_thd_l1_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x\n", __func__, ps_thd_l1_reg);		
		return -EINVAL;		
	}
    	ps_thd_l2_reg = stk3x1x_i2c_smbus_read_byte_data(ps_data->client,STK_THDL2_PS_REG);
    	if(ps_thd_l2_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x\n", __func__, ps_thd_l2_reg);		
		return -EINVAL;		
	}
	ps_thd_l1_reg = ps_thd_l1_reg<<8 | ps_thd_l2_reg;
    	return scnprintf(buf, PAGE_SIZE, "%d\n", ps_thd_l1_reg);
}


static ssize_t stk_ps_code_thd_l_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);		
	unsigned long value = 0;
	int ret;
	ret = strict_strtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	    
	}
    	stk3x1x_set_ps_thd_l(ps_data, value);
    	return size;
}

static ssize_t stk_ps_code_thd_h_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    	int32_t ps_thd_h1_reg, ps_thd_h2_reg;
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
    	ps_thd_h1_reg = stk3x1x_i2c_smbus_read_byte_data(ps_data->client,STK_THDH1_PS_REG);
    	if(ps_thd_h1_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x\n", __func__, ps_thd_h1_reg);		
		return -EINVAL;		
	}
    	ps_thd_h2_reg = stk3x1x_i2c_smbus_read_byte_data(ps_data->client,STK_THDH2_PS_REG);
    	if(ps_thd_h2_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x\n", __func__, ps_thd_h2_reg);		
		return -EINVAL;		
	}
	ps_thd_h1_reg = ps_thd_h1_reg<<8 | ps_thd_h2_reg;
    	return scnprintf(buf, PAGE_SIZE, "%d\n", ps_thd_h1_reg);
}


static ssize_t stk_ps_code_thd_h_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);		
	unsigned long value = 0;
	int ret;
	ret = strict_strtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	    
	}
    	stk3x1x_set_ps_thd_h(ps_data, value);
    	return size;
}


static ssize_t stk_all_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    	int32_t ps_reg[0x22];
	uint8_t cnt;
	int len = 0;
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
	
	for(cnt=0;cnt<0x20;cnt++)
	{
		ps_reg[cnt] = stk3x1x_i2c_smbus_read_byte_data(ps_data->client, (cnt));
		if(ps_reg[cnt] < 0)
		{
			printk(KERN_ERR "%s fail, ret=%d\n", __func__, ps_reg[cnt]);	
			return -EINVAL;
		}
		else
		{
			printk(KERN_INFO "reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
			len += scnprintf(buf+len, PAGE_SIZE-len, "[%2X]%2X,", cnt, ps_reg[cnt]);
		}
	}
	ps_reg[cnt] = stk3x1x_i2c_smbus_read_byte_data(ps_data->client, STK_PDT_ID_REG);
	if(ps_reg[cnt] < 0)
	{
		printk( KERN_ERR "%s fail, ret=%d\n", __func__, ps_reg[cnt]);	
		return -EINVAL;
	}
	printk( KERN_INFO "reg[0x%x]=0x%2X\n", STK_PDT_ID_REG, ps_reg[cnt]);	
	cnt++;
	ps_reg[cnt] = stk3x1x_i2c_smbus_read_byte_data(ps_data->client, STK_RSRVD_REG);
	if(ps_reg[cnt] < 0)
	{
		printk( KERN_ERR "%s fail, ret=%d\n", __func__, ps_reg[cnt]);	
		return -EINVAL;
	}
	printk( KERN_INFO "reg[0x%x]=0x%2X\n", STK_RSRVD_REG, ps_reg[cnt]);		
	len += scnprintf(buf+len, PAGE_SIZE-len, "[%2X]%2X,[%2X]%2X\n", cnt-1, ps_reg[cnt-1], cnt, ps_reg[cnt]);
	return len;
/*
    return scnprintf(buf, PAGE_SIZE, "[0]%2X [1]%2X [2]%2X [3]%2X [4]%2X [5]%2X [6/7 HTHD]%2X,%2X [8/9 LTHD]%2X, %2X [A]%2X [B]%2X [C]%2X [D]%2X [E/F Aoff]%2X,%2X,[10]%2X [11/12 PS]%2X,%2X [13]%2X [14]%2X [15/16 Foff]%2X,%2X [17]%2X [18]%2X [3E]%2X [3F]%2X\n", 	
		ps_reg[0], ps_reg[1], ps_reg[2], ps_reg[3], ps_reg[4], ps_reg[5], ps_reg[6], ps_reg[7], ps_reg[8], 
		ps_reg[9], ps_reg[10], ps_reg[11], ps_reg[12], ps_reg[13], ps_reg[14], ps_reg[15], ps_reg[16], ps_reg[17], 
		ps_reg[18], ps_reg[19], ps_reg[20], ps_reg[21], ps_reg[22], ps_reg[23], ps_reg[24], ps_reg[25], ps_reg[26]);
		*/
}

static ssize_t stk_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    	int32_t ps_reg[27];
	uint8_t cnt;
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
	for(cnt=0;cnt<25;cnt++)
	{
		ps_reg[cnt] = stk3x1x_i2c_smbus_read_byte_data(ps_data->client, (cnt));
		if(ps_reg[cnt] < 0)
		{
			printk(KERN_ERR "%s fail, ret=%d\n", __func__, ps_reg[cnt]);	
			return -EINVAL;
		}
		else
		{
			printk(KERN_INFO "reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
		}
	}
	ps_reg[cnt] = stk3x1x_i2c_smbus_read_byte_data(ps_data->client, STK_PDT_ID_REG);
	if(ps_reg[cnt] < 0)
	{
		printk( KERN_ERR "%s fail, ret=%d\n", __func__, ps_reg[cnt]);	
		return -EINVAL;
	}
	printk( KERN_INFO "reg[0x%x]=0x%2X\n", STK_PDT_ID_REG, ps_reg[cnt]);	
	cnt++;
	ps_reg[cnt] = stk3x1x_i2c_smbus_read_byte_data(ps_data->client, STK_RSRVD_REG);
	if(ps_reg[cnt] < 0)
	{
		printk( KERN_ERR "%s fail, ret=%d\n", __func__, ps_reg[cnt]);	
		return -EINVAL;
	}
	printk( KERN_INFO "reg[0x%x]=0x%2X\n", STK_RSRVD_REG, ps_reg[cnt]);		

    return scnprintf(buf, PAGE_SIZE, "[PS=%2X] [ALS=%2X] [WAIT=0x%4Xms] [EN_ASO=%2X] [EN_AK=%2X] [NEAR/FAR=%2X] [FLAG_OUI=%2X] [FLAG_PSINT=%2X] [FLAG_ALSINT=%2X]\n", 
		ps_reg[0]&0x01,(ps_reg[0]&0x02)>>1,((ps_reg[0]&0x04)>>2)*ps_reg[5]*6,(ps_reg[0]&0x20)>>5,
		(ps_reg[0]&0x40)>>6,ps_reg[16]&0x01,(ps_reg[16]&0x04)>>2,(ps_reg[16]&0x10)>>4,(ps_reg[16]&0x20)>>5);		
}

static ssize_t stk_recv_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&ps_data->recv_reg));     		
}


static ssize_t stk_recv_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    	unsigned long value = 0;
	int ret;
	int32_t recv_data;	
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
	
	if((ret = strict_strtoul(buf, 16, &value)) < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	recv_data = stk3x1x_i2c_smbus_read_byte_data(ps_data->client,value);
//	printk("%s: reg 0x%x=0x%x\n", __func__, (int)value, recv_data);
	atomic_set(&ps_data->recv_reg, recv_data);
	return size;
}


static ssize_t stk_send_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}


static ssize_t stk_send_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int addr, cmd;
	int32_t ret, i;
	char *token[10];
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
	
	for (i = 0; i < 2; i++)
		token[i] = strsep((char **)&buf, " ");
	if((ret = strict_strtoul(token[0], 16, (unsigned long *)&(addr))) < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	if((ret = strict_strtoul(token[1], 16, (unsigned long *)&(cmd))) < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	printk(KERN_INFO "%s: write reg 0x%x=0x%x\n", __func__, addr, cmd);		

	ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client, (unsigned char)addr, (unsigned char)cmd);
	if (0 != ret)
	{	
		printk(KERN_ERR "%s: stk3x1x_i2c_smbus_write_byte_data fail\n", __func__);
		return ret;
	}
	
	return size;
}

#ifdef STK_TUNE0

static ssize_t stk_ps_cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
	int32_t word_data;	
	unsigned char value[2];
	int ret;
	
	ret = stk3x1x_i2c_read_data(ps_data->client, 0x20, 2, &value[0]);
	if(ret < 0)
	{
		printk(KERN_ERR "%s fail, ret=0x%x\n", __func__, ret);
		return ret;
	}
	word_data = (value[0]<<8) | value[1];	
	
	ret = stk3x1x_i2c_read_data(ps_data->client, 0x22, 2, &value[0]);
	if(ret < 0)
	{
		printk(KERN_ERR "%s fail, ret=0x%x\n", __func__, ret);
		return ret;
	}
	word_data += ((value[0]<<8) | value[1]);		

#ifdef CALI_EVERY_TIME
	printk(KERN_INFO "%s: boot HT=%d, LT=%d\n", __func__, ps_data->ps_high_thd_boot, ps_data->ps_low_thd_boot);
#endif
	
	printk("%s: psi_set=%d, psa=%d,psi=%d, word_data=%d\n", __func__, 
		ps_data->psi_set, ps_data->psa, ps_data->psi, word_data);	
	
	return 0;
}

static ssize_t stk_mmi_prx_thd_h_limit_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	
	return snprintf(buf, PAGE_SIZE, "%d\n", ps_data->pdata->mmi_thd_h_limit);	
}

static ssize_t stk_mmi_prx_thd_h_limit_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long value = 0;
	int ret;
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	
	ret = strict_strtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	    
	}
	
	ps_data->pdata->mmi_thd_h_limit = value;
	
	return size;
}

static ssize_t stk_mmi_prx_thd_l_limit_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	
	return snprintf(buf, PAGE_SIZE, "%d\n", ps_data->pdata->mmi_thd_l_limit);	
}

static ssize_t stk_mmi_prx_thd_l_limit_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long value = 0;
	int ret;
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	
	ret = strict_strtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	    
	}
	
	ps_data->pdata->mmi_thd_l_limit = value;
	
	return size;
}

static ssize_t stk_trace_param_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t stk_trace_param_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);

	printk("%s:%d\n", __func__, size);
	memcpy(&ps_data->factory_data, buf, sizeof(struct trace_data));

	if (ps_data->factory_data.tag == TRACE_TAG) {
		if (ps_data->factory_data.factory_def_thd_offset.tag == DEF_TH_OFFSET_TAG) {
			ps_data->pdata->low_ps_ct_h_offset = ps_data->factory_data.factory_def_thd_offset.low_ps_cali_th_offset_h;
			ps_data->pdata->low_ps_ct_l_offset = ps_data->factory_data.factory_def_thd_offset.low_ps_cali_th_offset_l;
			ps_data->pdata->hi_ps_ct_h_offset  = ps_data->factory_data.factory_def_thd_offset.hi_ps_cali_th_offset_h;
			ps_data->pdata->hi_ps_ct_l_offset  = ps_data->factory_data.factory_def_thd_offset.hi_ps_cali_th_offset_l;

			ps_data->is_have_good_cali = false;
			ps_data->last_good_psi = 0xffff;
			ps_data->min_psi = 0xffff;

			printk("trace:low_ps_ct_h_off=%d,low_ps_ct_l_off=%d,hi_ps_ct_h_off=%d,hi_ps_ct_l_off=%d\n", \
			ps_data->pdata->low_ps_ct_h_offset, ps_data->pdata->low_ps_ct_l_offset, \
			ps_data->pdata->hi_ps_ct_h_offset, ps_data->pdata->hi_ps_ct_l_offset);
		}

		if (ps_data->factory_data.factory_cali_data.tag == CALI_DATA_TAG) {
			ps_data->psctrl_reg = ps_data->factory_data.factory_cali_data.psctrl;
			ps_data->ledctrl_reg = ps_data->factory_data.factory_cali_data.ledctrl;

			ps_data->is_have_good_cali = false;
			ps_data->last_good_psi = 0xffff;
			ps_data->min_psi = 0xffff;

			printk("Get psensor cali data from trace,psctrl_reg=%02x,ledctrl_reg=%02x\n", ps_data->psctrl_reg, ps_data->ledctrl_reg);
		}

		if (ps_data->factory_data.factory_def_thd_limit.tag == DEF_TH_LIMIT_TAG) {
			ps_data->pdata->mmi_thd_h_limit = ps_data->factory_data.factory_def_thd_limit.mmi_thd_h_limit;
			ps_data->pdata->mmi_thd_l_limit = ps_data->factory_data.factory_def_thd_limit.mmi_thd_l_limit;
			printk("Get psensor judge faulty thredhold from trace:H=%d,L=%d\n", ps_data->pdata->mmi_thd_h_limit, ps_data->pdata->mmi_thd_l_limit);
		}

		if (ps_data->factory_data.factory_def_mmi_data.tag == DEF_MMI_DATA) {
			printk("Get psensor ramdata from trace:%d\n", ps_data->factory_data.factory_def_mmi_data.mmi_data_rawdata);
		}
	}
	
	return size;
}


#endif	/* #ifdef STK_TUNE0 */

static struct device_attribute als_enable_attribute = __ATTR(enable,0664,stk_als_enable_show,stk_als_enable_store);
static struct device_attribute als_lux_attribute = __ATTR(lux,0664,stk_als_lux_show,stk_als_lux_store);
static struct device_attribute als_code_attribute = __ATTR(code, 0444, stk_als_code_show, NULL);
static struct device_attribute als_transmittance_attribute = __ATTR(transmittance,0664,stk_als_transmittance_show,stk_als_transmittance_store);
static struct device_attribute als_poll_delay_attribute = __ATTR(delay,0664,stk_als_delay_show,stk_als_delay_store);
static struct device_attribute als_ir_code_attribute = __ATTR(ircode,0444,stk_als_ir_code_show,NULL);
#ifdef STK_ALS_FIR
static struct device_attribute als_firlen_attribute = __ATTR(firlen,0664,stk_als_firlen_show,stk_als_firlen_store);
#endif

static struct attribute *stk_als_attrs [] =
{
	&als_enable_attribute.attr,
	&als_lux_attribute.attr,
	&als_code_attribute.attr,
    	&als_transmittance_attribute.attr,
	&als_poll_delay_attribute.attr,
	&als_ir_code_attribute.attr,
#ifdef STK_ALS_FIR
	&als_firlen_attribute.attr,
#endif	
    	NULL
};

static struct attribute_group stk_als_attribute_group = {
	//.name = "driver",
	.attrs = stk_als_attrs,
};

#ifdef STK_GES		
static struct device_attribute ges_enable_attribute = __ATTR(enable,0664,stk_ges_enable_show,stk_ges_enable_store);
static struct device_attribute ges_code_attribute = __ATTR(code, 0664, stk_ges_code_show, stk_ges_code_store);
static struct device_attribute ges_poll_attribute = __ATTR(poll, 0664, stk_ges_poll_show, stk_ges_poll_store);
static struct device_attribute ges_recv_attribute = __ATTR(recv,0664,stk_recv_show,stk_recv_store);
static struct device_attribute ges_send_attribute = __ATTR(send,0664,stk_send_show, stk_send_store);

static struct attribute *stk_ges_attrs [] =
{
    	&ges_enable_attribute.attr,	
    	&ges_code_attribute.attr,
    	&ges_poll_attribute.attr,
	&ges_recv_attribute.attr,
	&ges_send_attribute.attr,
    	NULL
};

static struct attribute_group stk_ges_attribute_group = 
{
	//.name = "driver",	
	.attrs = stk_ges_attrs,
};
#endif	/* #ifdef STK_GES */

static struct device_attribute ps_enable_attribute = __ATTR(enable,0664,stk_ps_enable_show,stk_ps_enable_store);
static struct device_attribute ps_enable_aso_attribute = __ATTR(enableaso,0664,stk_ps_enable_aso_show,stk_ps_enable_aso_store);
static struct device_attribute ps_distance_attribute = __ATTR(distance,0664,stk_ps_distance_show, stk_ps_distance_store);
static struct device_attribute ps_offset_attribute = __ATTR(prx_offset,0664,stk_ps_offset_show, stk_ps_offset_store);
static struct device_attribute ps_code_attribute = __ATTR(code, 0444, stk_ps_code_show, NULL);
static struct device_attribute ps_prx_raw_off_attribute = __ATTR(prx_raw_off, 0444, stk_ps_prx_raw_off_show, NULL);
static struct device_attribute ps_prx_raw_org_attribute = __ATTR(prx_raw_org, 0444, stk_ps_prx_raw_org_show, NULL);
static struct device_attribute ps_code_thd_l_attribute = __ATTR(codethdl,0664,stk_ps_code_thd_l_show,stk_ps_code_thd_l_store);
static struct device_attribute ps_code_thd_h_attribute = __ATTR(codethdh,0664,stk_ps_code_thd_h_show,stk_ps_code_thd_h_store);
static struct device_attribute ps_recv_attribute = __ATTR(recv,0664,stk_recv_show,stk_recv_store);
static struct device_attribute ps_send_attribute = __ATTR(send,0664,stk_send_show, stk_send_store);
static struct device_attribute all_reg_attribute = __ATTR(allreg, 0444, stk_all_reg_show, NULL);
static struct device_attribute status_attribute = __ATTR(status, 0444, stk_status_show, NULL);
static struct device_attribute mmi_prx_thd_h_limit_attribute = __ATTR(mmi_prx_thd_h_limit, 0664, stk_mmi_prx_thd_h_limit_show, stk_mmi_prx_thd_h_limit_store);
static struct device_attribute mmi_prx_thd_l_limit_attribute = __ATTR(mmi_prx_thd_l_limit, 0664, stk_mmi_prx_thd_l_limit_show, stk_mmi_prx_thd_l_limit_store);
static struct device_attribute trace_param_attribute = __ATTR(trace_param, 0664, stk_trace_param_show, stk_trace_param_store);
#ifdef STK_TUNE0
static struct device_attribute ps_cali_attribute = __ATTR(cali,0444,stk_ps_cali_show, NULL);
#endif

static struct attribute *stk_ps_attrs [] =
{
    	&ps_enable_attribute.attr,
    	&ps_enable_aso_attribute.attr,
    	&ps_distance_attribute.attr,
	&ps_offset_attribute.attr,
    	&ps_code_attribute.attr,
    	&ps_prx_raw_off_attribute.attr,
	&ps_prx_raw_org_attribute.attr,
	&ps_code_thd_l_attribute.attr,
	&ps_code_thd_h_attribute.attr,	
	&ps_recv_attribute.attr,
	&ps_send_attribute.attr,	
	&all_reg_attribute.attr,
	&status_attribute.attr,
	&mmi_prx_thd_h_limit_attribute.attr,
	&mmi_prx_thd_l_limit_attribute.attr,
	&trace_param_attribute.attr,

#ifdef STK_TUNE0
	&ps_cali_attribute.attr,
#endif	
    NULL
};

static struct attribute_group stk_ps_attribute_group = {
	//.name = "driver",	
	.attrs = stk_ps_attrs,
};

static int stk_ps_val(struct stk3x1x_data *ps_data)
{
	int mode;
	int32_t word_data, lii;	
	unsigned char value[4];
	int ret;
	
	ret = stk3x1x_i2c_read_data(ps_data->client, 0x20, 4, value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s fail, ret=0x%x\n", __func__, ret);
		return ret;
	}
	word_data = (value[0]<<8) | value[1];	
	word_data += ((value[2]<<8) | value[3]);	
	
	mode = (ps_data->psctrl_reg) & 0x3F;
	if(mode == 0x30)	
		lii = 100;	
	else if (mode == 0x31)
		lii = 200;		
	else if (mode == 0x32)
		lii = 400;		
	else if (mode == 0x33)
		lii = 800;	
	else
	{
		printk(KERN_ERR "%s: unsupported PS_IT(0x%x)\n", __func__, mode);
		return -1;
	}
	
	if(word_data > lii)	
	{
		printk(KERN_INFO "%s: word_data=%d, lii=%d\n", __func__, word_data, lii);	
		return 0xFFFF;	
	}
	return 0;
}	

#ifdef STK_TUNE0	

static int stk_ps_tune_zero_final(struct stk3x1x_data *ps_data)
{
	int ret;
	
	ps_data->tune_zero_init_proc = false;
	ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client, STK_INT_REG, ps_data->int_reg);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}	
	
	ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, 0);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}		
	
	if(ps_data->data_count == -1)
	{
		printk(KERN_INFO "%s: exceed limit\n", __func__);
		hrtimer_cancel(&ps_data->ps_tune0_timer);	
		
		ps_data->ps_thd_h = ps_data->ps_high_thd_def;
		ps_data->ps_thd_l = ps_data->ps_low_thd_def;
		ps_data->ps_high_thd_boot = ps_data->ps_thd_h;
		ps_data->ps_low_thd_boot = ps_data->ps_thd_l;
#ifdef TUNE_LARGE_DATA
		ps_data->ps_thd_h = 1700;
		ps_data->ps_thd_l = 1500;
#endif
		stk3x1x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
		stk3x1x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
		
		return 0;
	}
	
	ps_data->psa = ps_data->ps_stat_data[0];
	ps_data->psi = ps_data->ps_stat_data[2];		

#ifndef CALI_EVERY_TIME	
	ps_data->ps_thd_h = ps_data->ps_stat_data[1] + STK_HT_N_CT;
	ps_data->ps_thd_l = ps_data->ps_stat_data[1] + STK_LT_N_CT;		
#else
	/*ps_data->ps_thd_h = ps_data->ps_stat_data[1] + STK_HT_N_CT*2;
	ps_data->ps_thd_l = ps_data->ps_stat_data[1] + STK_LT_N_CT*2;*/
	ps_data->ps_thd_h = ps_data->ps_high_thd_def;
	ps_data->ps_thd_l = ps_data->ps_low_thd_def;
	ps_data->ps_high_thd_boot = ps_data->ps_thd_h;
	ps_data->ps_low_thd_boot = ps_data->ps_thd_l;
#endif
#ifdef TUNE_LARGE_DATA
	ps_data->ps_thd_h = 1700;
	ps_data->ps_thd_l = 1500;	
#endif	
	stk3x1x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
	stk3x1x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);				
	printk(KERN_INFO "%s: set HT=%d,LT=%d\n", __func__, ps_data->ps_thd_h,  ps_data->ps_thd_l);		
	hrtimer_cancel(&ps_data->ps_tune0_timer);					
	return 0;
}
	
static int32_t stk_tune_zero_get_ps_data(struct stk3x1x_data *ps_data)
{
	uint32_t ps_adc;
	int ret;
	
	ret = stk_ps_val(ps_data);	
	if(ret == 0xFFFF)
	{		
		ps_data->data_count = -1;
		stk_ps_tune_zero_final(ps_data);
		return 0;
	}	
	
	ps_adc = stk3x1x_get_ps_reading(ps_data);
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 begin . 20140115
#ifdef STK_DEBUG_PRINTF	
	printk(KERN_INFO "%s: ps_adc #%d=%d\n", __func__, ps_data->data_count, ps_adc);
#endif
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 end . 
	if(ps_adc < 0)		
		return ps_adc;		
	
	ps_data->ps_stat_data[1]  +=  ps_adc;			
	if(ps_adc > ps_data->ps_stat_data[0])
		ps_data->ps_stat_data[0] = ps_adc;
	if(ps_adc < ps_data->ps_stat_data[2])
		ps_data->ps_stat_data[2] = ps_adc;						
	ps_data->data_count++;	
	
	if(ps_data->data_count == 5)
	{
		ps_data->ps_stat_data[1]  /= ps_data->data_count;
		ps_data->min_psi = ps_data->ps_stat_data[1];	
		printk("%s:min_psi=%d", __func__, ps_data->min_psi);	
		stk_ps_tune_zero_final(ps_data);
	}		
	
	return 0;
}

static int stk_ps_tune_zero_init(struct stk3x1x_data *ps_data)
{
	int32_t ret = 0;
	uint8_t w_state_reg;	
	
	ps_data->psi_set = 0;	
	ps_data->tune_zero_init_proc = true;		
	ps_data->ps_stat_data[0] = 0;
	ps_data->ps_stat_data[2] = 9999;
	ps_data->ps_stat_data[1] = 0;
	ps_data->data_count = 0;
	
	ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client, STK_INT_REG, 0);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}			
	
	w_state_reg = (STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK);			
	ret = stk3x1x_i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, w_state_reg);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}		
	hrtimer_start(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay, HRTIMER_MODE_REL);		
	return 0;	
}

static int stk_ps_tune_zero_func_fae(struct stk3x1x_data *ps_data)
{
	int32_t word_data;
	int ret, diff, is_cali = 0;
	unsigned char value[2];
	static int debouse_cnt = 0;
//modify(add) by junfeng.zhou.sz for cali the psensor everytime call with 533451 begin . 20140115	
#ifndef CALI_EVERY_TIME		
	if(ps_data->psi_set || !(ps_data->ps_enabled))
#else
	if(!(ps_data->ps_enabled))
#endif
	{
		return 0;
	}	
//modify(add) by junfeng.zhou.sz for cali the psensor everytime call  with 533451 end . 
	ret = stk3x1x_get_flag(ps_data);
	if(ret < 0)
		return ret;
	if(!(ret&STK_FLG_PSDR_MASK))
	{
		//printk(KERN_INFO "%s: ps data is not ready yet\n", __func__);
		return 0;
	}
	
	ret = stk_ps_val(ps_data);	
	if(ret == 0)
	{				
		ret = stk3x1x_i2c_read_data(ps_data->client, 0x11, 2, &value[0]);
		if(ret < 0)
		{
			printk(KERN_ERR "%s fail, ret=0x%x\n", __func__, ret);
			return ret;
		}
		word_data = (value[0]<<8) | value[1];						
		//printk(KERN_INFO "%s: word_data=%d\n", __func__, word_data);
		
		if(word_data == 0)
		{
			//printk(KERN_ERR "%s: incorrect word data (0)\n", __func__);
			return 0xFFFF;
		}
		
		if(word_data > ps_data->psa)
		{
			ps_data->psa = word_data;
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 begin . 20140115
#ifdef STK_DEBUG_PRINTF	
			printk(KERN_INFO "%s: update psa: psa=%d,psi=%d\n", __func__, ps_data->psa, ps_data->psi);
#endif
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 end . 
		}
		if(word_data < ps_data->psi)
		{
			ps_data->psi = word_data;
			ps_data->min_psi = min(ps_data->min_psi, ps_data->psi);	
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 begin . 20140115
#ifdef STK_DEBUG_PRINTF	
			printk(KERN_INFO "%s: update psi: psa=%d,psi=%d\n", __func__, ps_data->psa, ps_data->psi);	
#endif
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 end . 
		}	
	}	
	else
	{
		//sunshine or other problem  set default thd
#ifdef TUNE_LARGE_DATA
		//ps_data->ps_thd_h = 1600;
		//ps_data->ps_thd_l = 1400;
		
		if (ps_data->is_have_good_cali)
		{
			ps_data->ps_thd_h = ps_data->min_psi + 450;
			ps_data->ps_thd_l = ps_data->min_psi + 400;
		}
		else 
		{
			ret = stk3x1x_i2c_read_data(ps_data->client, 0x11, 2, &value[0]);
			if(ret < 0)
			{
				printk(KERN_ERR "%s fail, ret=0x%x\n", __func__, ret);
				return ret;
			}
			word_data = (value[0]<<8) | value[1];

			ps_data->ps_thd_h = word_data + 450;
			ps_data->ps_thd_l = word_data + 400;
		}
		
#ifdef STK_DEBUG_PRINTF
		printk(KERN_INFO "%s: use default thd %d %d\n", __func__, ps_data->ps_thd_h, ps_data->ps_thd_l);	
#endif

#else
		ps_data->ps_thd_h = ps_data->ps_high_thd_def;
		ps_data->ps_thd_l = ps_data->ps_low_thd_def;
#endif
		stk3x1x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
		stk3x1x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);

		return 0;
	}	
	
	diff = ps_data->psa - ps_data->psi;
/*		
#ifdef CALI_EVERY_TIME*/
		/*if( ps_data->ps_thd_h > ps_data->ps_high_thd_boot )
		{
			ps_data->ps_high_thd_boot = ps_data->ps_thd_h;
			ps_data->ps_low_thd_boot = ps_data->ps_thd_l;		
			printk(KERN_INFO "%s: update boot HT=%d, LT=%d\n", __func__, ps_data->ps_high_thd_boot, ps_data->ps_low_thd_boot);
		}*/
        //add by junfeng.zhou.sz@tcl.com for default thd.
/*        if( ps_data->psa > STK_MAX_CT_VALUE && ps_data->psi > STK_MAX_CT_VALUE)
        {
            if(diff > 800)
            {
                printk(KERN_INFO "%s : use the default thd because ps_data->psi %d > 1200 ,diff = %d",__func__,ps_data->psi,diff);
                ps_data->ps_thd_h = ps_data->ps_high_thd_def;
                ps_data->ps_thd_l = ps_data->ps_low_thd_def;
            }
            else
            {
                return 0;
            }
            
        }
#endif*/

#ifdef TUNE_LARGE_DATA
//add by junfeng.zhou.sz@tcl.com for default thd.
//divide 3 area  0-1000   1000-1500  1500-4400
//due to ps_data->psi equal to rawdata so will use the psi to judge
	if((ps_data->psi <= 100)&&(diff > ps_data->pdata->low_ps_ct_h_offset - 60+5))  //50-100   HT:ps+40 LT:ps+28
	{
		ps_data->ps_thd_h = ps_data->psi + ps_data->pdata->low_ps_ct_h_offset - 70;
		ps_data->ps_thd_l = ps_data->psi + ps_data->pdata->low_ps_ct_l_offset - 32;
		is_cali = 1;
	}
	else if((( ps_data->psi <= 150) &&( ps_data->psi > 100))&&(diff > ps_data->pdata->low_ps_ct_h_offset - 53+1))   //100-150   HT:ps+60 LT:ps+40
	{
	
		ps_data->ps_thd_h = ps_data->psi + ps_data->pdata->low_ps_ct_h_offset - 50;
		ps_data->ps_thd_l = ps_data->psi + ps_data->pdata->low_ps_ct_l_offset - 20;
		is_cali = 1;
	}
	else if((( ps_data->psi <= 300) &&( ps_data->psi > 150))&&(diff > ps_data->pdata->low_ps_ct_h_offset+1))   //300-150  HT:ps+130 LT:ps+85
	{
	
		ps_data->ps_thd_h = ps_data->psi + ps_data->pdata->low_ps_ct_h_offset+20;
		ps_data->ps_thd_l = ps_data->psi + ps_data->pdata->low_ps_ct_l_offset+25;
		is_cali = 1;
	}
	else if((( ps_data->psi <= ps_data->pdata->mmi_thd_h_limit+50) &&( ps_data->psi > 300))&&(diff > ps_data->pdata->hi_ps_ct_h_offset+1))   //300-500   HT:ps+240 LT:ps+120
	{
	
		ps_data->ps_thd_h = ps_data->psi + ps_data->pdata->hi_ps_ct_h_offset+40;
		ps_data->ps_thd_l = ps_data->psi + ps_data->pdata->hi_ps_ct_l_offset+50;
		is_cali = 1;
	}
	
	else if ((ps_data->psi > ps_data->pdata->mmi_thd_h_limit+50) || (ps_data->psi-ps_data->min_psi>450))
	{
		if ((ps_data->is_have_good_cali)||(ps_data->psi-ps_data->min_psi>450))
		{
			ps_data->ps_thd_h = ps_data->min_psi + 450;
			ps_data->ps_thd_l = ps_data->min_psi + 400;
#ifdef STK_DEBUG_PRINTF			
			printk("%s:ps_data->ps_thd_l = ps_data->min_psi + 800,,HT=%d,LT=%d\n", __func__, ps_data->ps_thd_h, ps_data->ps_thd_l);
#endif			
		}
		else
		{
			if (ps_data->psi < 1500)	// 800 ~ 1500
			{
				ps_data->ps_thd_h = ps_data->psi + 110;
				ps_data->ps_thd_l = ps_data->psi + 70;
			}
			
			if (ps_data->psi > 1500)	// above 1500
			{
				ps_data->ps_thd_h = ps_data->psi + 110;
				ps_data->ps_thd_l = ps_data->psi + 75;
			}
		}


		stk3x1x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
		stk3x1x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);

	}

#endif
		
	if(is_cali)
	{
		ps_data->is_have_good_cali = true;
		ps_data->last_good_psi = ps_data->psi;

		stk3x1x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
		stk3x1x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
		printk("%s: HT=%d, LT=%d\n", __func__, ps_data->ps_thd_h, ps_data->ps_thd_l);
#ifdef STK_DEBUG_PRINTF				
		printk(KERN_INFO "%s: FAE tune0 psa-psi(%d) > STK_DIFF found\n", __func__, diff);
#endif			
		if (debouse_cnt == 15) {		
			hrtimer_cancel(&ps_data->ps_tune0_timer);			
			debouse_cnt = 0;
		}
		else {
			debouse_cnt++;
		}
	}
	
	return 0;
}

static void stk_ps_tune0_work_func(struct work_struct *work)
{
	struct stk3x1x_data *ps_data = container_of(work, struct stk3x1x_data, stk_ps_tune0_work);		
	if(ps_data->tune_zero_init_proc)
		stk_tune_zero_get_ps_data(ps_data);
	else
		stk_ps_tune_zero_func_fae(ps_data);
	return;
}	


static enum hrtimer_restart stk_ps_tune0_timer_func(struct hrtimer *timer)
{
	struct stk3x1x_data *ps_data = container_of(timer, struct stk3x1x_data, ps_tune0_timer);
	queue_work(ps_data->stk_ps_tune0_wq, &ps_data->stk_ps_tune0_work);	
	hrtimer_forward_now(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay);
	return HRTIMER_RESTART;	
}
#endif

#ifdef STK_POLL_ALS
static enum hrtimer_restart stk_als_timer_func(struct hrtimer *timer)
{
	struct stk3x1x_data *ps_data = container_of(timer, struct stk3x1x_data, als_timer);
	queue_work(ps_data->stk_als_wq, &ps_data->stk_als_work);	
	hrtimer_forward_now(&ps_data->als_timer, ps_data->als_poll_delay);
	return HRTIMER_RESTART;	
	
}

static void stk_als_poll_work_func(struct work_struct *work)
{
	struct stk3x1x_data *ps_data = container_of(work, struct stk3x1x_data, stk_als_work);	
	int32_t reading, reading_lux,/* als_comperator,*/ flag_reg;
#ifdef ALS_REPORT_LUX_TABLE
	uint8_t index=0;
#endif
//modify(add) by junfeng.zhou.sz for fix i2c timeout error with 661727 begin . 20140501
    	if(ps_data->in_suspend)
    	{
        	printk(KERN_INFO "%s devices in suspend \n",__func__);
        	return ;
    	}
//modify(add) by junfeng.zhou.sz for fix i2c timeout error with 661727 end . 
	flag_reg = stk3x1x_get_flag(ps_data);
	if(flag_reg < 0)
		return;		
	
	if(!(flag_reg&STK_FLG_ALSDR_MASK))
		return;

	mutex_lock(&ps_data->io_als_lock);
	reading = stk3x1x_get_als_reading(ps_data);
	if(reading < 0)		
	{
        	mutex_unlock(&ps_data->io_als_lock);
		return;
	}
    	else if(reading <= STK_ALS_DARKCODE_THD)
    	{   
		if(dark_code_flag > 20)
        	{
            		ps_data->als_lux_last = 0;
        	}
        	else if(dark_code_flag %2 == 0)
        	{
            		ps_data->als_lux_last = 1;
            		dark_code_flag++;
        	}
        	else
        	{
            		ps_data->als_lux_last = 0;
            		dark_code_flag++;
        	}
       		
	//input_report_abs(ps_data->als_input_dev, ABS_MISC, ps_data->als_lux_last);
	//input_sync(ps_data->als_input_dev);
	stk3x1x_report_value(ps_data->als_input_dev, ABS_MISC, ps_data->als_lux_last);  
        	mutex_unlock(&ps_data->io_als_lock);
        	return;
    	}
    	else if((dark_code_flag!=0) && (reading < (STK_ALS_DARKCODE_THD+STK_ALS_CHANGE_THD)))
    	{
		
        //input_report_abs(ps_data->als_input_dev, ABS_MISC, ps_data->als_lux_last);
	//input_sync(ps_data->als_input_dev);
	stk3x1x_report_value(ps_data->als_input_dev, ABS_MISC, ps_data->als_lux_last); 
        	mutex_unlock(&ps_data->io_als_lock);
        	return;
    	}
//    else if((dark_code_flag!=0) && (reading >= (STK_ALS_DARKCODE_THD+STK_ALS_CHANGE_THD)))
    	else
    	{
        	dark_code_flag = 0;
    	}

    	mutex_unlock(&ps_data->io_als_lock);
    /*
	if(ps_data->ir_code)
	{
		ps_data->als_correct_factor = 1000;
		if(reading < STK_IRC_MAX_ALS_CODE && reading > STK_IRC_MIN_ALS_CODE && 
			ps_data->ir_code > STK_IRC_MIN_IR_CODE)
		{
			als_comperator = reading * STK_IRC_ALS_NUMERA / STK_IRC_ALS_DENOMI;
			if(ps_data->ir_code > als_comperator)
				ps_data->als_correct_factor = STK_IRC_ALS_CORREC;
		}
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 begin . 20140115
#ifdef STK_DEBUG_PRINTF	
		printk(KERN_INFO "%s: als=%d, ir=%d, als_correct_factor=%d", __func__, reading, ps_data->ir_code, ps_data->als_correct_factor);
#endif
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 end .
		ps_data->ir_code = 0;
	}	
	*/
	reading = reading * ps_data->als_correct_factor / 1000;
	
	reading_lux = stk_alscode2lux(ps_data, reading);
	if(abs(ps_data->als_lux_last - reading_lux) >= STK_ALS_CHANGE_THD)
	{
		ps_data->als_lux_last = reading_lux;
	//	ps_data->als_lux_last = ps_data->als_lux_last * 10 / 44; 		
#ifdef ALS_REPORT_LUX_TABLE
        for(index = 0;index<10;index++)
        {
            if(ps_data->als_lux_last > lux_adc_table[index] && \
                ps_data->als_lux_last < lux_adc_table[index+1])
                {
                    ps_data->als_lux_last = lux_report_table[index];
                    break;
                }
        }
        if(ps_data->als_lux_last > 3500)
        {
            ps_data->als_lux_last = 2000;
        }
#endif
	
	//input_report_abs(ps_data->als_input_dev, ABS_MISC, ps_data->als_lux_last);
	//input_sync(ps_data->als_input_dev);
	stk3x1x_report_value(ps_data->als_input_dev, ABS_MISC, ps_data->als_lux_last); 

//modify(add) by junfeng.zhou.sz for delete debug info with 533451 begin . 20140115
#ifdef STK_DEBUG_PRINTF	
	printk(KERN_INFO "%s: als input event %d lux\n",__func__, reading_lux);	
#endif	
//modify(add) by junfeng.zhou.sz for delete debug info with 533451 end .
	}
	return;
}
#endif /* #ifdef STK_POLL_ALS */


#ifdef STK_POLL_PS	
static enum hrtimer_restart stk_ps_timer_func(struct hrtimer *timer)
{
	struct stk3x1x_data *ps_data = container_of(timer, struct stk3x1x_data, ps_timer);
	queue_work(ps_data->stk_ps_wq, &ps_data->stk_ps_work);
	hrtimer_forward_now(&ps_data->ps_timer, ps_data->ps_poll_delay);
	return HRTIMER_RESTART;		
}

static void stk_ps_poll_work_func(struct work_struct *work)
{
	struct stk3x1x_data *ps_data = container_of(work, struct stk3x1x_data, stk_ps_work);	
	uint32_t reading;
	int32_t near_far_state;
    	uint8_t org_flag_reg;	
	int32_t ret;
    	uint8_t disable_flag = 0;
#ifdef STK_GES			
	uint8_t disable_flag2 = 0, org_flag2_reg;
	if(ps_data->ges_enabled == 2)
	{
		ret = stk3x1x_get_flag2(ps_data);
		if(ret < 0)
			goto err_i2c_rw;	
		org_flag2_reg = ret;
		
		disable_flag2 = org_flag2_reg & (STK_FLG2_INT_GS_MASK | 
			STK_FLG2_GS10_MASK | STK_FLG2_GS01_MASK);
		if(org_flag2_reg & STK_FLG2_GS10_MASK)
		{
			printk(KERN_INFO "%s: >>>>>>>>>>>>\n", __func__);
		}
		if(org_flag2_reg & STK_FLG2_GS01_MASK)
		{
			printk(KERN_INFO "%s: <<<<<<<<<<<<\n", __func__);
		}
		atomic_set(&ps_data->gesture2, (disable_flag2 & 
				(STK_FLG2_GS10_MASK | STK_FLG2_GS01_MASK)));  		

		if(disable_flag2)
		{
			ret = stk3x1x_set_flag2(ps_data, org_flag2_reg, disable_flag2);		
			if(ret < 0)
				goto err_i2c_rw;
		}			
	}	
#endif
	
	if(ps_data->ps_enabled)
	{
#ifndef CALI_EVERY_TIME
  #ifdef STK_TUNE0
		if(!(ps_data->psi_set))
			return;	
  #endif	
#endif
		org_flag_reg = stk3x1x_get_flag(ps_data);
		if(org_flag_reg < 0)
			goto err_i2c_rw;		

		if(!(org_flag_reg&STK_FLG_PSDR_MASK))
		{
			return;
		}	
				
		near_far_state = (org_flag_reg & STK_FLG_NF_MASK)?1:0;	
		reading = stk3x1x_get_ps_reading(ps_data);
		if(ps_data->ps_distance_last != near_far_state)
		{
			ps_data->ps_distance_last = near_far_state;
			//input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, near_far_state);
			//input_sync(ps_data->ps_input_dev);
			stk3x1x_report_value(ps_data->ps_input_dev, ABS_DISTANCE, near_far_state); 
			wake_lock_timeout(&ps_data->ps_wakelock, HZ/2);		
#ifdef STK_DEBUG_PRINTF		
			printk(KERN_INFO "%s: ps input event %d cm, ps code = %d\n",__func__, near_far_state, reading);		
#endif		
		}
		ret = stk3x1x_set_flag(ps_data, org_flag_reg, disable_flag);		
		if(ret < 0)
			goto err_i2c_rw;
		return;
	}
	
	
err_i2c_rw:
	msleep(30);	
	return;
}
#endif

#if (!defined(STK_POLL_PS) || !defined(STK_POLL_ALS))
static void stk_work_func(struct work_struct *work)
{
	uint32_t reading;
#if ((STK_INT_PS_MODE != 0x03) && (STK_INT_PS_MODE != 0x02))
    	int32_t ret;
    	uint8_t disable_flag = 0;
    	uint8_t org_flag_reg;
#endif	/* #if ((STK_INT_PS_MODE != 0x03) && (STK_INT_PS_MODE != 0x02)) */

#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD	
	uint32_t nLuxIndex;	
#endif
	struct stk3x1x_data *ps_data = container_of(work, struct stk3x1x_data, stk_work);	
	int32_t near_far_state;
	int32_t als_comperator;
#ifdef STK_GES				
	uint8_t disable_flag2 = 0, org_flag2_reg;
#endif	
#ifdef ALS_REPORT_LUX_TABLE
	uint8_t index=0;
#endif
#if (STK_INT_PS_MODE	== 0x03)
	near_far_state = gpio_get_value(ps_data->int_pin);
#elif	(STK_INT_PS_MODE	== 0x02)
	near_far_state = !(gpio_get_value(ps_data->int_pin));
#endif	

#if ((STK_INT_PS_MODE == 0x03) || (STK_INT_PS_MODE	== 0x02))
	ps_data->ps_distance_last = near_far_state;
	//input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, near_far_state);
	//input_sync(ps_data->ps_input_dev);
	stk3x1x_report_value(ps_data->ps_input_dev, ABS_DISTANCE, near_far_state);
	wake_lock_timeout(&ps_data->ps_wakelock, HZ/2);
	reading = stk3x1x_get_ps_reading(ps_data);
#ifdef STK_DEBUG_PRINTF	
	printk(KERN_INFO "%s: ps input event %d cm, ps code = %d\n",__func__, near_far_state, reading);			
#endif	
#else
	/* mode 0x01 or 0x04 */	
	org_flag_reg = stk3x1x_get_flag(ps_data);
	if(org_flag_reg < 0)
		goto err_i2c_rw;	
	
    	if (org_flag_reg & STK_FLG_ALSINT_MASK)
    	{
		disable_flag |= STK_FLG_ALSINT_MASK;
        	reading = stk3x1x_get_als_reading(ps_data);
		if(reading < 0)		
		{
			printk(KERN_ERR "%s: stk3x1x_get_als_reading fail, ret=%d\n", __func__, reading);
			goto err_i2c_rw;
		}			
#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
        	nLuxIndex = stk_get_lux_interval_index(reading);
        	stk3x1x_set_als_thd_h(ps_data, code_threshold_table[nLuxIndex]);
        	stk3x1x_set_als_thd_l(ps_data, code_threshold_table[nLuxIndex-1]);
#else
        	stk_als_set_new_thd(ps_data, reading);
#endif //CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD

		if(ps_data->ir_code)
		{
			if(reading < STK_IRC_MAX_ALS_CODE && reading > STK_IRC_MIN_ALS_CODE && 
			ps_data->ir_code > STK_IRC_MIN_IR_CODE)
			{
				als_comperator = reading * STK_IRC_ALS_NUMERA / STK_IRC_ALS_DENOMI;
				if(ps_data->ir_code > als_comperator)
					ps_data->als_correct_factor = STK_IRC_ALS_CORREC;
				else
					ps_data->als_correct_factor = 1000;
			}
			printk(KERN_INFO "%s: als=%d, ir=%d, als_correct_factor=%d\n", __func__, reading, ps_data->ir_code, ps_data->als_correct_factor);
			ps_data->ir_code = 0;
		}	

		reading = reading * ps_data->als_correct_factor / 1000;

		ps_data->als_lux_last = stk_alscode2lux(ps_data, reading);
#ifdef ALS_REPORT_LUX_TABLE
        	for(index = 0;index<10;index++)
        	{
		    	if(ps_data->als_lux_last > lux_adc_table[index] && \
		        	ps_data->als_lux_last < lux_adc_table[index+1])
		        {
		            ps_data->als_lux_last = lux_report_table[index];
		            break;
		        }
        	}
        	if(ps_data->als_lux_last > 3500)
        	{
            		ps_data->als_lux_last = 2000;
        	}
#endif
		//input_report_abs(ps_data->als_input_dev, ABS_MISC, ps_data->als_lux_last);
		//input_sync(ps_data->als_input_dev);
		stk3x1x_report_value(ps_data->als_input_dev, ABS_MISC, ps_data->als_lux_last);

#ifdef STK_DEBUG_PRINTF		
		printk(KERN_INFO "%s: als input event %d lux\n",__func__, ps_data->als_lux_last);			
#endif		
    	}
    	if (org_flag_reg & STK_FLG_PSINT_MASK)
    	{
		disable_flag |= STK_FLG_PSINT_MASK;
		near_far_state = (org_flag_reg & STK_FLG_NF_MASK)?1:0;
		
		ps_data->ps_distance_last = near_far_state;
		printk(KERN_INFO "%s: ps-distant = %s \n",__func__,ps_data->ps_distance_last? "far" : "near");
#ifdef PRINT_TIME
		print_local_time("stk_report_data_time"); 
#endif
		//input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, near_far_state);
		//input_sync(ps_data->ps_input_dev);
		stk3x1x_report_value(ps_data->ps_input_dev, ABS_DISTANCE, near_far_state);
		//wake_lock_timeout(&ps_data->ps_wakelock, HZ/2);			
		reading = stk3x1x_get_ps_reading(ps_data);
#ifdef STK_DEBUG_PRINTF		
		printk(KERN_INFO "%s: ps input event=%d, ps code = %d\n",__func__, near_far_state, reading);
#endif			
	}
	
	if(disable_flag)	
	{	
		ret = stk3x1x_set_flag(ps_data, org_flag_reg, disable_flag);		
		if(ret < 0)
			goto err_i2c_rw;
	}
#endif	
	usleep_range(1000, 2000);
	//msleep(1);
    	enable_irq(ps_data->irq);
	return;

err_i2c_rw:
	msleep(30);
	enable_irq(ps_data->irq);
	return;	
}

static irqreturn_t stk_oss_irq_handler(int irq, void *data)
{
	struct stk3x1x_data *pData = data;
	wake_lock_timeout(&pData->ps_wakelock, HZ/2);	
	disable_irq_nosync(irq);
	queue_work(pData->stk_wq,&pData->stk_work);
	return IRQ_HANDLED;
}
#endif	/*	#if (!defined(STK_POLL_PS) || !defined(STK_POLL_ALS))	*/
static int32_t stk3x1x_init_all_setting(struct i2c_client *client, struct stk3x1x_platform_data *plat_data)
{
	int32_t ret;
	struct stk3x1x_data *ps_data = i2c_get_clientdata(client);		
	
	stk3x1x_proc_plat_data(ps_data, plat_data);
    
    	ret = stk3x1x_check_pid(ps_data);
	if(ret < 0)
		return ret;
    
	ret = stk3x1x_software_reset(ps_data); 
	if(ret < 0)
		return ret;
	

	ret = stk3x1x_init_all_reg(ps_data);
	if(ret < 0)
		return ret;	
		
	ps_data->als_enabled = false;
	ps_data->ps_enabled = false;		
	ps_data->re_enable_als = false;
	ps_data->re_enable_ps = false;
	ps_data->ir_code = 0;
	ps_data->als_correct_factor = 1000;
	ps_data->first_boot = true;	
#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
	stk_init_code_threshold_table(ps_data);
#endif			
#ifdef STK_TUNE0
	stk_ps_tune_zero_init(ps_data);
#endif	
#ifdef STK_ALS_FIR
	memset(&ps_data->fir, 0x00, sizeof(ps_data->fir));  
	atomic_set(&ps_data->firlength, STK_FIR_LEN);	
#endif
	atomic_set(&ps_data->recv_reg, 0);  
#ifdef STK_GES	
	ps_data->re_enable_ges = 0;	
	atomic_set(&ps_data->gesture2, 0);  
	//memset(stk_ges_op, 0, sizeof(stk_ges_op));
#endif	
    return 0;
}
// [Feature]Add-BEGIN by TCTSZ. ning.wei@tcl.com, 2015/08/03, for add stk p/l sensor pull up&down ALM389755
static int stk3x1x_pinctrl_init(struct stk3x1x_data *ps_data)
{
	struct i2c_client *client = ps_data->client;

	ps_data->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(ps_data->pinctrl)) {
		dev_err(&client->dev, "Failed to get pinctrl\n");
		return PTR_ERR(ps_data->pinctrl);
	}

	ps_data->pin_default =
		pinctrl_lookup_state(ps_data->pinctrl, "default");
	if (IS_ERR_OR_NULL(ps_data->pin_default)) {
		dev_err(&client->dev, "Failed to look up default state\n");
		return PTR_ERR(ps_data->pin_default);
	}

	ps_data->pin_sleep =
		pinctrl_lookup_state(ps_data->pinctrl, "sleep");
	if (IS_ERR_OR_NULL(ps_data->pin_sleep)) {
		dev_err(&client->dev, "Failed to look up sleep state\n");
		return PTR_ERR(ps_data->pin_sleep);
	}

	return 0;
}
// [Feature]-Add-END by TCTSZ. ning.wei@tcl.com, 2015/08/03, for add stk p/l sensor pull up&down ALM389755

#if (!defined(STK_POLL_PS) || !defined(STK_POLL_ALS))
static int stk3x1x_setup_irq(struct i2c_client *client)
{		
	int irq, err = -EIO;
	struct stk3x1x_data *ps_data = i2c_get_clientdata(client);

#ifdef SPREADTRUM_PLATFORM	
	irq = sprd_alloc_gpio_irq(ps_data->int_pin);	
#else	
	irq = gpio_to_irq(ps_data->int_pin);
#endif	
#ifdef STK_DEBUG_PRINTF	
	printk(KERN_INFO "%s: int pin #=%d, irq=%d\n",__func__, ps_data->int_pin, irq);	
#endif	
	if (irq <= 0)
	{
		printk(KERN_ERR "irq number is not specified, irq # = %d, int pin=%d\n",irq, ps_data->int_pin);
		return irq;
	}
	ps_data->irq = irq;	
	err = gpio_request(ps_data->int_pin,"stk-int");        
	if(err < 0)
	{
		printk(KERN_ERR "%s: gpio_request, err=%d\n", __func__, err);
		return err;
	}
	err = gpio_direction_input(ps_data->int_pin);
	if(err < 0)
	{
		printk(KERN_ERR "%s: gpio_direction_input, err=%d\n", __func__, err);
		return err;
	}		
	
// [Feature]Add-BEGIN by TCTSZ. ning.wei@tcl.com, 2015/08/03, for add stk p/l sensor pull up&down ALM389755
	err = stk3x1x_pinctrl_init(ps_data);
	if (err) {
		printk(KERN_ERR "%s: Can't initialize pinctrl, err=%d \n", __func__, err);
		return err;
			
	}
	err = pinctrl_select_state(ps_data->pinctrl, ps_data->pin_default);
	if (err) {
		printk(KERN_ERR "%s: Can't select pinctrl default state, err=%d \n", __func__, err);
		return err;
	}
// [Feature]-Add-END by TCTSZ. ning.wei@tcl.com, 2015/08/03, for add stk p/l sensor pull up&down ALM389755
	
#if ((STK_INT_PS_MODE == 0x03) || (STK_INT_PS_MODE	== 0x02))	
	err = request_any_context_irq(irq, stk_oss_irq_handler, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, DEVICE_NAME, ps_data);
#else	
	err = request_any_context_irq(irq, stk_oss_irq_handler, IRQF_TRIGGER_LOW, DEVICE_NAME, ps_data);
#endif	
	if (err < 0) 
	{
		printk(KERN_WARNING "%s: request_any_context_irq(%d) failed for (%d)\n", __func__, irq, err);		
		goto err_request_any_context_irq;
	}
	disable_irq(irq);
	
	return 0;
err_request_any_context_irq:	
#ifdef SPREADTRUM_PLATFORM
	sprd_free_gpio_irq(ps_data->int_pin);	
#else	
	gpio_free(ps_data->int_pin);		
#endif	
	return err;
}
#endif


static int stk3x1x_suspend(struct device *dev)
{
	struct stk3x1x_data *ps_data = dev_get_drvdata(dev);
	//modify(add) by junfeng.zhou.sz for fix i2c timeout error with 661727 begin . 20140501
    //uint8_t curr_als_enable = (ps_data->als_enabled)?1:0;
    //uint8_t curr_ps_enable = (ps_data->ps_enabled)?1:0;
#if 1
#if (!defined(STK_POLL_ALS) || defined(STK_CHK_REG))
	int err;
#endif
#ifndef STK_POLL_PS	
    	struct i2c_client *client = to_i2c_client(dev);	
#endif

	
#ifndef SPREADTRUM_PLATFORM	
	mutex_lock(&ps_data->io_lock);  	
#endif	
#ifdef STK_CHK_REG
	if (ps_data->ps_enabled || ps_data->als_enabled) {
		err = stk3x1x_validate_n_handle(ps_data->client);
		if(err < 0)	
		{
			printk(KERN_ERR "stk3x1x_validate_n_handle fail: %d\n", err); 
		}
		else if (err == 0xFF)
		{
			if(ps_data->ps_enabled)
				stk3x1x_enable_ps(ps_data, 1, 0);
		}
	}
#endif /* #ifdef STK_CHK_REG */
#ifdef STK_GES	
	if(ps_data->ges_enabled == 1)
	{
		ps_data->re_enable_ges = ps_data->ges_enabled;		
		stk3x1x_enable_ges(ps_data, 0, 1);		
	}
	else if(ps_data->ges_enabled == 2)
	{
		ps_data->re_enable_ges = ps_data->ges_enabled;	
		stk3x1x_enable_ges(ps_data, 0, 2);
	}
#endif	
#ifndef SPREADTRUM_PLATFORM	
	if(ps_data->als_enabled)
	{	
		printk(KERN_INFO "%s: Enable ALS : 0\n", __func__);
		stk3x1x_enable_als(ps_data, 0);		
		ps_data->re_enable_als = true;
	}  	
#endif	
	if(ps_data->ps_enabled)
	{
#ifdef STK_POLL_PS		
		wake_lock(&ps_data->ps_nosuspend_wl);
#else		
		if(device_may_wakeup(&client->dev))
		{
			err = enable_irq_wake(ps_data->irq);	
			if (err)
				printk(KERN_WARNING "%s: set_irq_wake(%d) failed, err=(%d)\n", __func__, ps_data->irq, err);				
		}
		else
		{
			printk(KERN_ERR "%s: not support wakeup source\n", __func__);
		}		
#endif	
	}
    
#ifndef SPREADTRUM_PLATFORM		
	mutex_unlock(&ps_data->io_lock);		
#endif	
#endif
    	printk(KERN_INFO "%s\n", __func__);    
	//ps_data->als_enable_pre = curr_als_enable;
	//stk3x1x_enable_als(ps_data, 0);
	//ps_data->ps_enable_pre = curr_ps_enable;
	//stk3x1x_enable_ps(ps_data, 0,1);
	ps_data->in_suspend = 1;
	//modify(add) by junfeng.zhou.sz for fix i2c timeout error with 661727 end . 
	return 0;	
}

static int stk3x1x_resume(struct device *dev)
{
	struct stk3x1x_data *ps_data = dev_get_drvdata(dev);	
	//modify(add) by junfeng.zhou.sz for fix i2c timeout error with 661727 begin . 20140501
#if 1
#if (!defined(STK_POLL_ALS) || defined(STK_CHK_REG))
	int err;
#endif
	
#ifndef STK_POLL_PS	    
    	struct i2c_client *client = to_i2c_client(dev);	
#endif
	
	printk(KERN_INFO "%s\n", __func__);	



#ifndef SPREADTRUM_PLATFORM		
	mutex_lock(&ps_data->io_lock); 		
#endif	
#ifdef STK_CHK_REG

	if (ps_data->ps_enabled || ps_data->als_enabled) {
		err = stk3x1x_validate_n_handle(ps_data->client);
		if(err < 0)	
		{
			printk(KERN_ERR "stk3x1x_validate_n_handle fail: %d\n", err); 
		}
		else if (err == 0xFF)
		{
			if(ps_data->ps_enabled)
			stk3x1x_enable_ps(ps_data, 1, 0);
		}
	}	
#endif /* #ifdef STK_CHK_REG */
#ifdef STK_GES	
	if(ps_data->re_enable_ges == 1)
	{
		stk3x1x_enable_ges(ps_data, 1, 1);
		ps_data->re_enable_ges = 0;		
	}
	else if(ps_data->re_enable_ges == 2)
	{
		stk3x1x_enable_ges(ps_data, 1, 2);		
		ps_data->re_enable_ges = 0;				
	}
#endif	
#ifndef SPREADTRUM_PLATFORM		
	if(ps_data->re_enable_als)
	{
		printk(KERN_INFO "%s: Enable ALS : 1\n", __func__);		
		stk3x1x_enable_als(ps_data, 1);		
		ps_data->re_enable_als = false;		
	}
#endif
	if(ps_data->ps_enabled)
	{
#ifdef STK_POLL_PS		
		wake_unlock(&ps_data->ps_nosuspend_wl);		
#else		
		if(device_may_wakeup(&client->dev))
		{	
			err = disable_irq_wake(ps_data->irq);	
			if (err)		
				printk(KERN_WARNING "%s: disable_irq_wake(%d) failed, err=(%d)\n", __func__, ps_data->irq, err);		
		}		
#endif	
	}
     
#ifndef SPREADTRUM_PLATFORM			
	mutex_unlock(&ps_data->io_lock);
#endif	
#endif
    	printk(KERN_INFO "%s\n", __func__);
    	ps_data->in_suspend = 0;
    	//stk3x1x_enable_als(ps_data, ps_data->als_enable_pre);
    	//stk3x1x_enable_ps(ps_data, ps_data->ps_enable_pre,1);
    	//modify(add) by junfeng.zhou.sz for fix i2c timeout error with 661727 end . 
	return 0;	
}

static const struct dev_pm_ops stk3x1x_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(stk3x1x_suspend, stk3x1x_resume)
};

//#ifdef CONFIG_HAS_EARLYSUSPEND
#if 0
static void stk3x1x_early_suspend(struct early_suspend *h)
{
	
}

static void stk3x1x_late_resume(struct early_suspend *h)
{

}
#endif	//#ifdef CONFIG_HAS_EARLYSUSPEND
//modify(add) by junfeng.zhou.sz for add power supply begin . 20140214
#ifdef POWER_REGULATOR
static int stk3x1x_power_ctl(struct stk3x1x_data *data, bool on)
{
	int ret = 0;

	if (!on && data->power_enabled) {
		ret = regulator_disable(data->vdd);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(data->vio);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vio disable failed ret=%d\n", ret);
			//retregulator_enable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
		dev_dbg(&data->client->dev, "stk3x1x_power_ctl on=%d\n",
				on);
	} else if (on && !data->power_enabled) {

		ret = regulator_enable(data->vdd);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_enable(data->vio);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vio enable failed ret=%d\n", ret);
			regulator_disable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
		dev_dbg(&data->client->dev, "stk3x1x_power_ctl on=%d\n",
				on);
	} else {
		dev_warn(&data->client->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
	}

	return ret;
}

static int stk3x1x_power_init(struct stk3x1x_data *data, bool on)
{
	int ret;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd,
					0, STK3X1X_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio,
					0, STK3X1X_VIO_MAX_UV);

		regulator_put(data->vio);
	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			ret = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,
				"Regulator get failed vdd ret=%d\n", ret);
			return ret;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			ret = regulator_set_voltage(data->vdd,
					STK3X1X_VDD_MIN_UV,
					STK3X1X_VDD_MAX_UV);
			if (ret) {
				dev_err(&data->client->dev,
					"Regulator set failed vdd ret=%d\n",
					ret);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			ret = PTR_ERR(data->vio);
			dev_err(&data->client->dev,
				"Regulator get failed vio ret=%d\n", ret);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			ret = regulator_set_voltage(data->vio,
					STK3X1X_VIO_MIN_UV,
					STK3X1X_VIO_MAX_UV);
			if (ret) {
				dev_err(&data->client->dev,
				"Regulator set failed vio ret=%d\n", ret);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, STK3X1X_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return ret;
}

static int stk3x1x_device_ctl(struct stk3x1x_data *ps_data, bool enable)
{
	int ret;
	struct device *dev = &ps_data->client->dev;

	if (enable && !ps_data->power_enabled) {
		ret = stk3x1x_power_ctl(ps_data, true);
		if (ret) {
			dev_err(dev, "Failed to enable device power\n");
			goto err_exit;
		}
/*		ret = stk3x1x_init_all_setting(ps_data->client, ps_data->pdata);
		if (ret < 0) {
			stk3x1x_power_ctl(ps_data, false);
			dev_err(dev, "Failed to re-init device setting\n");
			goto err_exit;
		}*/
	} else if (!enable && ps_data->power_enabled) {
		if (!ps_data->als_enabled && !ps_data->ps_enabled) {
			ret = stk3x1x_power_ctl(ps_data, false);
			if (ret) {
				dev_err(dev, "Failed to disable device power\n");
				goto err_exit;
			}
		} else {
			dev_dbg(dev, "device control: als_enabled=%d, ps_enabled=%d\n",
				ps_data->als_enabled, ps_data->ps_enabled);
		}
	} else {
		dev_dbg(dev, "device control: enable=%d, power_enabled=%d\n",
			enable, ps_data->power_enabled);
	}
	return 0;

err_exit:
	return ret;
}
#endif
//modify(add) by junfeng.zhou.sz for add power supply end . 

static int stk3x1x_set_wq(struct stk3x1x_data *ps_data)
{
#ifdef STK_POLL_ALS	
	ps_data->stk_als_wq = create_singlethread_workqueue("stk_als_wq");
	INIT_WORK(&ps_data->stk_als_work, stk_als_poll_work_func);
	hrtimer_init(&ps_data->als_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ps_data->als_poll_delay = ns_to_ktime(110 * NSEC_PER_MSEC);
	ps_data->als_timer.function = stk_als_timer_func;
#endif	

#ifdef STK_POLL_PS	
	ps_data->stk_ps_wq = create_singlethread_workqueue("stk_ps_wq");
	INIT_WORK(&ps_data->stk_ps_work, stk_ps_poll_work_func);
	hrtimer_init(&ps_data->ps_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ps_data->ps_poll_delay = ns_to_ktime(60 * NSEC_PER_MSEC);
	ps_data->ps_timer.function = stk_ps_timer_func;
#endif	

#ifdef STK_TUNE0
	ps_data->stk_ps_tune0_wq = create_singlethread_workqueue("stk_ps_tune0_wq");
	INIT_WORK(&ps_data->stk_ps_tune0_work, stk_ps_tune0_work_func);
	hrtimer_init(&ps_data->ps_tune0_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ps_data->ps_tune0_delay = ns_to_ktime(60 * NSEC_PER_MSEC);
	ps_data->ps_tune0_timer.function = stk_ps_tune0_timer_func;
#endif

#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
	ps_data->stk_wq = create_singlethread_workqueue("stk_wq");
	INIT_WORK(&ps_data->stk_work, stk_work_func);
#endif
	return 0;
}


static int stk3x1x_set_input_devices(struct stk3x1x_data *ps_data)
{
	int err;
	
	ps_data->als_input_dev = input_allocate_device();
	if (ps_data->als_input_dev==NULL)
	{
		printk(KERN_ERR "%s: could not allocate als device\n", __func__);
		err = -ENOMEM;
		return err;
	}
	ps_data->ps_input_dev = input_allocate_device();
	if (ps_data->ps_input_dev==NULL)
	{
		printk(KERN_ERR "%s: could not allocate ps device\n", __func__);		
		err = -ENOMEM;
		return err;
	}
	ps_data->als_input_dev->name = ALS_NAME;
	ps_data->ps_input_dev->name = PS_NAME;
	set_bit(EV_ABS, ps_data->als_input_dev->evbit);
	set_bit(EV_ABS, ps_data->ps_input_dev->evbit);
	input_set_abs_params(ps_data->als_input_dev, ABS_MISC, 0, stk_alscode2lux(ps_data, (1<<16)-1), 0, 0);
	input_set_abs_params(ps_data->ps_input_dev, ABS_DISTANCE, 0,1, 0, 0);
	err = input_register_device(ps_data->als_input_dev);
	if (err<0)
	{
		printk(KERN_ERR "%s: can not register als input device\n", __func__);		
		return err;
	}
	err = input_register_device(ps_data->ps_input_dev);	
	if (err<0)
	{
		printk(KERN_ERR "%s: can not register ps input device\n", __func__);	
		return err;
	}
	
	err = sysfs_create_group(&ps_data->als_input_dev->dev.kobj, &stk_als_attribute_group);
	if (err < 0) 
	{
		printk(KERN_ERR "%s:could not create sysfs group for als\n", __func__);
		return err;
	}
	err = sysfs_create_group(&ps_data->ps_input_dev->dev.kobj, &stk_ps_attribute_group);
	if (err < 0) 
	{
		printk(KERN_ERR "%s:could not create sysfs group for ps\n", __func__);
		return err;
	}
	input_set_drvdata(ps_data->als_input_dev, ps_data);
	input_set_drvdata(ps_data->ps_input_dev, ps_data);		
	
#ifdef STK_GES
	ps_data->ges_input_dev = input_allocate_device();
	if (ps_data->ges_input_dev==NULL)
	{
		printk(KERN_ERR "%s: could not allocate ps device\n", __func__);		
		err = -ENOMEM;
		return err;		
	}
	ps_data->ges_input_dev->name = "stk_ges";
	ps_data->ges_input_dev->evbit[0] = BIT_MASK(EV_KEY);
	set_bit(KEY_PAGEUP, ps_data->ges_input_dev->keybit);
	set_bit(KEY_PAGEDOWN, ps_data->ges_input_dev->keybit);
	set_bit(KEY_VOLUMEUP, ps_data->ges_input_dev->keybit);
	set_bit(KEY_VOLUMEDOWN, ps_data->ges_input_dev->keybit);
	/*
	set_bit(KEY_LEFT, ps_data->ges_input_dev->keybit);
	set_bit(KEY_RIGHT, ps_data->ges_input_dev->keybit);
	set_bit(KEY_UP, ps_data->ges_input_dev->keybit);
	set_bit(KEY_DOWN, ps_data->ges_input_dev->keybit);
	*/	
	err = input_register_device(ps_data->ges_input_dev);	
	if (err<0)
	{
		printk(KERN_ERR "%s: can not register ps input device\n", __func__);	
		return err;
	}
	
	err = sysfs_create_group(&ps_data->ges_input_dev->dev.kobj, &stk_ges_attribute_group);
	if (err < 0) 
	{
		printk(KERN_ERR "%s:could not create sysfs group for ps\n", __func__);
		return err;
	}	
	input_set_drvdata(ps_data->ges_input_dev, ps_data);	
#endif	
	
	return 0;
}

#ifdef CONFIG_OF
static int stk3x1x_parse_dt(struct device *dev,
			struct stk3x1x_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	u32 temp_val;

	pdata->int_pin = of_get_named_gpio_flags(np, "stk,irq-gpio",
				0, &pdata->int_flags);
	if (pdata->int_pin < 0) {
		dev_err(dev, "Unable to read irq-gpio\n");
		return pdata->int_pin;
	}

	rc = of_property_read_u32(np, "stk,transmittance", &temp_val);
	if (!rc)
		pdata->transmittance = temp_val;
	else {
		dev_err(dev, "Unable to read transmittance\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,state-reg", &temp_val);
	if (!rc)
		pdata->state_reg = temp_val;
	else {
		dev_err(dev, "Unable to read state-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,psctrl-reg", &temp_val);
	if (!rc)
		pdata->psctrl_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read psctrl-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,alsctrl-reg", &temp_val);
	if (!rc)
		pdata->alsctrl_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read alsctrl-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,ledctrl-reg", &temp_val);
	if (!rc)
		pdata->ledctrl_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read ledctrl-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,wait-reg", &temp_val);
	if (!rc)
		pdata->wait_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read wait-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,ps-thdh", &temp_val);
	if (!rc)
		pdata->ps_thd_h = (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdh\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,ps-thdl", &temp_val);
	if (!rc)
		pdata->ps_thd_l = (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdl\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,mmi-thdh-limit", &temp_val);
	if (!rc)
		pdata->mmi_thd_h_limit = (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdl\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,mmi-thdl-limit", &temp_val);
	if (!rc)
		pdata->mmi_thd_l_limit = (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdl\n");
		return rc;
	}

	pdata->use_fir = of_property_read_bool(np, "stk,use-fir");

	return 0;
}
#else
static int stk3x1x_parse_dt(struct device *dev,
			struct stk3x1x_platform_data *pdata)
{
	return -ENODEV;
}
#endif /* !CONFIG_OF */
static int stk3x1x_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
    	int err = -ENODEV;
    	struct stk3x1x_data *ps_data;
	struct stk3x1x_platform_data *plat_data;
    	printk(KERN_INFO "%s: driver version = %s\n", __func__, DRIVER_VERSION);
	
    	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    	{
        	printk(KERN_ERR "%s: No Support for I2C_FUNC_I2C\n", __func__);
        	return -ENODEV;
    	}

	ps_data = kzalloc(sizeof(struct stk3x1x_data),GFP_KERNEL);
	if(!ps_data)
	{
		printk(KERN_ERR "%s: failed to allocate stk3x1x_data\n", __func__);
		return -ENOMEM;
	}
	ps_data->client = client;
	i2c_set_clientdata(client,ps_data);
	mutex_init(&ps_data->io_lock);
    	mutex_init(&ps_data->io_als_lock);
    	mutex_init(&stk_i2c_lock);
	wake_lock_init(&ps_data->ps_wakelock,WAKE_LOCK_SUSPEND, "stk_input_wakelock");

#ifdef STK_POLL_PS			
	wake_lock_init(&ps_data->ps_nosuspend_wl,WAKE_LOCK_SUSPEND, "stk_nosuspend_wakelock");
#endif	

	if (client->dev.of_node) {
		plat_data = devm_kzalloc(&client->dev,
			sizeof(struct stk3x1x_platform_data), GFP_KERNEL);
		if (!plat_data) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = stk3x1x_parse_dt(&client->dev, plat_data);
		dev_err(&client->dev,
			"%s: stk3x1x_parse_dt ret=%d\n", __func__, err);
		if (err)
			return err;
	} else {
		plat_data = client->dev.platform_data;
	}
	
	ps_data->als_transmittance = plat_data->transmittance;	
	ps_data->int_pin = plat_data->int_pin;
	ps_data->use_fir = plat_data->use_fir;
	ps_data->ps_low_thd_def = plat_data->ps_thd_l;
	ps_data->ps_high_thd_def = plat_data->ps_thd_h;
	//modfiy by junfeng.zhou . for not use it
	ps_data->pdata = plat_data;
	//modify(add) by junfeng.zhou.sz for fix i2c timeout error with 661727 begin . 20140501
    	ps_data->in_suspend = 0;
	ps_data->is_have_good_cali = false;
	ps_data->last_good_psi = 0xffff;
	ps_data->min_psi = 0xffff;
	plat_data->low_ps_ct_h_offset = STK_LOW_PS_HT_N_CT;
	plat_data->low_ps_ct_l_offset = STK_LOW_PS_LT_N_CT;
	plat_data->hi_ps_ct_h_offset = STK_HI_PS_HT_N_CT;
	plat_data->hi_ps_ct_l_offset = STK_HI_PS_LT_N_CT;
    	//modify(add) by junfeng.zhou.sz for fix i2c timeout error with 661727 end . 

	if (ps_data->als_transmittance == 0) {
		dev_err(&client->dev,
			"%s: Please set als_transmittance\n", __func__);
		goto err_als_input_allocate;
	}
	

	if (!plat_data) {
		dev_err(&client->dev,
			"%s: no stk3x1x platform data!\n", __func__);
		goto err_als_input_allocate;
	}
	
	stk3x1x_set_wq(ps_data);
	err = stk3x1x_init_all_setting(client, plat_data);
	if(err < 0)
		goto err_init_all_setting;			
	
	err = stk3x1x_set_input_devices(ps_data);
	if(err < 0)
		goto err_setup_input_device;	
	
	device_create_file(&(client->dev),&als_code_attribute);
	device_create_file(&(client->dev),&ps_offset_attribute);
	device_create_file(&(client->dev),&ps_prx_raw_off_attribute);
	device_create_file(&(client->dev),&ps_prx_raw_org_attribute);
	device_create_file(&(client->dev),&mmi_prx_thd_h_limit_attribute);
	device_create_file(&(client->dev),&mmi_prx_thd_l_limit_attribute);
	device_create_file(&(client->dev),&trace_param_attribute);
	
#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
	err = stk3x1x_setup_irq(client);
	if(err < 0)
		goto err_stk3x1x_setup_irq;
#endif		
	device_init_wakeup(&client->dev, true);

	ps_data->als_cdev = sensors_light_cdev;
	ps_data->als_cdev.sensors_enable = stk_als_enable_set;
	ps_data->als_cdev.sensors_poll_delay = stk_als_poll_delay_set;
	err = sensors_classdev_register(&ps_data->als_input_dev->dev, &ps_data->als_cdev);
	if (err)
		goto err_ls_class_sysfs;

	ps_data->ps_cdev = sensors_proximity_cdev;
	ps_data->ps_cdev.sensors_enable = stk_ps_enable_set;
	ps_data->ps_cdev.sensors_poll_delay = stk_ps_poll_delay_set;
	err = sensors_classdev_register(&ps_data->ps_input_dev->dev, &ps_data->ps_cdev);
	if (err)
		goto err_ps_class_sysfs;
	
//#ifdef CONFIG_HAS_EARLYSUSPEND
#if 0
	ps_data->stk_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ps_data->stk_early_suspend.suspend = stk3x1x_early_suspend;
	ps_data->stk_early_suspend.resume = stk3x1x_late_resume;
	register_early_suspend(&ps_data->stk_early_suspend);
#endif
//modify(add) by junfeng.zhou.sz for add power supply begin . 20140214
#ifdef POWER_REGULATOR
    	err = stk3x1x_power_init(ps_data, true);
	if (err)
		goto err_power_init;

	err = stk3x1x_power_ctl(ps_data, true);
	if (err)
		goto err_power_on;

	ps_data->als_enabled = false;
	ps_data->ps_enabled = false;
	
	err = stk3x1x_power_ctl(ps_data, false);
	if (err)
		goto err_init_all_setting_power;
#endif
//modify(add) by junfeng.zhou.sz for add power supply end . 
	printk(KERN_INFO "%s: probe successfully\n", __func__);
	return 0;

//modify(add) by junfeng.zhou.sz for add power supply begin . 20140214
#ifdef POWER_REGULATOR
err_init_all_setting_power:
	stk3x1x_power_ctl(ps_data, false);
err_power_on:
	stk3x1x_power_init(ps_data, false);
err_power_init: 
#endif
err_ps_class_sysfs:
	sensors_classdev_unregister(&ps_data->ps_cdev);
err_ls_class_sysfs:
	sensors_classdev_unregister(&ps_data->als_cdev);

//modify(add) by junfeng.zhou.sz for add power supply end . 
#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
err_stk3x1x_setup_irq:
	free_irq(ps_data->irq, ps_data);
	#ifdef SPREADTRUM_PLATFORM	
		sprd_free_gpio_irq(ps_data->int_pin);		
	#else	
		gpio_free(ps_data->int_pin);	
	#endif	
#endif
err_setup_input_device:
#ifdef STK_GES
	sysfs_remove_group(&ps_data->ges_input_dev->dev.kobj, &stk_ges_attribute_group);	
	input_unregister_device(ps_data->ges_input_dev);	
	input_free_device(ps_data->ges_input_dev);	
#endif	
	sysfs_remove_group(&ps_data->ps_input_dev->dev.kobj, &stk_ps_attribute_group);	
	sysfs_remove_group(&ps_data->als_input_dev->dev.kobj, &stk_als_attribute_group);	
	input_unregister_device(ps_data->ps_input_dev);		
	input_unregister_device(ps_data->als_input_dev);	
	input_free_device(ps_data->ps_input_dev);	
	input_free_device(ps_data->als_input_dev);
err_init_all_setting:	
#ifdef STK_POLL_ALS		
	hrtimer_try_to_cancel(&ps_data->als_timer);
	destroy_workqueue(ps_data->stk_als_wq);	
#endif	
#ifdef STK_TUNE0
	destroy_workqueue(ps_data->stk_ps_tune0_wq);	
#endif	
#ifdef STK_POLL_PS	
	hrtimer_try_to_cancel(&ps_data->ps_timer);	
	destroy_workqueue(ps_data->stk_ps_wq);	
#endif		
#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
	destroy_workqueue(ps_data->stk_wq);	
#endif		
err_als_input_allocate:
#ifdef STK_POLL_PS
    	wake_lock_destroy(&ps_data->ps_nosuspend_wl);	
#endif	
    	wake_lock_destroy(&ps_data->ps_wakelock);	
    	mutex_destroy(&stk_i2c_lock);
    	mutex_destroy(&ps_data->io_als_lock);
    	mutex_destroy(&ps_data->io_lock);
    	if (client->dev.of_node && (plat_data != NULL))
		devm_kfree(&client->dev, plat_data);
		kfree(ps_data);
    	return err;
}


static int stk3x1x_remove(struct i2c_client *client)
{
	struct stk3x1x_data *ps_data = i2c_get_clientdata(client);

	sensors_classdev_unregister(&ps_data->ps_cdev);
	sensors_classdev_unregister(&ps_data->als_cdev);
	device_init_wakeup(&client->dev, false);
#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
	free_irq(ps_data->irq, ps_data);
	#ifdef SPREADTRUM_PLATFORM	
		sprd_free_gpio_irq(ps_data->int_pin);		
	#else	
		gpio_free(ps_data->int_pin);	
	#endif	
#endif	/* #if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS)) */	


#ifdef STK_GES	
	sysfs_remove_group(&ps_data->ges_input_dev->dev.kobj, &stk_ges_attribute_group);	
	input_unregister_device(ps_data->ges_input_dev);
	input_free_device(ps_data->ges_input_dev);		
#endif	
#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
	destroy_workqueue(ps_data->stk_wq);	
#endif	
	sysfs_remove_group(&ps_data->ps_input_dev->dev.kobj, &stk_ps_attribute_group);	
	sysfs_remove_group(&ps_data->als_input_dev->dev.kobj, &stk_als_attribute_group);	
	input_unregister_device(ps_data->ps_input_dev);		
	input_unregister_device(ps_data->als_input_dev);	
	input_free_device(ps_data->ps_input_dev);	
	input_free_device(ps_data->als_input_dev);	
#ifdef STK_POLL_ALS		
	hrtimer_try_to_cancel(&ps_data->als_timer);	
	destroy_workqueue(ps_data->stk_als_wq);	
#endif	
#ifdef STK_TUNE0
	destroy_workqueue(ps_data->stk_ps_tune0_wq);	
#endif	
#ifdef STK_POLL_PS
	hrtimer_try_to_cancel(&ps_data->ps_timer);	
	destroy_workqueue(ps_data->stk_ps_wq);		
	wake_lock_destroy(&ps_data->ps_nosuspend_wl);	
#endif	
	wake_lock_destroy(&ps_data->ps_wakelock);	
    	mutex_destroy(&stk_i2c_lock);
    	mutex_destroy(&ps_data->io_als_lock);
    	mutex_destroy(&ps_data->io_lock);
	kfree(ps_data);
	
    	return 0;
}

static const struct i2c_device_id stk_ps_id[] =
{
    	{ DEVICE_NAME, 0},
    	{}
};
MODULE_DEVICE_TABLE(i2c, stk_ps_id);

static struct i2c_driver stk_ps_driver =
{
    	.driver = {
        	.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		//modify(add) by junfeng.zhou.sz for fix i2c timeout error with 661727 begin . 20140501	
		.pm = &stk3x1x_pm_ops,		
		//modify(add) by junfeng.zhou.sz for fix i2c timeout error with 661727 end . 
    	},
    	.probe = stk3x1x_probe,
    	.remove = stk3x1x_remove,
    	.id_table = stk_ps_id,
};


static int __init stk3x1x_init(void)
{
	int ret;
//modify(add) by junfeng.zhou.sz for delete debug info with 586473 begin . 20140109
	//printk(KERN_INFO "@@@@%s",__func__);
//modify(add) by junfeng.zhou.sz for delete debug info with 586473 end . 
    	ret = i2c_add_driver(&stk_ps_driver);
    	if (ret)
	{
		i2c_del_driver(&stk_ps_driver);
        	return ret;
	}
    	return 0;
}

static void __exit stk3x1x_exit(void)
{
    	i2c_del_driver(&stk_ps_driver);	
}

module_init(stk3x1x_init);
module_exit(stk3x1x_exit);
MODULE_AUTHOR("Lex Hsieh <lex_hsieh@sensortek.com.tw>");
MODULE_DESCRIPTION("Sensortek stk3x1x Proximity Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
