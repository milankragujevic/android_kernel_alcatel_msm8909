/*****************************************************************************
 *
 * Copyright (c) 2014 mCube, Inc.  All rights reserved.
 *
 * This source is subject to the mCube Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of mCube Inc.
 *
 * All other rights reserved.
 *
 * This code and information are provided "as is" without warranty of any
 * kind, either expressed or implied, including but not limited to the
 * implied warranties of merchantability and/or fitness for a
 * particular purpose.
 *
 * The following software/firmware and/or related documentation ("mCube Software")
 * have been modified by mCube Inc. All revisions are subject to any receiver's
 * applicable license agreements with mCube Inc.
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
 *
 *****************************************************************************/ 

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
//#include <linux/earlysuspend.h>
#include	<linux/pm.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>

//#include <mach/system.h>
#include <mach/hardware.h>
#include <linux/fs.h>
#include	<linux/sensors.h>

//=== CONFIGURATIONS ==========================================================
#define DOT_CALI
//#define _MC3XXX_DEBUG_ON_

//=============================================================================
#ifdef _MC3XXX_DEBUG_ON_
    #define mcprintkreg(x...)     printk(x)
    #define mcprintkfunc(x...)    printk(x)
    #define GSE_ERR(x...) 	      printk(x)
    #define GSE_LOG(x...) 	      printk(x)
#else
    #define mcprintkreg(x...)
    #define mcprintkfunc(x...)
    #define GSE_ERR(x...)
    #define GSE_LOG(x...)
#endif

//=============================================================================
#define SENSOR_NAME              "mc3xxx"
#define ACCEL_INPUT_DEV_NAME	"accelerometer"
#define SENSOR_DRIVER_VERSION    "1.1.1"
#define SENSOR_DATA_SIZE         3
#define AVG_NUM                  16

//=============================================================================
/* addresses to scan */
static union{
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
}u_i2c_addr = {{0x00},};

//=============================================================================
#define G_0            ABS_X
#define G_1            ABS_Y
#define G_2            ABS_Z
#define G_0_REVERSE    1
#define G_1_REVERSE    1
#define G_2_REVERSE    1

#define GRAVITY_1G_VALUE    1000

//=============================================================================
#define MC3XXX_CONVERT_PARAMETER    (1.5f * (9.80665f) / 256.0f)
#define MC3XXX_DISPLAY_NAME         SENSOR_NAME
#define MC3XXX_DIPLAY_VENDOR        "mCube"

//=============================================================================
#define MC3XXX_AXIS_X      0
#define MC3XXX_AXIS_Y      1
#define MC3XXX_AXIS_Z      2
#define MC3XXX_AXES_NUM    3
#define MC3XXX_DATA_LEN    6

#define MC3XXX_XOUT_REG						0x00
#define MC3XXX_YOUT_REG						0x01
#define MC3XXX_ZOUT_REG						0x02
#define MC3XXX_Tilt_Status_REG				0x03
#define MC3XXX_SAMPLING_RATE_STATUS_REG     0x04
#define MC3XXX_SLEEP_COUNT_REG 				0x05
#define MC3XXX_INTERRUPT_ENABLE_REG         0x06
#define MC3XXX_MODE_FEATURE_REG				0x07
#define MC3XXX_SAMPLE_RATE_REG				0x08
#define MC3XXX_TAP_DETECTION_ENABLE_REG		0x09
#define MC3XXX_TAP_DWELL_REJECT_REG			0x0a
#define MC3XXX_DROP_CONTROL_REG				0x0b
#define MC3XXX_SHAKE_DEBOUNCE_REG			0x0c
#define MC3XXX_XOUT_EX_L_REG				0x0d
#define MC3XXX_XOUT_EX_H_REG				0x0e
#define MC3XXX_YOUT_EX_L_REG				0x0f
#define MC3XXX_YOUT_EX_H_REG				0x10
#define MC3XXX_ZOUT_EX_L_REG				0x11
#define MC3XXX_ZOUT_EX_H_REG				0x12
#define MC3XXX_CHIP_ID_REG					0x18
#define MC3XXX_RANGE_CONTROL_REG			0x20
#define MC3XXX_SHAKE_THRESHOLD_REG			0x2B
#define MC3XXX_UD_Z_TH_REG					0x2C
#define MC3XXX_UD_X_TH_REG					0x2D
#define MC3XXX_RL_Z_TH_REG					0x2E
#define MC3XXX_RL_Y_TH_REG					0x2F
#define MC3XXX_FB_Z_TH_REG					0x30
#define MC3XXX_DROP_THRESHOLD_REG			0x31
#define MC3XXX_TAP_THRESHOLD_REG			0x32

#define MC3XXX_HIGH_END    0x01
/*******MC3210/20 define this**********/
#define MCUBE_8G_14BIT     0x10

#define MC3XXX_LOW_END     0x02
/*******mc3xxx define this**********/
#define MCUBE_1_5G_8BIT    0x20
#define MC3XXX_MODE_DEF    0x43

#define MC3XXX_RESOLUTION_LOW     1
#define MC3XXX_RESOLUTION_HIGH    2

/***********************************************
 *** RETURN CODE
 ***********************************************/
#define MC3XXX_RETCODE_SUCCESS                 (0)
#define MC3XXX_RETCODE_ERROR_I2C               (-1)
#define MC3XXX_RETCODE_ERROR_STATUS            (-3)
#define MC3XXX_RETCODE_ERROR_SETUP             (-4)
#define MC3XXX_RETCODE_ERROR_GET_DATA          (-5)
#define MC3XXX_RETCODE_ERROR_IDENTIFICATION    (-6)

/***********************************************
 *** PRODUCT ID
 ***********************************************/
#define MC3XXX_PCODE_3210     0x90
#define MC3XXX_PCODE_3230     0x19
#define MC3XXX_PCODE_3253     0x88
#define MC3XXX_PCODE_3410     0xA8
#define MC3XXX_PCODE_3410N    0xB8
#define MC3XXX_PCODE_3430     0x29
#define MC3XXX_PCODE_3430N    0x39
#define MC3XXX_PCODE_3510     0x40
#define MC3XXX_PCODE_3530     0x30
#define MC3XXX_PCODE_3216     0x10
#define MC3XXX_PCODE_3433     0x60

#define MC3XXX_PCODE_RESERVE_1    0x20
#define MC3XXX_PCODE_RESERVE_2    0x11
#define MC3XXX_PCODE_RESERVE_3    0x21
#define MC3XXX_PCODE_RESERVE_4    0x61
#define MC3XXX_PCODE_RESERVE_5    0xA0
#define MC3XXX_PCODE_RESERVE_6    0xE0
#define MC3XXX_PCODE_RESERVE_7    0x91
#define MC3XXX_PCODE_RESERVE_8    0xA1
#define MC3XXX_PCODE_RESERVE_9    0xE1

#define MC3XXX_PCODE_RESERVE_10    0x99

//=============================================================================
#define MC3XXX_I2C_NAME            SENSOR_NAME
#define SENSOR_DEV_COUNT           1
#define SENSOR_DURATION_MAX        200
#define SENSOR_DURATION_MIN        10
#define SENSOR_DURATION_DEFAULT    100

#define INPUT_FUZZ    0
#define INPUT_FLAT    0

#define MC3XXX_BUFSIZE    256

//=============================================================================
static unsigned char    s_bResolution = 0x00;
static unsigned char    s_bPCODE      = 0x00;
static unsigned char    s_bPCODER     = 0x00;
static unsigned char    s_bHWID       = 0x00;
static unsigned char    s_bMPOL       = 0x00;

static unsigned char    s_baOTP_OffsetData[6] = { 0 };
//=============================================================================
#ifdef DOT_CALI

//#define CALIB_PATH              "/data/data/com.mcube.acc/files/mcube-calib.txt"
#define DATA_PATH              "/sdcard/mcube-register-map.txt"
static char file_path[MC3XXX_BUFSIZE] = "/data/data/com.mcube.acc/files/mcube-calib.txt";
//static char factory_path[MC3XXX_BUFSIZE] ="/data/data/com.mcube.acc/files/fac-calib.txt";
static       unsigned short     mc3xxx_i2c_auto_probe_addr[] = { 0x4C, 0x6C, 0x4E, 0x6D, 0x6E, 0x6F };

typedef struct {
	unsigned short	x;		/**< X axis */
	unsigned short	y;		/**< Y axis */
	unsigned short	z;		/**< Z axis */
} GSENSOR_VECTOR3D;

static GSENSOR_VECTOR3D gsensor_gain = { 0 };
static struct miscdevice mc3xxx_device;

static struct file * fd_file = NULL;

static mm_segment_t oldfs = { 0 };
static unsigned char offset_buf[9] = { 0 };
static signed int offset_data[3] = { 0 };
s16 G_RAW_DATA[3] = { 0 };
static signed int gain_data[3] = { 0 };
static signed int enable_RBM_calibration = 0;

//=============================================================================
#define SENSOR_DMARD_IOCTL_BASE          234

#define IOCTL_SENSOR_SET_DELAY_ACCEL     _IO(SENSOR_DMARD_IOCTL_BASE, 100)
#define IOCTL_SENSOR_GET_DELAY_ACCEL     _IO(SENSOR_DMARD_IOCTL_BASE, 101)
#define IOCTL_SENSOR_GET_STATE_ACCEL     _IO(SENSOR_DMARD_IOCTL_BASE, 102)
#define IOCTL_SENSOR_SET_STATE_ACCEL     _IO(SENSOR_DMARD_IOCTL_BASE, 103)
#define IOCTL_SENSOR_GET_NAME            _IO(SENSOR_DMARD_IOCTL_BASE, 301)
#define IOCTL_SENSOR_GET_VENDOR          _IO(SENSOR_DMARD_IOCTL_BASE, 302)
#define IOCTL_SENSOR_GET_CONVERT_PARA    _IO(SENSOR_DMARD_IOCTL_BASE, 401)

//=============================================================================
#define GSENSOR                                0x95

#define GSENSOR_IOCTL_INIT                     _IO(GSENSOR,  0x01)
#define GSENSOR_IOCTL_READ_CHIPINFO            _IOR(GSENSOR, 0x02, int)
#define GSENSOR_IOCTL_READ_SENSORDATA          _IOR(GSENSOR, 0x03, int)
#define GSENSOR_IOCTL_READ_OFFSET              _IOR(GSENSOR, 0x04, GSENSOR_VECTOR3D)
#define GSENSOR_IOCTL_READ_GAIN                _IOR(GSENSOR, 0x05, GSENSOR_VECTOR3D)
#define GSENSOR_IOCTL_READ_RAW_DATA            _IOR(GSENSOR, 0x06, int)
#define GSENSOR_IOCTL_GET_CALI                 _IOW(GSENSOR, 0x07, SENSOR_DATA)
#define GSENSOR_IOCTL_CLR_CALI                 _IO(GSENSOR, 0x08)
#define GSENSOR_MCUBE_IOCTL_READ_RBM_DATA      _IOR(GSENSOR, 0x09, SENSOR_DATA)
#define GSENSOR_MCUBE_IOCTL_SET_RBM_MODE       _IO(GSENSOR, 0x0a)
#define GSENSOR_MCUBE_IOCTL_CLEAR_RBM_MODE     _IO(GSENSOR, 0x0b)
#define GSENSOR_MCUBE_IOCTL_SET_CALI           _IOW(GSENSOR, 0x0c, SENSOR_DATA)
#define GSENSOR_MCUBE_IOCTL_REGISTER_MAP       _IO(GSENSOR, 0x0d)
#define GSENSOR_IOCTL_SET_CALI_MODE            _IOW(GSENSOR, 0x0e,int)
#define GSENSOR_MCUBE_IOCTL_READ_PRODUCT_ID    _IOR(GSENSOR, 0x0f, int)
#define GSENSOR_MCUBE_IOCTL_READ_FILEPATH      _IOR(GSENSOR, 0x10, char[256])

typedef struct{
	int x;
	int y;
	int z;
}SENSOR_DATA;

static int load_cali_flg = 0;

#endif  // END OF #ifdef DOT_CALI

//=============================================================================
#define MC3XXX_WAKE       1
#define MC3XXX_SNIFF      2
#define MC3XXX_STANDBY    3

#define MCUBE_RREMAP(nDataX, nDataY)                                                  \
            if (MC3XXX_PCODE_3253 == s_bPCODE)                                        \
            {                                                                         \
                int    _nTemp = 0;                                                    \
                                                                                      \
                _nTemp = nDataX;                                                      \
                nDataX = nDataY;                                                      \
                nDataY = -_nTemp;                                                     \
                GSE_LOG("[%s] 3250 read remap\n", __FUNCTION__);                      \
            }                                                                         \
            else                                                                      \
            {                                                                         \
                if (s_bMPOL & 0x01)    nDataX = -nDataX;                              \
                if (s_bMPOL & 0x02)    nDataY = -nDataY;                              \
                GSE_LOG("[%s] 35X0 remap [s_bMPOL: %d]\n", __FUNCTION__, s_bMPOL);    \
            }

#define MCUBE_WREMAP(nDataX, nDataY)                                                  \
            if (MC3XXX_PCODE_3253 == s_bPCODE)                                        \
            {                                                                         \
                int    _nTemp = 0;                                                    \
                                                                                      \
                _nTemp = nDataX;                                                      \
                nDataX = -nDataY;                                                     \
                nDataY = _nTemp;                                                      \
                GSE_LOG("[%s] 3250 write remap\n", __FUNCTION__);                     \
            }                                                                         \
            else                                                                      \
            {                                                                         \
                if (s_bMPOL & 0x01)    nDataX = -nDataX;                              \
                if (s_bMPOL & 0x02)    nDataY = -nDataY;                              \
                GSE_LOG("[%s] 35X0 remap [s_bMPOL: %d]\n", __FUNCTION__, s_bMPOL);    \
            }

#define IS_MCFM12()    ((0xC0 <= s_bHWID) && (s_bHWID <= 0xCF))
#define IS_MCFM3X()    ((0x20 == s_bHWID) || ((0x22 <= s_bHWID) && (s_bHWID <= 0x2F)))


struct dev_data {
	struct i2c_client *client;
};

static struct dev_data dev = { 0 };

struct acceleration {
	int x;
	int y;
	int z;
};

struct mc3xxx_data {
	struct mutex lock;
	struct i2c_client *client;
	struct sensors_classdev cdev;
	struct work_struct  work;
	struct workqueue_struct *mc3xxx_wq;
	struct hrtimer timer;
	struct device *device;
	struct input_dev *input_dev;
	int use_count;
	int enabled;
	volatile unsigned int duration;
	int use_irq; 
	int irq;
	unsigned long irqflags;
	int gpio;
	unsigned int map[3];
	int inv[3];

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

static struct sensors_classdev mc3xxx_acc_cdev = {
	.name = "mc3xxx-accel",
	.vendor = "Mcube",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "156.8",
	.resolution = "0.01",
	.sensor_power = "0.01",
	.min_delay = 5000,
	.delay_msec = 200,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

//=============================================================================
enum mc3xx0_orientation
{
    MC3XXX_TOP_LEFT_DOWN = 0,
    MC3XXX_TOP_RIGHT_DOWN,
    MC3XXX_TOP_RIGHT_UP,
    MC3XXX_TOP_LEFT_UP,
    MC3XXX_BOTTOM_LEFT_DOWN,
    MC3XXX_BOTTOM_RIGHT_DOWN,
    MC3XXX_BOTTOM_RIGHT_UP,
    MC3XXX_BOTTOM_LEFT_UP
};

/*enum mc3xx0_axis
{
    MC3XXX_AXIS_X = 0,
    MC3XXX_AXIS_Y,
    MC3XXX_AXIS_Z,
    MC3XXX_AXIS_NUM
};*/

struct mc3xx0_hwmsen_convert
{
    signed char sign[3];
    unsigned char map[3];
};

// Transformation matrix for chip mounting position
static const struct mc3xx0_hwmsen_convert mc3xx0_cvt[] =
{
    {{ 1,  1,  1}, {MC3XXX_AXIS_X, MC3XXX_AXIS_Y, MC3XXX_AXIS_Z}},    // 0: top   , left-down
    {{-1,  1,  1}, {MC3XXX_AXIS_Y, MC3XXX_AXIS_X, MC3XXX_AXIS_Z}},    // 1: top   , right-down
    {{-1, -1,  1}, {MC3XXX_AXIS_X, MC3XXX_AXIS_Y, MC3XXX_AXIS_Z}},    // 2: top   , right-up
    {{ 1, -1,  1}, {MC3XXX_AXIS_Y, MC3XXX_AXIS_X, MC3XXX_AXIS_Z}},    // 3: top   , left-up
    {{-1,  1, -1}, {MC3XXX_AXIS_X, MC3XXX_AXIS_Y, MC3XXX_AXIS_Z}},    // 4: bottom, left-down
    {{ 1,  1, -1}, {MC3XXX_AXIS_Y, MC3XXX_AXIS_X, MC3XXX_AXIS_Z}},    // 5: bottom, right-down
    {{ 1, -1, -1}, {MC3XXX_AXIS_X, MC3XXX_AXIS_Y, MC3XXX_AXIS_Z}},    // 6: bottom, right-up
    {{-1, -1, -1}, {MC3XXX_AXIS_Y, MC3XXX_AXIS_X, MC3XXX_AXIS_Z}},    // 7: bottom, left-up
};

static unsigned char mc3xx0_current_placement = MC3XXX_BOTTOM_LEFT_UP; // current soldered placement

//=============================================================================
volatile static unsigned int sensor_duration = SENSOR_DURATION_DEFAULT;
volatile static short sensor_state_flag = 1;

//=============================================================================
static int MC3XXX_ReadRegMap(struct i2c_client *p_i2c_client, u8 *pbUserBuf);

//=============================================================================
static ssize_t mc3xxx_map_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *data = NULL;
	int i = 0;
	data = i2c_get_clientdata(client);
	for (i = 0; i< 3; i++)
	{
		if(data->inv[i] == 1)
		{
			switch(data->map[i])
			{
				case ABS_X:
					buf[i] = 'x';
					break;
				case ABS_Y:
					buf[i] = 'y';
					break;
				case ABS_Z:
					buf[i] = 'z';
					break;
				default:
					buf[i] = '_';
					break;
			}
		}
		else
		{
			switch(data->map[i])
			{
				case ABS_X:
					buf[i] = 'X';
					break;
				case ABS_Y:
					buf[i] = 'Y';
					break;
				case ABS_Z:
					buf[i] = 'Z';
					break;
				default:
					buf[i] = '-';
					break;
			}
		}
	}
	sprintf(buf+3,"\r\n");
	return 5;
}

//=============================================================================
static ssize_t mc3xxx_map_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *data = NULL;
	int i = 0;
	data = i2c_get_clientdata(client);

	if(count < 3) return -EINVAL;

	for(i = 0; i< 3; i++)
	{
		switch(buf[i])
		{
			case 'x':
				data->map[i] = ABS_X;
				data->inv[i] = 1;
				break;
			case 'y':
				data->map[i] = ABS_Y;
				data->inv[i] = 1;
				break;
			case 'z':
				data->map[i] = ABS_Z;
				data->inv[i] = 1;
				break;
			case 'X':
				data->map[i] = ABS_X;
				data->inv[i] = -1;
				break;
			case 'Y':
				data->map[i] = ABS_Y;
				data->inv[i] = -1;
				break;
			case 'Z':
				data->map[i] = ABS_Z;
				data->inv[i] = -1;
				break;
			default:
				return -EINVAL;
		}
	}

	return count;
}

/*****************************************
 *** mc3xxx_validate_sensor_IC
 *****************************************/
static int mc3xxx_validate_sensor_IC(unsigned char *pbPCode, unsigned char *pbHwID)
{
    GSE_LOG("[%s] *pbPCode: 0x%02X, *pbHwID: 0x%02X\n", __FUNCTION__, *pbPCode, *pbHwID);

    if (   (0x01 == *pbHwID)
        || (0x03 == *pbHwID)
        || ((0x04 <= *pbHwID) && (*pbHwID <= 0x0F)))
    {
        if ((MC3XXX_PCODE_3210 == *pbPCode) || (MC3XXX_PCODE_3230 == *pbPCode) || (MC3XXX_PCODE_3253 == *pbPCode))
            return (MC3XXX_RETCODE_SUCCESS);
    }
    else if (   (0x02 == *pbHwID)
             || (0x21 == *pbHwID)
             || ((0x10 <= *pbHwID) && (*pbHwID <= 0x1F)))
    {
        if (   (MC3XXX_PCODE_3210 == *pbPCode) || (MC3XXX_PCODE_3230  == *pbPCode)
            || (MC3XXX_PCODE_3253 == *pbPCode)
            || (MC3XXX_PCODE_3410 == *pbPCode) || (MC3XXX_PCODE_3410N == *pbPCode)
            || (MC3XXX_PCODE_3430 == *pbPCode) || (MC3XXX_PCODE_3430N == *pbPCode))
        {
            return (MC3XXX_RETCODE_SUCCESS);
        }
    }
    else if ((0xC0 <= *pbHwID) && (*pbHwID <= 0xCF))
    {
        *pbPCode = (*pbPCode & 0x71);

        if ((MC3XXX_PCODE_3510 == *pbPCode) || (MC3XXX_PCODE_3530 == *pbPCode))
            return (MC3XXX_RETCODE_SUCCESS);
    }
    else if ((0x20 == *pbHwID) || ((0x22 <= *pbHwID) && (*pbHwID <= 0x2F)))
    {
        *pbPCode = (*pbPCode & 0xF1);

        if (   (MC3XXX_PCODE_3210      == *pbPCode) || (MC3XXX_PCODE_3216      == *pbPCode) || (MC3XXX_PCODE_3433      == *pbPCode)
            || (MC3XXX_PCODE_RESERVE_1 == *pbPCode) || (MC3XXX_PCODE_RESERVE_2 == *pbPCode) || (MC3XXX_PCODE_RESERVE_3 == *pbPCode)
            || (MC3XXX_PCODE_RESERVE_4 == *pbPCode) || (MC3XXX_PCODE_RESERVE_5 == *pbPCode) || (MC3XXX_PCODE_RESERVE_6 == *pbPCode)
            || (MC3XXX_PCODE_RESERVE_7 == *pbPCode) || (MC3XXX_PCODE_RESERVE_8 == *pbPCode) || (MC3XXX_PCODE_RESERVE_9 == *pbPCode))
        {
            return (MC3XXX_RETCODE_SUCCESS);
        }
    }

    return (MC3XXX_RETCODE_ERROR_IDENTIFICATION);
}

/*****************************************
 *** mc3xxx_set_resolution
 *****************************************/
static void mc3xxx_set_resolution(void)
{
    GSE_LOG("[%s]\n", __FUNCTION__);

    switch (s_bPCODE)
    {
    case MC3XXX_PCODE_3230:
    case MC3XXX_PCODE_3430:
    case MC3XXX_PCODE_3430N:
    case MC3XXX_PCODE_3530:
    case MC3XXX_PCODE_3433:
         s_bResolution = MC3XXX_RESOLUTION_LOW;
         break;

    case MC3XXX_PCODE_3210:
    case MC3XXX_PCODE_3253:
    case MC3XXX_PCODE_3410:
    case MC3XXX_PCODE_3410N:
    case MC3XXX_PCODE_3510:
    case MC3XXX_PCODE_3216:
         s_bResolution = MC3XXX_RESOLUTION_HIGH;
         break;

    // === RESERVED ==================================BGN===
    // === (move to normal section once it is confirmed) ===
    case MC3XXX_PCODE_RESERVE_10:
         GSE_ERR("RESERVED ONLINE!\n");
         // TODO: should have a default configuration...
         break;

    case MC3XXX_PCODE_RESERVE_1:
    case MC3XXX_PCODE_RESERVE_3:
    case MC3XXX_PCODE_RESERVE_4:
    case MC3XXX_PCODE_RESERVE_5:
    case MC3XXX_PCODE_RESERVE_6:
    case MC3XXX_PCODE_RESERVE_8:
    case MC3XXX_PCODE_RESERVE_9:
         GSE_ERR("RESERVED ONLINE!\n");
         s_bResolution = MC3XXX_RESOLUTION_LOW;
         break;

    case MC3XXX_PCODE_RESERVE_2:
    case MC3XXX_PCODE_RESERVE_7:
         GSE_ERR("RESERVED ONLINE!\n");
         s_bResolution = MC3XXX_RESOLUTION_HIGH;
         break;
    // === RESERVED ==================================END===

    default:
         GSE_ERR("ERR: no resolution assigned!\n");
         break;
    }

    GSE_LOG("[%s] s_bResolution: %d\n", __FUNCTION__, s_bResolution);
}

/*****************************************
 *** mc3xxx_set_sample_rate
 *****************************************/
static void mc3xxx_set_sample_rate(struct i2c_client *pt_i2c_client)
{
    unsigned char    _baDataBuf[2] = { 0 };

    GSE_LOG("[%s]\n", __FUNCTION__);

    _baDataBuf[0] = MC3XXX_SAMPLE_RATE_REG;
    _baDataBuf[1] = 0x00;

    if (IS_MCFM12() || IS_MCFM3X())
    {
        unsigned char    _baData2Buf[2] = { 0 };

        _baData2Buf[0] = 0x2A;
        i2c_master_send(pt_i2c_client, &(_baData2Buf[0]), 1);
        i2c_master_recv(pt_i2c_client, &(_baData2Buf[0]), 1);

        GSE_LOG("[%s] REG(0x2A) = 0x%02X\n", __FUNCTION__, _baData2Buf[0]);

        _baData2Buf[0] = (_baData2Buf[0] & 0xC0);

        switch (_baData2Buf[0])
        {
        case 0x00:    _baDataBuf[1] = 0x00;                                                    break;
        case 0x40:    _baDataBuf[1] = 0x08;                                                    break;
        case 0x80:    _baDataBuf[1] = 0x09;                                                    break;
        case 0xC0:    _baDataBuf[1] = 0x0A;                                                    break;

        default:      GSE_ERR("[%s] no chance to get here... check code!\n", __FUNCTION__);    break;
        }
    }

    i2c_master_send(pt_i2c_client, _baDataBuf, 0x2);
}

/*****************************************
 *** mc3xxx_config_range
 *****************************************/
static void mc3xxx_config_range(struct i2c_client *pt_i2c_client)
{
    unsigned char    _baDataBuf[2] = { 0 };

    _baDataBuf[0] = MC3XXX_RANGE_CONTROL_REG;
    _baDataBuf[1] = 0x3F;

    if (MC3XXX_RESOLUTION_LOW == s_bResolution)
        _baDataBuf[1] = 0x32;

    if (IS_MCFM12() || IS_MCFM3X())
    {
        if (MC3XXX_RESOLUTION_LOW == s_bResolution)
            _baDataBuf[1] = 0x02;
        else
            _baDataBuf[1] = 0x25;
    }

    i2c_master_send(pt_i2c_client, _baDataBuf, 0x2);

    GSE_LOG("[%s] set 0x%X\n", __FUNCTION__, _baDataBuf[1]);
}

/*****************************************
 *** mc3xxx_set_gain
 *****************************************/
static void mc3xxx_set_gain(void)
{
    gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = 1024;

    if (MC3XXX_RESOLUTION_LOW == s_bResolution)
    {
        gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = 86;

        if (IS_MCFM12() || IS_MCFM3X())
        {
            gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = 64;
        }
    }
    
    GSE_LOG("[%s] gain: %d / %d / %d\n", __FUNCTION__, gsensor_gain.x, gsensor_gain.y, gsensor_gain.z);
}

//=============================================================================
static int mc3xxx_enable(struct mc3xxx_data *data, int enable);

static ssize_t mc3xxx_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{	
	struct i2c_client *client = container_of(mc3xxx_device.parent, struct i2c_client, dev);

	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", mc3xxx->enabled);
}

//=============================================================================
static ssize_t mc3xxx_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	bool new_enable = false;

	struct i2c_client *client = container_of(mc3xxx_device.parent, struct i2c_client, dev);
	
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	if (sysfs_streq(buf, "1"))
		new_enable = true;
	else if (sysfs_streq(buf, "0"))
		new_enable = false;
	else {
		pr_debug("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	mc3xxx_enable(mc3xxx, new_enable);

	return count;
}

//=============================================================================
static ssize_t mc3xxx_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sensor_duration);
}

//=============================================================================
static ssize_t mc3xxx_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data = 0;
	int error = 0;

	error = strict_strtoul(buf, 10, &data);

	if (error)
		return error;

	if (data > SENSOR_DURATION_MAX)
		data = SENSOR_DURATION_MAX;
	if (data < SENSOR_DURATION_MIN)
		data = SENSOR_DURATION_MIN;

	sensor_duration = data;

	return count;
}

//=============================================================================
static ssize_t mc3xxx_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%s\n", SENSOR_DRIVER_VERSION);
}

//=============================================================================
static ssize_t mc3xxx_chip_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned char baChipID[4] = { 0 };
    struct i2c_client *client = container_of(mc3xxx_device.parent, struct i2c_client, dev);

    i2c_smbus_read_i2c_block_data(client, 0x3C, 4, baChipID);

    return sprintf(buf, "%02X-%02X-%02X-%02X\n", baChipID[3], baChipID[2], baChipID[1], baChipID[0]);
}

//=============================================================================
static ssize_t mc3xxx_position_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    printk("%s called\n", __func__);
    return sprintf(buf, "%d\n", mc3xx0_current_placement);
}

//=============================================================================
static ssize_t mc3xxx_position_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned long position = 0;

    printk("%s called\n", __func__);

    position = simple_strtoul(buf, NULL,10);

    if (position < 8)
        mc3xx0_current_placement = position;

    return count;
}

//=============================================================================
static ssize_t mc3xxx_regmap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8         _bIndex       = 0;
    u8         _baRegMap[64] = { 0 };
    ssize_t    _tLength      = 0;

    struct i2c_client *client = container_of(mc3xxx_device.parent, struct i2c_client, dev);

    struct mc3xxx_data *data = NULL;

    data = i2c_get_clientdata(client);

    if ((0xA5 == buf[0]) && (0x7B == buf[1]) && (0x40 == buf[2]))
    {
        mutex_lock(&data->lock);
        MC3XXX_ReadRegMap(client, buf);
        mutex_unlock(&data->lock);

        buf[0x21] = s_baOTP_OffsetData[0];
        buf[0x22] = s_baOTP_OffsetData[1];
        buf[0x23] = s_baOTP_OffsetData[2];
        buf[0x24] = s_baOTP_OffsetData[3];
        buf[0x25] = s_baOTP_OffsetData[4];
        buf[0x26] = s_baOTP_OffsetData[5];

        _tLength = 64;
    }
    else
    {
        mutex_lock(&data->lock);
        MC3XXX_ReadRegMap(client, _baRegMap);
        mutex_unlock(&data->lock);
    
        for (_bIndex = 0; _bIndex < 64; _bIndex++)
            _tLength += snprintf((buf + _tLength), (PAGE_SIZE - _tLength), "Reg[0x%02X]: 0x%02X\n", _bIndex, _baRegMap[_bIndex]); 
    }

    return (_tLength);
}

//=============================================================================
static ssize_t mc3xxx_regmap_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
    // reserved
    GSE_LOG("[%s] buf[0]: 0x%02X\n", __FUNCTION__, buf[0]);

    return count;
}

//=============================================================================
static ssize_t mc3xxx_orien_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    GSE_LOG("[%s] mc3xx0_current_placement: %d\n", __FUNCTION__, mc3xx0_current_placement);

	return sprintf(buf, "%d\n", mc3xx0_current_placement);
}

//=============================================================================
static ssize_t mc3xxx_orien_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error;

    GSE_LOG("[%s] mc3xx0_current_placement: %d\n", __FUNCTION__, mc3xx0_current_placement);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (8 > data)
		mc3xx0_current_placement = data;

    GSE_LOG("[%s] data: %ld, mc3xx0_current_placement: %d\n", __FUNCTION__, data, mc3xx0_current_placement);

	return count;
}

//=============================================================================
static DEVICE_ATTR(map     , S_IWUSR | S_IRUGO          , mc3xxx_map_show     , mc3xxx_map_store     );
static DEVICE_ATTR(enable  , S_IRUGO | S_IWUSR | S_IWGRP, mc3xxx_enable_show  , mc3xxx_enable_store  );
static DEVICE_ATTR(delay   , S_IRUGO | S_IWUSR | S_IWGRP, mc3xxx_delay_show   , mc3xxx_delay_store   );
static DEVICE_ATTR(version , S_IRUGO                    , mc3xxx_version_show , NULL                 );
static DEVICE_ATTR(chipid  , S_IRUGO                    , mc3xxx_chip_id_show , NULL                 );
static DEVICE_ATTR(position, S_IRUGO | S_IWUSR | S_IWGRP, mc3xxx_position_show, mc3xxx_position_store);
static DEVICE_ATTR(regmap  , S_IRUGO | S_IWUSR | S_IWGRP, mc3xxx_regmap_show  , mc3xxx_regmap_store  );
static DEVICE_ATTR(orien   , S_IRUGO | S_IWUSR | S_IWGRP, mc3xxx_orien_show   , mc3xxx_orien_store   );

static struct attribute* mc3xxx_attrs[] =
{
	&dev_attr_map.attr,
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_version.attr,
	&dev_attr_chipid.attr,
	&dev_attr_position.attr,
	&dev_attr_regmap.attr,
	&dev_attr_orien.attr,
	NULL
};

static struct attribute_group mc3xxx_group =
{
	.attrs = mc3xxx_attrs,
};

//=============================================================================
static int mc3xxx_chip_init(struct i2c_client *client)
{
    int ret = 0;
    unsigned char  _baDataBuf[2] = { 0 };
    
    _baDataBuf[0] = MC3XXX_MODE_FEATURE_REG;
    _baDataBuf[1] = 0x43;
    ret = i2c_smbus_write_byte_data(client, _baDataBuf[0], _baDataBuf[1]);
    if (ret < 0)
    {
        printk(KERN_ERR"%s: write i2c error, ret=%d\n", __func__, ret);
        return ret;
    }
    
    mc3xxx_set_resolution();
    mc3xxx_set_sample_rate(client);
    mc3xxx_config_range(client);
    mc3xxx_set_gain();
    
    _baDataBuf[0] = MC3XXX_TAP_DETECTION_ENABLE_REG;
    _baDataBuf[1] = 0x00;
    i2c_master_send(client, _baDataBuf, 0x2);
    
    _baDataBuf[0] = MC3XXX_INTERRUPT_ENABLE_REG;
    _baDataBuf[1] = 0x00;
    i2c_master_send(client, _baDataBuf, 0x2);

    _baDataBuf[0] = 0x2A;
    i2c_master_send(client, &(_baDataBuf[0]), 1);
    i2c_master_recv(client, &(_baDataBuf[0]), 1);
    s_bMPOL = (_baDataBuf[0] & 0x03);

    printk("[%s] init ok.\n", __FUNCTION__);

    return (MC3XXX_RETCODE_SUCCESS);
}

//=============================================================================
int mc3xxx_set_mode(struct i2c_client *client, unsigned char mode) 
{
	int comres = 0;
	unsigned char data = 0;

	if (mode < 4)
	{
		data = (0x40 | mode);
		comres = i2c_smbus_write_byte_data(client, MC3XXX_MODE_FEATURE_REG, data);
	} 

	return comres;
}

#ifdef DOT_CALI
//=============================================================================
struct file *openFile(char *path, int flag, int mode) 
{ 
	struct file *fp = NULL; 
	 
	fp = filp_open(path, flag, mode); 

	if (IS_ERR(fp) || !fp->f_op) 
	{
		GSE_LOG("Calibration File filp_open return NULL\n");
		return NULL; 
	}

	return fp; 
} 
 
//=============================================================================
int readFile(struct file *fp, char *buf, int readlen) 
{ 
	if (fp->f_op && fp->f_op->read) 
		return fp->f_op->read(fp,buf,readlen, &fp->f_pos); 
	else 
		return -1; 
} 

//=============================================================================
int writeFile(struct file *fp, char *buf, int writelen) 
{ 
	if (fp->f_op && fp->f_op->write) 
		return fp->f_op->write(fp,buf,writelen, &fp->f_pos); 
	else 
		return -1; 
}
 
//=============================================================================
int closeFile(struct file *fp) 
{ 
	filp_close(fp, NULL); 

	return 0; 
} 

//=============================================================================
void initKernelEnv(void) 
{ 
	oldfs = get_fs(); 
	set_fs(KERNEL_DS);
	printk(KERN_INFO "initKernelEnv\n");
} 

//=============================================================================
int MC3XXX_WriteCalibration(struct i2c_client *client, int dat[MC3XXX_AXES_NUM])
{
	int err = 0;
	u8 buf[9] = { 0 };
	s16 tmp = 0, x_gain = 0, y_gain = 0, z_gain = 0;
	s32 x_off = 0, y_off = 0, z_off = 0;
    int temp_cali_dat[MC3XXX_AXES_NUM] = { 0 };
	const struct mc3xx0_hwmsen_convert *pCvt = NULL;

    u8  bMsbFilter       = 0x3F;
    s16 wSignBitMask     = 0x2000;
    s16 wSignPaddingBits = 0xC000;
    s32 dwRangePosLimit  = 0x1FFF;
    s32 dwRangeNegLimit  = -0x2000;

	pCvt = &mc3xx0_cvt[mc3xx0_current_placement];

    temp_cali_dat[pCvt->map[MC3XXX_AXIS_X]] = pCvt->sign[MC3XXX_AXIS_X] * dat[MC3XXX_AXIS_X];
    temp_cali_dat[pCvt->map[MC3XXX_AXIS_Y]] = pCvt->sign[MC3XXX_AXIS_Y] * dat[MC3XXX_AXIS_Y];
    temp_cali_dat[pCvt->map[MC3XXX_AXIS_Z]] = pCvt->sign[MC3XXX_AXIS_Z] * dat[MC3XXX_AXIS_Z];

    temp_cali_dat[MC3XXX_AXIS_X] = ((temp_cali_dat[MC3XXX_AXIS_X] * gsensor_gain.x) / GRAVITY_1G_VALUE);
    temp_cali_dat[MC3XXX_AXIS_Y] = ((temp_cali_dat[MC3XXX_AXIS_Y] * gsensor_gain.y) / GRAVITY_1G_VALUE);
    temp_cali_dat[MC3XXX_AXIS_Z] = ((temp_cali_dat[MC3XXX_AXIS_Z] * gsensor_gain.z) / GRAVITY_1G_VALUE);

    MCUBE_WREMAP(temp_cali_dat[MC3XXX_AXIS_X], temp_cali_dat[MC3XXX_AXIS_Y]);

    dat[MC3XXX_AXIS_X] = temp_cali_dat[MC3XXX_AXIS_X];
    dat[MC3XXX_AXIS_Y] = temp_cali_dat[MC3XXX_AXIS_Y];
    dat[MC3XXX_AXIS_Z] = temp_cali_dat[MC3XXX_AXIS_Z];
 
	GSE_LOG("UPDATE dat: (%+3d %+3d %+3d)\n", dat[MC3XXX_AXIS_X], dat[MC3XXX_AXIS_Y], dat[MC3XXX_AXIS_Z]);

    // read register 0x21~0x29
	err  = i2c_smbus_read_i2c_block_data(client, 0x21, 3, &buf[0]);
	err |= i2c_smbus_read_i2c_block_data(client, 0x24, 3, &buf[3]);
	err |= i2c_smbus_read_i2c_block_data(client, 0x27, 3, &buf[6]);

    if (IS_MCFM12() || IS_MCFM3X())
    {
        bMsbFilter       = 0x7F;
        wSignBitMask     = 0x4000;
        wSignPaddingBits = 0x8000;
        dwRangePosLimit  = 0x3FFF;
        dwRangeNegLimit  = -0x4000;
    }

	// get x,y,z offset
    tmp = ((buf[1] & bMsbFilter) << 8) + buf[0];
    if (tmp & wSignBitMask)
        tmp |= wSignPaddingBits;
    x_off = tmp;
    
    tmp = ((buf[3] & bMsbFilter) << 8) + buf[2];
    if (tmp & wSignBitMask)
        tmp |= wSignPaddingBits;
    y_off = tmp;
    
    tmp = ((buf[5] & bMsbFilter) << 8) + buf[4];
    if (tmp & wSignBitMask)
        tmp |= wSignPaddingBits;
    z_off = tmp;
					
	// get x,y,z gain
	x_gain = ((buf[1] >> 7) << 8) + buf[6];
	y_gain = ((buf[3] >> 7) << 8) + buf[7];
	z_gain = ((buf[5] >> 7) << 8) + buf[8];
								
	// prepare new offset
	x_off = x_off + 16 * dat[MC3XXX_AXIS_X] * 256 * 128 / 3 / gsensor_gain.x / (40 + x_gain);
	y_off = y_off + 16 * dat[MC3XXX_AXIS_Y] * 256 * 128 / 3 / gsensor_gain.y / (40 + y_gain);
	z_off = z_off + 16 * dat[MC3XXX_AXIS_Z] * 256 * 128 / 3 / gsensor_gain.z / (40 + z_gain);

	// range check
	if (x_off > dwRangePosLimit) 
	    x_off = dwRangePosLimit;
	else if (x_off < dwRangeNegLimit)
	    x_off = dwRangeNegLimit;

	if (y_off > dwRangePosLimit) 
	    y_off = dwRangePosLimit;
	else if (y_off < dwRangeNegLimit)
	    y_off = dwRangeNegLimit;

	if (z_off > dwRangePosLimit) 
	    z_off = dwRangePosLimit;
	else if (z_off < dwRangeNegLimit)
	    z_off = dwRangeNegLimit;

	//storege the cerrunt offset data with DOT format
	offset_data[0] = x_off;
	offset_data[1] = y_off;
	offset_data[2] = z_off;

	//storege the cerrunt Gain data with GOT format
	gain_data[0] = 256*8*128/3/(40+x_gain);
	gain_data[1] = 256*8*128/3/(40+y_gain);
	gain_data[2] = 256*8*128/3/(40+z_gain);
	GSE_LOG("%d %d ======================\n\n ", gain_data[0], x_gain);

	buf[0] = 0x43;
	i2c_smbus_write_byte_data(client, 0x07, buf[0]);

	buf[0] = x_off & 0xff;
	buf[1] = ((x_off >> 8) & bMsbFilter) | (x_gain & 0x0100 ? 0x80 : 0);
	buf[2] = y_off & 0xff;
	buf[3] = ((y_off >> 8) & bMsbFilter) | (y_gain & 0x0100 ? 0x80 : 0);
	buf[4] = z_off & 0xff;
	buf[5] = ((z_off >> 8) & bMsbFilter) | (z_gain & 0x0100 ? 0x80 : 0);

	i2c_smbus_write_i2c_block_data(client, 0x21,   2, &buf[0]);
	i2c_smbus_write_i2c_block_data(client, 0x21+2, 2, &buf[2]);
	i2c_smbus_write_i2c_block_data(client, 0x21+4, 2, &buf[4]);
	
	buf[0] = 0x41;
	i2c_smbus_write_byte_data(client, 0x07,buf[0]);

    msleep(50);

	return err;
}

//=============================================================================
int mcube_read_cali_file(struct i2c_client *client)
{
	int cali_data[3] = { 0 };
	int err = 0;
	char buf[64] = { 0 };

	GSE_LOG("%s %d\n",__func__,__LINE__);

	initKernelEnv();

	fd_file = openFile(file_path, O_RDONLY, 0); 

	if (fd_file == NULL) 
	{
		GSE_LOG("fail to open\n");
		cali_data[0] = 0;
		cali_data[1] = 0;
		cali_data[2] = 0;

		return -1;
	}
	else
	{
		memset(buf, 0, 64); 

		if ((err = readFile(fd_file, buf, 64)) > 0) 
			GSE_LOG("buf:%s\n",buf); 
		else 
			GSE_LOG("read file error %d\n",err); 

		set_fs(oldfs); 
		closeFile(fd_file); 

		sscanf(buf, "%d %d %d", &cali_data[MC3XXX_AXIS_X], &cali_data[MC3XXX_AXIS_Y], &cali_data[MC3XXX_AXIS_Z]);
		GSE_LOG("cali_data: %d %d %d\n", cali_data[MC3XXX_AXIS_X], cali_data[MC3XXX_AXIS_Y], cali_data[MC3XXX_AXIS_Z]); 	
				
		MC3XXX_WriteCalibration(client, cali_data);
	}

	return 0;
}

//=============================================================================
static int mcube_write_log_data(struct i2c_client *client, u8 data[0x3f])
{
	#define _WRT_LOG_DATA_BUFFER_SIZE    (66 * 50)

	s16 rbm_data[3]={0}, raw_data[3]={0};
	int err =0;
	char *_pszBuffer = NULL;
	int n=0,i=0;

	initKernelEnv();
	fd_file = openFile(DATA_PATH ,O_RDWR | O_CREAT,0); 
	if (fd_file == NULL) 
	{
		GSE_LOG("mcube_write_log_data fail to open\n");	
	}
	else
	{
		rbm_data[MC3XXX_AXIS_X] = (s16)((data[0x0d]) | (data[0x0e] << 8));
		rbm_data[MC3XXX_AXIS_Y] = (s16)((data[0x0f]) | (data[0x10] << 8));
		rbm_data[MC3XXX_AXIS_Z] = (s16)((data[0x11]) | (data[0x12] << 8));

		raw_data[MC3XXX_AXIS_X] = (rbm_data[MC3XXX_AXIS_X] + offset_data[0]/2)*gsensor_gain.x/gain_data[0];
		raw_data[MC3XXX_AXIS_Y] = (rbm_data[MC3XXX_AXIS_Y] + offset_data[1]/2)*gsensor_gain.y/gain_data[1];
		raw_data[MC3XXX_AXIS_Z] = (rbm_data[MC3XXX_AXIS_Z] + offset_data[2]/2)*gsensor_gain.z/gain_data[2];

		_pszBuffer = kzalloc(_WRT_LOG_DATA_BUFFER_SIZE, GFP_KERNEL);
		if (NULL == _pszBuffer)
		{
			GSE_ERR("fail to allocate memory for buffer\n");
    		closeFile(fd_file); 
			return -1;
		}
		memset(_pszBuffer, 0, _WRT_LOG_DATA_BUFFER_SIZE); 

		n += sprintf(_pszBuffer+n, "G-sensor RAW X = %d  Y = %d  Z = %d\n", raw_data[0] ,raw_data[1] ,raw_data[2]);
		n += sprintf(_pszBuffer+n, "G-sensor RBM X = %d  Y = %d  Z = %d\n", rbm_data[0] ,rbm_data[1] ,rbm_data[2]);
		for(i=0; i<64; i++)
		{
            n += sprintf(_pszBuffer+n, "mCube register map Register[%x] = 0x%x\n",i,data[i]);
		}
		msleep(50);		
		if ((err = writeFile(fd_file,_pszBuffer,n))>0) 
			GSE_LOG("buf:%s\n",_pszBuffer); 
		else 
			GSE_LOG("write file error %d\n",err); 

		kfree(_pszBuffer);

		set_fs(oldfs); 
		closeFile(fd_file); 
	}
	return 0;
}

//=============================================================================
void MC3XXX_rbm(struct i2c_client *client, int enable)
{
    char    _baDataBuf[3] = { 0 };
    
    _baDataBuf[0] = 0x43; 
    i2c_smbus_write_byte_data(client, 0x07, _baDataBuf[0]);

    _baDataBuf[0] = i2c_smbus_read_byte_data(client, 0x04);

    GSE_LOG("[%s] REG(0x04): 0x%X, enable: %d\n", __FUNCTION__, _baDataBuf[0], enable);
    
    if (0x00 == (_baDataBuf[0] & 0x40))
    {
        _baDataBuf[0] = 0x6D;
        i2c_smbus_write_byte_data(client, 0x1B, _baDataBuf[0]);
        
        _baDataBuf[0] = 0x43;
        i2c_smbus_write_byte_data(client, 0x1B, _baDataBuf[0]);
    }

    GSE_LOG("BEGIN - REG(0x04): 0x%X\n", _baDataBuf[0]);
    
    if (1 == enable)
    {
        _baDataBuf[0] = 0x00;
        i2c_smbus_write_byte_data(client, 0x3B, _baDataBuf[0]);

        _baDataBuf[0] = 0x02; 
        i2c_smbus_write_byte_data(client, 0x14, _baDataBuf[0]);
        
        if (MC3XXX_RESOLUTION_LOW == s_bResolution)
        {
            gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = 1024;
        }

        enable_RBM_calibration = 1;

        GSE_LOG("set rbm!!\n");
    }
    else if (0 == enable)
    {
        _baDataBuf[0] = 0x00; 
        i2c_smbus_write_byte_data(client, 0x14, _baDataBuf[0]);

        _baDataBuf[0] = s_bPCODER; 
        i2c_smbus_write_byte_data(client, 0x3B, _baDataBuf[0]);
        
        mc3xxx_set_gain();

        enable_RBM_calibration = 0;

        GSE_LOG("clear rbm!!\n");
    }
    
    _baDataBuf[0] = i2c_smbus_read_byte_data(client, 0x04);

    GSE_LOG("RBM CONTROL DONE - REG(0x04): 0x%X\n", _baDataBuf[0]);
    
    if (_baDataBuf[0] & 0x40)
    {
        _baDataBuf[0] = 0x6D;
        i2c_smbus_write_byte_data(client, 0x1B, _baDataBuf[0]);
        
        _baDataBuf[0] = 0x43;
        i2c_smbus_write_byte_data(client, 0x1B, _baDataBuf[0]);
    }
    
    GSE_LOG("END - REG(0x04): 0x%X\n", _baDataBuf[0]);
    
    _baDataBuf[0] = 0x41; 
    i2c_smbus_write_byte_data(client, 0x07, _baDataBuf[0]);

    msleep(220);
}

//=============================================================================
int MC3XXX_ReadOffset(struct i2c_client *client,s16 ofs[MC3XXX_AXES_NUM])
{    
	int err = 0;
	u8 off_data[6] = { 0 };

	if(MC3XXX_RESOLUTION_HIGH == s_bResolution)
	{
		err = i2c_smbus_read_i2c_block_data(client, MC3XXX_XOUT_EX_L_REG, MC3XXX_DATA_LEN, off_data);

		ofs[MC3XXX_AXIS_X] = ((s16)(off_data[0]))|((s16)(off_data[1])<<8);
		ofs[MC3XXX_AXIS_Y] = ((s16)(off_data[2]))|((s16)(off_data[3])<<8);
		ofs[MC3XXX_AXIS_Z] = ((s16)(off_data[4]))|((s16)(off_data[5])<<8);
	}
	else if(MC3XXX_RESOLUTION_LOW == s_bResolution)
	{
		err = i2c_smbus_read_i2c_block_data(client, 0, 3, off_data);

		ofs[MC3XXX_AXIS_X] = (s8)off_data[0];
		ofs[MC3XXX_AXIS_Y] = (s8)off_data[1];
		ofs[MC3XXX_AXIS_Z] = (s8)off_data[2];			
	}

    MCUBE_RREMAP(ofs[MC3XXX_AXIS_X], ofs[MC3XXX_AXIS_Y]);

	GSE_LOG("MC3XXX_ReadOffset %d %d %d\n", ofs[MC3XXX_AXIS_X], ofs[MC3XXX_AXIS_Y], ofs[MC3XXX_AXIS_Z]);

    return err;  
}

//=============================================================================
int MC3XXX_ResetCalibration(struct i2c_client *client)
{
	u8 buf[6] = { 0 };
	s16 tmp = 0;
	int err = 0;

    u8  bMsbFilter       = 0x3F;
    s16 wSignBitMask     = 0x2000;
    s16 wSignPaddingBits = 0xC000;

	buf[0] = 0x43;
	err = i2c_smbus_write_byte_data(client, 0x07, buf[0]);
	if(err)
	{
		GSE_ERR("error 0x07: %d\n", err);
	}

	err = i2c_smbus_write_i2c_block_data(client, 0x21, 6, offset_buf);
	if(err)
	{
		GSE_ERR("error: %d\n", err);
	}
	
	buf[0] = 0x41;
	err = i2c_smbus_write_byte_data(client, 0x07, buf[0]);
	if(err)
	{
		GSE_ERR("error: %d\n", err);
	}

	msleep(20);
	
    if (IS_MCFM12() || IS_MCFM3X())
    {
        bMsbFilter       = 0x7F;
        wSignBitMask     = 0x4000;
        wSignPaddingBits = 0x8000;
    }

	tmp = ((offset_buf[1] & bMsbFilter) << 8) + offset_buf[0];
	if (tmp & wSignBitMask)
		tmp |= wSignPaddingBits;
	offset_data[0] = tmp;
					
	tmp = ((offset_buf[3] & bMsbFilter) << 8) + offset_buf[2];
	if (tmp & wSignBitMask)
			tmp |= wSignPaddingBits;
	offset_data[1] = tmp;
					
	tmp = ((offset_buf[5] & bMsbFilter) << 8) + offset_buf[4];
	if (tmp & wSignBitMask)
		tmp |= wSignPaddingBits;
	offset_data[2] = tmp;	

	return 0;
}

//=============================================================================
int MC3XXX_ReadCalibration(struct i2c_client *client, int dat[MC3XXX_AXES_NUM])
{
    signed short MC_offset[MC3XXX_AXES_NUM + 1] = { 0 };    // +1: for 4-byte alignment
    int err = 0;

	memset(MC_offset, 0, sizeof(MC_offset));

	err = MC3XXX_ReadOffset(client, MC_offset);

    if (err)
    {
        GSE_ERR("read offset fail, %d\n", err);
        return err;
    }    
    
    dat[MC3XXX_AXIS_X] = MC_offset[MC3XXX_AXIS_X];
    dat[MC3XXX_AXIS_Y] = MC_offset[MC3XXX_AXIS_Y];
    dat[MC3XXX_AXIS_Z] = MC_offset[MC3XXX_AXIS_Z];  
                                      
    return 0;
}

//=============================================================================
int MC3XXX_ReadData(struct i2c_client *client, s16 buffer[MC3XXX_AXES_NUM])
{
	unsigned char buf[6] = { 0 };
	signed char buf1[6] = { 0 };
	char rbm_buf[6] = { 0 };
	int ret = 0;

        mc3xxx_set_mode(client, MC3XXX_WAKE);

	if (enable_RBM_calibration == 0)
	{
		//err = hwmsen_read_block(client, addr, buf, 0x06);
	}
	else if (enable_RBM_calibration == 1)
	{		
		memset(rbm_buf, 0, 6);
        i2c_smbus_read_i2c_block_data(client, 0x0d  , 2, &rbm_buf[0]);
        i2c_smbus_read_i2c_block_data(client, 0x0d+2, 2, &rbm_buf[2]);
        i2c_smbus_read_i2c_block_data(client, 0x0d+4, 2, &rbm_buf[4]);
	}

	if (enable_RBM_calibration == 0)
	{
		if(MC3XXX_RESOLUTION_HIGH == s_bResolution)
		
		{
			ret = i2c_smbus_read_i2c_block_data(client, MC3XXX_XOUT_EX_L_REG, 6, buf);
			
			buffer[0] = (signed short)((buf[0])|(buf[1]<<8));
			buffer[1] = (signed short)((buf[2])|(buf[3]<<8));
			buffer[2] = (signed short)((buf[4])|(buf[5]<<8));
		}

		else if(MC3XXX_RESOLUTION_LOW == s_bResolution)
		{
			ret = i2c_smbus_read_i2c_block_data(client, MC3XXX_XOUT_REG, 3, buf1);
				
			buffer[0] = (signed short)buf1[0];
			buffer[1] = (signed short)buf1[1];
			buffer[2] = (signed short)buf1[2];
		}
	
		mcprintkreg("MC3XXX_ReadData: %d %d %d\n", buffer[0], buffer[1], buffer[2]);
	}
	else if (enable_RBM_calibration == 1)
	{
		buffer[MC3XXX_AXIS_X] = (s16)((rbm_buf[0]) | (rbm_buf[1] << 8));
		buffer[MC3XXX_AXIS_Y] = (s16)((rbm_buf[2]) | (rbm_buf[3] << 8));
		buffer[MC3XXX_AXIS_Z] = (s16)((rbm_buf[4]) | (rbm_buf[5] << 8));

		GSE_LOG("%s RBM<<<<<[%08d %08d %08d]\n", __func__, buffer[MC3XXX_AXIS_X], buffer[MC3XXX_AXIS_Y], buffer[MC3XXX_AXIS_Z]);

		if(gain_data[0] == 0)
		{
			buffer[MC3XXX_AXIS_X] = 0;
			buffer[MC3XXX_AXIS_Y] = 0;
			buffer[MC3XXX_AXIS_Z] = 0;

			return 0;
		}

		buffer[MC3XXX_AXIS_X] = (buffer[MC3XXX_AXIS_X] + offset_data[0]/2)*gsensor_gain.x/gain_data[0];
		buffer[MC3XXX_AXIS_Y] = (buffer[MC3XXX_AXIS_Y] + offset_data[1]/2)*gsensor_gain.y/gain_data[1];
		buffer[MC3XXX_AXIS_Z] = (buffer[MC3XXX_AXIS_Z] + offset_data[2]/2)*gsensor_gain.z/gain_data[2];
			
		GSE_LOG("%s offset_data <<<<<[%d %d %d]\n", __func__, offset_data[0], offset_data[1], offset_data[2]);
		GSE_LOG("%s gsensor_gain <<<<<[%d %d %d]\n", __func__, gsensor_gain.x, gsensor_gain.y, gsensor_gain.z);
		GSE_LOG("%s gain_data <<<<<[%d %d %d]\n", __func__, gain_data[0], gain_data[1], gain_data[2]);
		GSE_LOG("%s RBM->RAW <<<<<[%d %d %d]\n", __func__, buffer[MC3XXX_AXIS_X], buffer[MC3XXX_AXIS_Y], buffer[MC3XXX_AXIS_Z]);
	}

    MCUBE_RREMAP(buffer[MC3XXX_AXIS_X], buffer[MC3XXX_AXIS_Y]);

	return 0;
}

//=============================================================================
int MC3XXX_ReadRawData(struct i2c_client *client, char * buf)
{
	int res = 0;
	s16 raw_buf[3] = { 0 };

	if (!buf || !client)
	{
		return -EINVAL;
	}
	
	mc3xxx_set_mode(client, MC3XXX_WAKE);
	res = MC3XXX_ReadData(client, &raw_buf[0]);
	if(res)
	{     
		GSE_ERR("I2C error: ret value=%d", res);
		return -EIO;
	}
	else
	{
    	const struct mc3xx0_hwmsen_convert *pCvt = &mc3xx0_cvt[mc3xx0_current_placement];

		GSE_LOG("UPDATE dat: (%+3d %+3d %+3d)\n", raw_buf[MC3XXX_AXIS_X], raw_buf[MC3XXX_AXIS_Y], raw_buf[MC3XXX_AXIS_Z]);

        raw_buf[MC3XXX_AXIS_X] = ((raw_buf[MC3XXX_AXIS_X] * GRAVITY_1G_VALUE) / gsensor_gain.x);
        raw_buf[MC3XXX_AXIS_Y] = ((raw_buf[MC3XXX_AXIS_Y] * GRAVITY_1G_VALUE) / gsensor_gain.y);
        raw_buf[MC3XXX_AXIS_Z] = ((raw_buf[MC3XXX_AXIS_Z] * GRAVITY_1G_VALUE) / gsensor_gain.z);

        G_RAW_DATA[MC3XXX_AXIS_X] = pCvt->sign[MC3XXX_AXIS_X] * raw_buf[pCvt->map[MC3XXX_AXIS_X]];
        G_RAW_DATA[MC3XXX_AXIS_Y] = pCvt->sign[MC3XXX_AXIS_Y] * raw_buf[pCvt->map[MC3XXX_AXIS_Y]];
        G_RAW_DATA[MC3XXX_AXIS_Z] = pCvt->sign[MC3XXX_AXIS_Z] * raw_buf[pCvt->map[MC3XXX_AXIS_Z]];

		sprintf(buf, "%04x %04x %04x", G_RAW_DATA[MC3XXX_AXIS_X], G_RAW_DATA[MC3XXX_AXIS_Y], G_RAW_DATA[MC3XXX_AXIS_Z]);

		GSE_LOG("G_RAW_DATA: (%+3d %+3d %+3d)\n", G_RAW_DATA[MC3XXX_AXIS_X], G_RAW_DATA[MC3XXX_AXIS_Y], G_RAW_DATA[MC3XXX_AXIS_Z]);
	}

	return 0;
}

//=============================================================================
static int MC3XXX_ReadRegMap(struct i2c_client *p_i2c_client, u8 *pbUserBuf)
{
    #define MC3XXX_REGMAP_LENGTH    (64)

    u8     _baData[MC3XXX_REGMAP_LENGTH] = { 0 };
    int    _nIndex = 0;

    GSE_LOG("[%s]\n", __func__);

    if(NULL == p_i2c_client)
        return (-EINVAL);

    for(_nIndex = 0; _nIndex < MC3XXX_REGMAP_LENGTH; _nIndex++)
    {
        _baData[_nIndex] = i2c_smbus_read_byte_data(p_i2c_client, _nIndex);

        if (NULL != pbUserBuf)
            pbUserBuf[_nIndex] = _baData[_nIndex];

        printk(KERN_INFO "[%s] REG[0x%02X] = 0x%02X\n", __FUNCTION__, _nIndex, _baData[_nIndex]);
    }

    mcube_write_log_data(p_i2c_client, _baData);

    return (0);

    #undef MC3XXX_REGMAP_LENGTH
}
#endif

//=============================================================================
void MC3XXX_Reset(struct i2c_client *client) 
{
    unsigned char    _baBuf[2] = { 0 };

	s16 tmp = 0, x_gain = 0, y_gain = 0, z_gain = 0;
	s32 x_off = 0, y_off = 0, z_off = 0;
    u8  bMsbFilter       = 0x3F;
    s16 wSignBitMask     = 0x2000;
    s16 wSignPaddingBits = 0xC000;

    _baBuf[0] = 0x43;
    i2c_smbus_write_byte_data(client, 0x07, _baBuf[0]);
    
    _baBuf[0] = i2c_smbus_read_byte_data(client, 0x04);
    
    if (0x00 == (_baBuf[0] & 0x40))
    {
        _baBuf[0] = 0x6D;
        i2c_smbus_write_byte_data(client, 0x1B, _baBuf[0]);
        
        _baBuf[0] = 0x43;
        i2c_smbus_write_byte_data(client, 0x1B, _baBuf[0]);
    }
    
    _baBuf[0] = 0x43;
    i2c_smbus_write_byte_data(client, 0x07, _baBuf[0]);
    
    _baBuf[0] = 0x80;
    i2c_smbus_write_byte_data(client, 0x1C, _baBuf[0]);

    _baBuf[0] = 0x80;
    i2c_smbus_write_byte_data(client, 0x17, _baBuf[0]);

    msleep(5);
    
    _baBuf[0] = 0x00;
    i2c_smbus_write_byte_data(client, 0x1C, _baBuf[0]);

    _baBuf[0] = 0x00;
    i2c_smbus_write_byte_data(client, 0x17, _baBuf[0]);

    msleep(5);
    
    i2c_smbus_read_i2c_block_data(client, 0x21, 9, offset_buf);

    _baBuf[0] = i2c_smbus_read_byte_data(client, 0x04);

    if (_baBuf[0] & 0x40)
    {
        _baBuf[0] = 0x6D;
        i2c_smbus_write_byte_data(client, 0x1B, _baBuf[0]);
        
        _baBuf[0] = 0x43;
        i2c_smbus_write_byte_data(client, 0x1B, _baBuf[0]);
    }

    if (IS_MCFM12() || IS_MCFM3X())
    {
        bMsbFilter       = 0x7F;
        wSignBitMask     = 0x4000;
        wSignPaddingBits = 0x8000;
    }

	tmp = ((offset_buf[1] & bMsbFilter) << 8) + offset_buf[0];
	if (tmp & wSignBitMask)
		tmp |= wSignPaddingBits;
	x_off = tmp;
					
	tmp = ((offset_buf[3] & bMsbFilter) << 8) + offset_buf[2];
	if (tmp & wSignBitMask)
			tmp |= wSignPaddingBits;
	y_off = tmp;
					
	tmp = ((offset_buf[5] & bMsbFilter) << 8) + offset_buf[4];
	if (tmp & wSignBitMask)
		tmp |= wSignPaddingBits;
	z_off = tmp;	
    	
    // get x,y,z gain
    x_gain = ((offset_buf[1] >> 7) << 8) + offset_buf[6];
    y_gain = ((offset_buf[3] >> 7) << 8) + offset_buf[7];
    z_gain = ((offset_buf[5] >> 7) << 8) + offset_buf[8];
    			
    //storege the cerrunt offset data with DOT format
    offset_data[0] = x_off;
    offset_data[1] = y_off;
    offset_data[2] = z_off;
    
    //storege the cerrunt Gain data with GOT format
    gain_data[0] = 256*8*128/3/(40+x_gain);
    gain_data[1] = 256*8*128/3/(40+y_gain);
    gain_data[2] = 256*8*128/3/(40+z_gain);
    
    GSE_LOG("offser gain = %d %d %d %d %d %d======================\n\n ", gain_data[0], gain_data[1], gain_data[2], offset_data[0], offset_data[1], offset_data[2]);
}

//=============================================================================
static void MC3XXX_SaveDefaultOffset(struct i2c_client *p_i2c_client)
{
    GSE_LOG("[%s]\n", __func__);

    i2c_smbus_read_i2c_block_data(p_i2c_client, 0x21, 3, &s_baOTP_OffsetData[0]);
    i2c_smbus_read_i2c_block_data(p_i2c_client, 0x24, 3, &s_baOTP_OffsetData[3]);

    GSE_LOG("s_baOTP_OffsetData: 0x%02X - 0x%02X - 0x%02X - 0x%02X - 0x%02X - 0x%02X\n",
            s_baOTP_OffsetData[0], s_baOTP_OffsetData[1], s_baOTP_OffsetData[2],
            s_baOTP_OffsetData[3], s_baOTP_OffsetData[4], s_baOTP_OffsetData[5]);
}

//=============================================================================
int mc3xxx_read_accel_xyz(struct i2c_client *client, s16 *acc)
{
    int comres = 0;
    s16 raw_data[MC3XXX_AXES_NUM] = { 0 };
    const struct mc3xx0_hwmsen_convert *pCvt = &mc3xx0_cvt[mc3xx0_current_placement];

    mc3xxx_set_mode(client, MC3XXX_WAKE);

    #ifdef DOT_CALI
        comres = MC3XXX_ReadData(client, &raw_data[0]);
    #else
        unsigned char raw_buf[6] = { 0 };
        signed char raw_buf1[3] = { 0 };
    
        if(MC3XXX_RESOLUTION_HIGH == s_bResolution)
        {
            comres = i2c_smbus_read_i2c_block_data(client, MC3XXX_XOUT_EX_L_REG, 6, raw_buf);
            
            acc[0] = (signed short)((raw_buf[0])|(raw_buf[1]<<8));
            acc[1] = (signed short)((raw_buf[2])|(raw_buf[3]<<8));
            acc[2] = (signed short)((raw_buf[4])|(raw_buf[5]<<8));
        }
        else if(MC3XXX_RESOLUTION_LOW == s_bResolution)
        {
            comres = i2c_smbus_read_i2c_block_data(client, MC3XXX_XOUT_REG, 3, raw_buf1);
            
            acc[0] = (signed short)raw_buf1[0];
            acc[1] = (signed short)raw_buf1[1];
            acc[2] = (signed short)raw_buf1[2];
        }

        raw_data[MC3XXX_AXIS_X] = acc[MC3XXX_AXIS_X];
        raw_data[MC3XXX_AXIS_Y] = acc[MC3XXX_AXIS_Y];
        raw_data[MC3XXX_AXIS_Z] = acc[MC3XXX_AXIS_Z];

        MCUBE_RREMAP(raw_data[MC3XXX_AXIS_X], raw_data[MC3XXX_AXIS_Y]);
    #endif
    
    raw_data[MC3XXX_AXIS_X] = ((raw_data[MC3XXX_AXIS_X] * GRAVITY_1G_VALUE) / gsensor_gain.x);
    raw_data[MC3XXX_AXIS_Y] = ((raw_data[MC3XXX_AXIS_Y] * GRAVITY_1G_VALUE) / gsensor_gain.y);
    raw_data[MC3XXX_AXIS_Z] = ((raw_data[MC3XXX_AXIS_Z] * GRAVITY_1G_VALUE) / gsensor_gain.z);
    
    acc[MC3XXX_AXIS_X] = pCvt->sign[MC3XXX_AXIS_X] * raw_data[pCvt->map[MC3XXX_AXIS_X]];
    acc[MC3XXX_AXIS_Y] = pCvt->sign[MC3XXX_AXIS_Y] * raw_data[pCvt->map[MC3XXX_AXIS_Y]];
    acc[MC3XXX_AXIS_Z] = pCvt->sign[MC3XXX_AXIS_Z] * raw_data[pCvt->map[MC3XXX_AXIS_Z]];

    printk(KERN_ERR"MC3XXX_DataAferMap: %d %d %d\n", acc[MC3XXX_AXIS_X], acc[MC3XXX_AXIS_Y] , acc[MC3XXX_AXIS_Z]);
    
    return comres;
}

//=============================================================================
static int mc3xxx_measure(struct i2c_client *client, struct acceleration *accel)
{
	s16 raw[3] = { 0 };
	
    #ifdef DOT_CALI
        int ret = 0;

        if( load_cali_flg > 0)
        {
            ret = mcube_read_cali_file(client);

            if(ret == 0)
                load_cali_flg = ret;
            else 
                load_cali_flg--;

            GSE_LOG("load_cali %d\n",ret); 
        }  
    #endif

	mc3xxx_read_accel_xyz(client, &raw[0]);

	accel->x = raw[0];
	accel->y = raw[1];
	accel->z = raw[2];
	
	return 0;
}

//=============================================================================
static void mc3xxx_work_func(struct work_struct *work)
{
	struct mc3xxx_data *data = container_of(work, struct mc3xxx_data, work);
	struct acceleration accel = { 0 };

	mc3xxx_measure(data->client, &accel);
	
	input_report_abs(data->input_dev, ABS_X, accel.x);
	input_report_abs(data->input_dev, ABS_Y, accel.y);
	input_report_abs(data->input_dev, ABS_Z, accel.z);
	input_sync(data->input_dev);
}

//=============================================================================
static enum hrtimer_restart mc3xxx_timer_func(struct hrtimer *timer)
{
	struct mc3xxx_data *data = container_of(timer, struct mc3xxx_data, timer);

	queue_work(data->mc3xxx_wq, &data->work);

	hrtimer_start(&data->timer, ktime_set(0, sensor_duration * 1000000), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

//=============================================================================
static int mc3xxx_enable(struct mc3xxx_data *data, int enable)
{
	if(enable)
	{
//		msleep(10);
        mutex_lock(&data->lock);
		mc3xxx_chip_init(data->client);                
        mutex_unlock(&data->lock);
		hrtimer_start(&data->timer, ktime_set(0, sensor_duration * 1000000), HRTIMER_MODE_REL);
		data->enabled = true;
	}
	else
	{
		hrtimer_cancel(&data->timer);
		data->enabled = false;
	}

	return 0;
}

//=============================================================================
static long mc3xxx_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	float convert_para = 0.0f;

    #ifdef DOT_CALI
        void __user *data1 = NULL;
        char strbuf[256] = { 0 };
        int cali[3] = { 0 };
        int temp = 0;
        SENSOR_DATA sensor_data = { 0 };
        struct i2c_client *client = container_of(mc3xxx_device.parent, struct i2c_client, dev);
    #endif

	struct mc3xxx_data *data = NULL;

	data = i2c_get_clientdata(client);

	switch (cmd) {
	case IOCTL_SENSOR_SET_DELAY_ACCEL:
		if(copy_from_user((void *)&sensor_duration, (void __user *) arg, sizeof(short))!=0){
			printk("copy from error in %s.\n",__func__);
		}

		break;

	case IOCTL_SENSOR_GET_DELAY_ACCEL:
		if(copy_to_user((void __user *) arg, (const void *)&sensor_duration, sizeof(short))!=0){
			printk("copy to error in %s.\n",__func__);
		} 

		break;

	case IOCTL_SENSOR_GET_STATE_ACCEL:
		if(copy_to_user((void __user *) arg, (const void *)&sensor_state_flag, sizeof(short))!=0){
			printk("copy to error in %s.\n",__func__);
		}

		break;

	case IOCTL_SENSOR_SET_STATE_ACCEL:
		if(copy_from_user((void *)&sensor_state_flag, (void __user *) arg, sizeof(short))!=0){
			printk("copy from error in %s.\n",__func__);
		}     

		break;
	case IOCTL_SENSOR_GET_NAME:
		if(copy_to_user((void __user *) arg,(const void *)MC3XXX_DISPLAY_NAME, sizeof(MC3XXX_DISPLAY_NAME))!=0){
			printk("copy to error in %s.\n",__func__);
		}     			
		break;		

	case IOCTL_SENSOR_GET_VENDOR:
		if(copy_to_user((void __user *) arg,(const void *)MC3XXX_DIPLAY_VENDOR, sizeof(MC3XXX_DIPLAY_VENDOR))!=0){
			printk("copy to error in %s.\n",__func__);
		}     			
		break;

	case IOCTL_SENSOR_GET_CONVERT_PARA:
		convert_para = MC3XXX_CONVERT_PARAMETER;
		if(copy_to_user((void __user *) arg,(const void *)&convert_para,sizeof(float))!=0){
			printk("copy to error in %s.\n",__func__);
		}     			
        break;
			
	#ifdef DOT_CALI
	case GSENSOR_IOCTL_READ_SENSORDATA:	
	case GSENSOR_IOCTL_READ_RAW_DATA:
	case GSENSOR_MCUBE_IOCTL_READ_RBM_DATA:
		GSE_LOG("fwq GSENSOR_IOCTL_READ_RAW_DATA\n");

        mutex_lock(&data->lock);
		MC3XXX_ReadRawData(client, strbuf);
        mutex_unlock(&data->lock);

		if (copy_to_user((void __user *) arg, &strbuf, strlen(strbuf)+1)) {
			printk("failed to copy sense data to user space.");
			return -EFAULT;
		}
		break;

	case GSENSOR_MCUBE_IOCTL_SET_CALI:
		GSE_LOG("fwq GSENSOR_MCUBE_IOCTL_SET_CALI!!\n");
		data1 = (void __user *)arg;

		if(data1 == NULL)
		{
			ret = -EINVAL;
			break;	  
		}
		if(copy_from_user(&sensor_data, data1, sizeof(sensor_data)))
		{
			ret = -EFAULT;
			break;	  
		}
		else
		{
			cali[MC3XXX_AXIS_X] = sensor_data.x;
			cali[MC3XXX_AXIS_Y] = sensor_data.y;
			cali[MC3XXX_AXIS_Z] = sensor_data.z;	

			GSE_LOG("GSENSOR_MCUBE_IOCTL_SET_CALI %d  %d  %d  %d  %d  %d!!\n", cali[MC3XXX_AXIS_X], cali[MC3XXX_AXIS_Y],cali[MC3XXX_AXIS_Z] ,sensor_data.x, sensor_data.y ,sensor_data.z);
				
            mutex_lock(&data->lock);
			ret = MC3XXX_WriteCalibration(client, cali);			 
            mutex_unlock(&data->lock);
		}
		break;
		
	case GSENSOR_IOCTL_CLR_CALI:
		GSE_LOG("fwq GSENSOR_IOCTL_CLR_CALI!!\n");
        mutex_lock(&data->lock);
		ret = MC3XXX_ResetCalibration(client);
        mutex_unlock(&data->lock);
		break;

	case GSENSOR_IOCTL_GET_CALI:
		GSE_LOG("fwq mc3xxx GSENSOR_IOCTL_GET_CALI\n");
			
		data1 = (unsigned char*)arg;
		
		if(data1 == NULL)
		{
			ret = -EINVAL;
			break;	  
		}

        mutex_lock(&data->lock);
		if((ret = MC3XXX_ReadCalibration(client, cali)))
		{
            mutex_unlock(&data->lock);
			GSE_LOG("fwq mc3xxx MC3XXX_ReadCalibration error!!!!\n");
			break;
		}
        mutex_unlock(&data->lock);

		sensor_data.x = cali[MC3XXX_AXIS_X];
		sensor_data.y = cali[MC3XXX_AXIS_Y];
		sensor_data.z = cali[MC3XXX_AXIS_Z];

		if(copy_to_user(data1, &sensor_data, sizeof(sensor_data)))
		{
			ret = -EFAULT;
			break;
		}		
		break;	

	case GSENSOR_IOCTL_SET_CALI_MODE:
		GSE_LOG("fwq mc3xxx GSENSOR_IOCTL_SET_CALI_MODE\n");
		break;

	case GSENSOR_MCUBE_IOCTL_SET_RBM_MODE:
		GSE_LOG("fwq GSENSOR_MCUBE_IOCTL_SET_RBM_MODE\n");
        mutex_lock(&data->lock);
		MC3XXX_rbm(client, 1);
        mutex_unlock(&data->lock);
		break;

	case GSENSOR_MCUBE_IOCTL_CLEAR_RBM_MODE:
		GSE_LOG("fwq GSENSOR_MCUBE_IOCTL_CLEAR_RBM_MODE\n");
        mutex_lock(&data->lock);
		MC3XXX_rbm(client, 0);
        mutex_unlock(&data->lock);
		break;

	case GSENSOR_MCUBE_IOCTL_REGISTER_MAP:
		GSE_LOG("fwq GSENSOR_MCUBE_IOCTL_REGISTER_MAP\n");
        mutex_lock(&data->lock);
        MC3XXX_ReadRegMap(client, NULL);
        mutex_unlock(&data->lock);
		break;

    case GSENSOR_MCUBE_IOCTL_READ_PRODUCT_ID:
        GSE_LOG("fwq GSENSOR_MCUBE_IOCTL_READ_PRODUCT_ID\n");
        data1 = (void __user *) arg;
        if(data1 == NULL)
        {
            ret = -EINVAL;
            break;	  
        }
        
        if (MC3XXX_RETCODE_SUCCESS == mc3xxx_validate_sensor_IC(&s_bPCODE, &s_bHWID))
            temp = true;
        else
            temp = false;
        
        if(copy_to_user(data1, &temp, sizeof(temp)))
        {
            GSE_LOG("%s: read pcode fail to copy!\n", __func__);
            return -EFAULT;
        }
        break;

	case GSENSOR_MCUBE_IOCTL_READ_FILEPATH:
		GSE_LOG("fwq GSENSOR_MCUBE_IOCTL_READ_FILEPATH\n");
		data1 = (void __user *) arg;
		if(data1 == NULL)
		{
			ret = -EINVAL;
			break;	  
		}
		
		if(copy_to_user(data1, file_path, (strlen(file_path)+1)))
		{
			ret = -EFAULT;
			break;
		}				 
		break;		
#endif
		
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

//=============================================================================
static int mc3xxx_open(struct inode *inode, struct file *filp)
{
	return nonseekable_open(inode, filp);
}

//=============================================================================
static int mc3xxx_release(struct inode *inode, struct file *filp)
{
	return 0;
}

//=============================================================================
static struct file_operations sensor_fops =
{
    .owner          = THIS_MODULE,
    .open       	= mc3xxx_open,
    .release    	= mc3xxx_release,
    .unlocked_ioctl = mc3xxx_ioctl,
};

#if 0 //#ifdef CONFIG_HAS_EARLYSUSPEND
//=============================================================================
static void mc3xxx_early_suspend(struct early_suspend *handler)
{
	struct mc3xxx_data *data = NULL;

	data = container_of(handler, struct mc3xxx_data, early_suspend);

	hrtimer_cancel(&data->timer);

    mutex_lock(&data->lock);
	mc3xxx_set_mode(data->client, MC3XXX_STANDBY);
    mutex_unlock(&data->lock);
}

//=============================================================================
static void mc3xxx_early_resume(struct early_suspend *handler)
{
	struct mc3xxx_data *data = NULL;

	data = container_of(handler, struct mc3xxx_data, early_suspend);
	
    mutex_lock(&data->lock);
    mc3xxx_chip_init(data->client); 
    MC3XXX_ResetCalibration(data->client); 
    mcube_read_cali_file(data->client); 
    mc3xxx_set_mode(data->client, MC3XXX_WAKE);
    mutex_unlock(&data->lock);

	hrtimer_start(&data->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
}
#else
static int mc3xxx_acc_resume(struct device *dev)
{
    struct mc3xxx_data *data = dev_get_drvdata(dev);

    hrtimer_cancel(&data->timer);

    mutex_lock(&data->lock);
    mc3xxx_set_mode(data->client, MC3XXX_STANDBY);
    mutex_unlock(&data->lock);
    return 0;
}

static int mc3xxx_acc_suspend(struct device *dev)
{
    struct mc3xxx_data *data = dev_get_drvdata(dev);

    hrtimer_cancel(&data->timer);

    mutex_lock(&data->lock);
    mc3xxx_set_mode(data->client, MC3XXX_STANDBY);
    mutex_unlock(&data->lock);
    return 0;
}
#endif

//=============================================================================
static struct miscdevice mc3xxx_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name  = SENSOR_NAME,
    .fops  = &sensor_fops,
};

/**
 * gsensor_fetch_sysconfig_para - get config info from sysconfig.fex file.
 * return value:  
 *                    = 0; success;
 *                    < 0; err
 */
//=============================================================================
static int gsensor_fetch_sysconfig_para(void)
{
	u_i2c_addr.dirty_addr_buf[0] = 0x4c;
	u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;

	return 0;
}

/*****************************************
 *** _mc3xxx_i2c_auto_probe
 *****************************************/
static int mc3xxx_i2c_auto_probe(struct i2c_client *client)
{
    #define _MC3XXX_I2C_PROBE_ADDR_COUNT_    (sizeof(mc3xxx_i2c_auto_probe_addr) / sizeof(mc3xxx_i2c_auto_probe_addr[0]))

    unsigned char    _baData1Buf[2] = { 0 };
    unsigned char    _baData2Buf[2] = { 0 };

    int              _nCount = 0;
    int              _naCheckCount[_MC3XXX_I2C_PROBE_ADDR_COUNT_] = { 0 };

    memset(_naCheckCount, 0, sizeof(_naCheckCount));

_I2C_AUTO_PROBE_RECHECK_:
    s_bPCODE  = 0x00;
    s_bPCODER = 0x00;
    s_bHWID   = 0x00;

    for (_nCount = 0; _nCount < _MC3XXX_I2C_PROBE_ADDR_COUNT_; _nCount++)
    {
        client->addr = mc3xxx_i2c_auto_probe_addr[_nCount];

        GSE_LOG("[%s][%d] probing addr: 0x%X\n", __FUNCTION__, _nCount, client->addr);

        _baData1Buf[0] = 0x3B;
        if (0 > i2c_master_send(client, &(_baData1Buf[0]), 1))
        {
            GSE_ERR("ERR: addr: 0x%X fail to communicate-2!\n", client->addr);
            continue;
        }
    
        if (0 > i2c_master_recv(client, &(_baData1Buf[0]), 1))
        {
            GSE_ERR("ERR: addr: 0x%X fail to communicate-3!\n", client->addr);
            continue;
        }

        _naCheckCount[_nCount]++;

        GSE_LOG("[%s][%d] addr: 0x%X ok to read REG(0x3B): 0x%X\n", __FUNCTION__, _nCount, client->addr, _baData1Buf[0]);

        if (0x00 == _baData1Buf[0])
        {
            if (1 == _naCheckCount[_nCount])
            {
                MC3XXX_Reset(client);
                msleep(3);
                goto _I2C_AUTO_PROBE_RECHECK_;
            }
            else
            {
                continue;
            }
        }

        _baData2Buf[0] = 0x18;
        i2c_master_send(client, &(_baData2Buf[0]), 1);
        i2c_master_recv(client, &(_baData2Buf[0]), 1);

        s_bPCODER = _baData1Buf[0];

        if (MC3XXX_RETCODE_SUCCESS == mc3xxx_validate_sensor_IC(&_baData1Buf[0], &_baData2Buf[0]))
        {
            s_bPCODE = _baData1Buf[0];
            s_bHWID  = _baData2Buf[0];

            MC3XXX_SaveDefaultOffset(client);

            GSE_LOG("[%s] addr: 0x%X confirmed ok to use. s_bPCODE: 0x%02X, s_bHWID: 0x%02X\n", __FUNCTION__, client->addr, s_bPCODE, s_bHWID);

            return (MC3XXX_RETCODE_SUCCESS);
        }
    }

    return (MC3XXX_RETCODE_ERROR_I2C);

    #undef _MC3XXX_I2C_PROBE_ADDR_COUNT_
}


static int mc3xxx_acc_poll_delay_set(struct sensors_classdev *sensors_cdev,
	unsigned int delay_msec)
{
       sensor_duration = delay_msec;
       
	if (sensor_duration > SENSOR_DURATION_MAX)
		sensor_duration = SENSOR_DURATION_MAX;
	if (sensor_duration < SENSOR_DURATION_MIN)
		sensor_duration = SENSOR_DURATION_MIN;
    
	return 0;
}

static int mc3xxx_acc_enable_set(struct sensors_classdev *sensors_cdev,
	unsigned int enable)
{
	struct mc3xxx_data *acc = container_of(sensors_cdev,
		struct mc3xxx_data, cdev);
	int err;

	if (enable)
		err = mc3xxx_enable(acc, 1);
	else
		err = mc3xxx_enable(acc, 0);
	return err;
}

//=============================================================================
static int mc3xxx_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	struct mc3xxx_data *data = NULL;
	printk("%s mc3xxx probe start... \n", __func__);
    if (MC3XXX_RETCODE_SUCCESS != mc3xxx_i2c_auto_probe(client))
    {
        GSE_ERR("ERR: fail to probe mCube sensor!\n");
        goto exit;
    }

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	GSE_LOG("[%s] confirmed i2c addr: 0x%X\n", __FUNCTION__, client->addr);
	
    #ifdef DOT_CALI
        load_cali_flg = 30;
    #endif

	data = kzalloc(sizeof(struct mc3xxx_data), GFP_KERNEL);
	if(data == NULL)
	{
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	data->mc3xxx_wq = create_singlethread_workqueue("mc3xxx_wq");
	if (!data->mc3xxx_wq)
	{
		ret = -ENOMEM;
		goto err_create_workqueue_failed;
	}
	INIT_WORK(&data->work, mc3xxx_work_func);
	mutex_init(&data->lock);

	sensor_duration = SENSOR_DURATION_DEFAULT;
	sensor_state_flag = 1;

	data->client = client;
	dev.client = client;

	i2c_set_clientdata(client, data);	

	data->input_dev = input_allocate_device();
	if (!data->input_dev) {
		ret = -ENOMEM;
		goto exit_input_dev_alloc_failed;
	}

    #ifdef DOT_CALI
        MC3XXX_Reset(client);
    #endif

	ret = mc3xxx_chip_init(client);
	if (ret < 0) {
		goto err_chip_init_failed;
	}

	set_bit(EV_ABS, data->input_dev->evbit);
	data->map[0] = G_0;
	data->map[1] = G_1;
	data->map[2] = G_2;
	data->inv[0] = G_0_REVERSE;
	data->inv[1] = G_1_REVERSE;
	data->inv[2] = G_2_REVERSE;

	input_set_abs_params(data->input_dev, ABS_X, -32*8, 32*8, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(data->input_dev, ABS_Y, -32*8, 32*8, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(data->input_dev, ABS_Z, -32*8, 32*8, INPUT_FUZZ, INPUT_FLAT);

	data->input_dev->name = ACCEL_INPUT_DEV_NAME;

	ret = input_register_device(data->input_dev);
	if (ret) {
		goto exit_input_register_device_failed;
	}
	
    mc3xxx_device.parent = &client->dev;
 
	ret = misc_register(&mc3xxx_device);
	if (ret) {
		goto exit_misc_device_register_failed;
	}

	ret = sysfs_create_group(&data->input_dev->dev.kobj, &mc3xxx_group);

	if (!data->use_irq){
		hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		data->timer.function = mc3xxx_timer_func;
		hrtimer_start(&data->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

	data->cdev = mc3xxx_acc_cdev;
	data->cdev.sensors_enable = mc3xxx_acc_enable_set;
	data->cdev.sensors_poll_delay = mc3xxx_acc_poll_delay_set;
	ret = sensors_classdev_register(&client->dev, &data->cdev);
	if (ret) {
		dev_err(&client->dev,
			"class device create failed: %d\n", ret);
		goto exit_remove_sysfs_int;
	}

#if 0 //#ifdef CONFIG_HAS_EARLYSUSPEND
  data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = mc3xxx_early_suspend;
	data->early_suspend.resume = mc3xxx_early_resume;
	register_early_suspend(&data->early_suspend);
#endif

	data->enabled = 1;
	printk(KERN_ERR"%s mc3xxx probe ok \n", __func__);

	return 0;

exit_remove_sysfs_int:
	sysfs_remove_group(&data->input_dev->dev.kobj, &mc3xxx_group);
exit_misc_device_register_failed:
	input_unregister_device(data->input_dev);	
exit_input_register_device_failed:
	input_free_device(data->input_dev);
err_chip_init_failed:
exit_input_dev_alloc_failed:
	destroy_workqueue(data->mc3xxx_wq);	
err_create_workqueue_failed:
	kfree(data);	
err_alloc_data_failed:
err_check_functionality_failed:
exit:
	printk("mc3xxx probe failed \n");
	return ret;
}

//=============================================================================
static int mc3xxx_remove(struct i2c_client *client)
{
	struct mc3xxx_data *data = i2c_get_clientdata(client);

	hrtimer_cancel(&data->timer);
	input_unregister_device(data->input_dev);	
	misc_deregister(&mc3xxx_device);
	sysfs_remove_group(&data->input_dev->dev.kobj, &mc3xxx_group);
	kfree(data);
	return 0;
}

//=============================================================================
static void mc3xxx_shutdown(struct i2c_client *client)
{
	struct mc3xxx_data *data = i2c_get_clientdata(client);
/*Modify in the absence of the device reboot will lead to an null pointer by ke.li@tcl.com 20150922 */
	if(data != NULL){
	   if(data->enabled){
		mc3xxx_enable(data, 0);
		}
	}
}

//=============================================================================
static const struct i2c_device_id mc3xxx_id[] =
{
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, mc3xxx_id);

static struct of_device_id mc3xxx_acc_match_table[] = {
	{ .compatible = "Mcube,mc3xxx", },
	{ },
};
static const struct dev_pm_ops mc3xxx_pm_ops = {
	.suspend	= mc3xxx_acc_suspend,
	.resume 	= mc3xxx_acc_resume,
};

static struct i2c_driver mc3xxx_driver =
{
    .class = I2C_CLASS_HWMON,

    .driver = {
                  .owner = THIS_MODULE,
                  .name	 = SENSOR_NAME,
        	    .of_match_table = mc3xxx_acc_match_table,
        	    .pm = &mc3xxx_pm_ops,
              },

    .id_table	  = mc3xxx_id,
    .probe		  = mc3xxx_probe,
    .remove		  = mc3xxx_remove,
    .shutdown	  = mc3xxx_shutdown,
    .address_list = u_i2c_addr.normal_i2c,
};

//=============================================================================
static int __init mc3xxx_init(void)
{
	int ret = -1;

	printk("mc3xxx: init\n");

	if (gsensor_fetch_sysconfig_para())
	{
		printk("%s: err.\n", __func__);
		return -1;
	}

	GSE_LOG("%s: after fetch_sysconfig_para:  normal_i2c: 0x%hx. normal_i2c[1]: 0x%hx \n", __func__, u_i2c_addr.normal_i2c[0], u_i2c_addr.normal_i2c[1]);

	ret = i2c_add_driver(&mc3xxx_driver);

	return ret;
}

//=============================================================================
static void __exit mc3xxx_exit(void)
{
	i2c_del_driver(&mc3xxx_driver);
}

//=============================================================================
module_init(mc3xxx_init);
module_exit(mc3xxx_exit);

MODULE_DESCRIPTION("mc3xxx accelerometer driver");
MODULE_AUTHOR("mCube-inc");
MODULE_LICENSE("GPL");
MODULE_VERSION(SENSOR_DRIVER_VERSION);

