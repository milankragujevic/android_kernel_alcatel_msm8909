/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
//Begin add by zhaohong.chen@tcl.com for camera sensor node
#include "camera_tct_func.h"
//End add
#define GC0310_SENSOR_NAME "gc0310_sunrise"
#define GC0310_PLATFORM_NAME "qcom,gc0310_sunrise"

/*#define CONFIG_MSMB_CAMERA_DEBUG*/
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

//Begin add by zhaohong.chen@tcl.com for yuv camera dynamic debug
extern char dynamic_regs_debug;
//End add

DEFINE_MSM_MUTEX(gc0310_mut);
static struct msm_sensor_ctrl_t gc0310_s_ctrl;

static struct msm_sensor_power_setting gc0310_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct msm_sensor_power_setting gc0310_power_down_setting[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct msm_camera_i2c_reg_conf gc0310_start_settings[] = {
	{0xfe, 0x03},
	{0x10, 0x90},//94 ppp
	{0xfe, 0x00},
};

static struct msm_camera_i2c_reg_conf gc0310_stop_settings[] = {
	{0xfe, 0x03},
	{0x10, 0x80},
	{0xfe, 0x00},
};

static struct msm_camera_i2c_reg_conf gc0310_recommend_settings[] = {
	{0xfe,0xf0},
	{0xfe,0xf0},
	{0xfe,0x00},
	{0xfc,0x0e},
	{0xfc,0x0e},
	{0xf2,0x80},
	{0xf3,0x00}, //
	{0xf7,0x1b},
	{0xf8,0x04},
	{0xf9,0x8e},
	{0xfa,0x11},
	
	/////////////////////////////////////////////////
	/////////////////	CISCTL reg	///////////////////
	/////////////////////////////////////////////////   
                                            
	{0xfe,0x00},
	{0x00,0x2f},
	{0x01,0x0f},//06
	{0x02,0x04},
	{0x03,0x03},
	{0x04,0x50},
	{0x09,0x00},
	{0x0a,0x00},
	{0x0b,0x00},
	{0x0c,0x04},
	{0x0d,0x01},
	{0x0e,0xe8},
	{0x0f,0x02},
	{0x10,0x88},
	{0x16,0x00},	//add 140422
	{0x17,0x14}, //0x17
	{0x18,0x1a},
	{0x19,0x14},
	{0x1b,0x48},
	{0x1e,0x6b},
	{0x1f,0x28},
	{0x20,0x8b},//89 travis20140801
	{0x21,0x49},
	{0x22,0xb0},
	{0x23,0x04},
	{0x24,0x16},
	{0x34,0x20},
	
	/////////////////////////////////////////////////
	////////////////////   BLK	 ////////////////////
	/////////////////////////////////////////////////
	{0x26,0x23},
	{0x28,0xff},
	{0x29,0x00},
	{0x33,0x10}, 
	{0x37,0x20},
	{0x38,0x10},
	{0x47,0x80},
	{0x4e,0x66},
	{0xa8,0x02},
	{0xa9,0x80},
	
	/////////////////////////////////////////////////
	//////////////////	ISP reg   ///////////////////
	/////////////////////////////////////////////////
	{0xfe,0x00},
	{0x40,0xff},
	{0x41,0x21},
	{0x42,0xcf},
	{0x44,0x02},
	{0x45,0xa8}, // 0xaf},	decrease first preview frame delay.
	{0x46,0x02}, //sync
	{0x4a,0x11},
	{0x4b,0x01},
	{0x4c,0x20},
	{0x4d,0x05},
	{0x4f,0x01},
	{0x50,0x01},
	{0x55,0x01},
	{0x56,0xe0},
	{0x57,0x02},
	{0x58,0x80},
	
	/////////////////////////////////////////////////
	///////////////////   GAIN   ////////////////////
	/////////////////////////////////////////////////
	{0xfe,0x00},
	{0x70,0x70},
	{0x5a,0x84},
	{0x5b,0xc9},
	{0x5c,0xed},
	{0x77,0x74},
	{0x78,0x40},
	{0x79,0x5f},
	
	///////////////////////////////////////////////// 
		///////////////////   DNDD	/////////////////////
		///////////////////////////////////////////////// 
		{0x82,0x08}, //1f
		{0x83,0x04},//1f
		{0x89,0xf0},
		
		///////////////////////////////////////////////// 
		//////////////////	 EEINTP  ////////////////////
		///////////////////////////////////////////////// 
		{0x8f,0xaa}, 
		{0x90,0x8c}, 
		{0x91,0x90},
		{0x92,0x03}, 
		{0x93,0x03}, 
		{0x94,0x03}, 
		{0x95,0x88}, //63
		{0x96,0xf0}, 
		
		///////////////////////////////////////////////// 
		/////////////////////  ASDE  ////////////////////
		///////////////////////////////////////////////// 
		{0xfe,0x00},
	
		{0x9a,0x20},
	    {0x9b,0x80},
		{0x9c,0x40},
	     {0x9d,0x80},
	     {0xa1,0x30},
      	{0xa2,0x32},
		{0xa4,0x30},
		{0xa5,0x30},
		{0xaa,0x10}, //10
	    {0xac,0x22},
		
		/////////////////////////////////////////////////
		///////////////////   GAMMA   ///////////////////
		/////////////////////////////////////////////////
		{0xfe,0x00},//default
		{0xbf , 0x0b},
		{0xc0 , 0x17},
		{0xc1 , 0x2a},
		{0xc2 , 0x41},
		{0xc3 , 0x54},
		{0xc4 , 0x66},
		{0xc5 , 0x74},
		{0xc6 , 0x8c},
		{0xc7 , 0xa3},
		{0xc8 , 0xb5},
		{0xc9 , 0xc4},
		{0xca , 0xd0},
		{0xcb , 0xdb},
		{0xcc , 0xe5},
		{0xcd , 0xf0},
		{0xce , 0xf7},
		{0xcf , 0xff},
		/////////////////////////////////////////////////
		///////////////////   YCP  //////////////////////
		/////////////////////////////////////////////////
		{0xd0,0x3d},//40
		{0xd1,0x30}, //34  dick//0x32 pad
		{0xd2,0x30}, //34  dick//0x32  pad
		{0xd3,0x44},//40  pad
		{0xd5,0x00},//	3c 
		{0xd6,0xf2},
		{0xd7,0x1b},
		{0xd8,0x18},
		{0xdd,0x03}, 
									
		/////////////////////////////////////////////////
		////////////////////   AEC	 ////////////////////
		/////////////////////////////////////////////////
		{0xfe,0x01},
		{0x05,0x30},
		{0x06,0x75},
		{0x07,0x40},
		{0x08,0xb0},
		{0x0a,0xc5},
		{0x0b,0x11},
		{0x0c,0x30},//00
		{0x12,0x52},
		{0x13,0x38},
		{0x18,0x95},
		{0x19,0x96},
		{0x1f,0x20},
		{0x20,0xc0}, 
		{0x3e,0x40}, 
		{0x3f,0x57}, 
		{0x40,0x7d}, 
		{0x03,0x60}, 
		{0x44,0x02}, 
									
		/////////////////////////////////////////////////
		////////////////////   AWB	 ////////////////////
		/////////////////////////////////////////////////
		{0xfe, 0x01},
		{0x84, 0x86},
		{0x1c, 0x91},
		{0x21, 0x15},
		{0x50, 0x80},
		{0x56, 0x06},
		{0x59, 0x08},
		{0x5b, 0x02},
		{0x61, 0x8d},
		{0x62, 0xa7},
		{0x63, 0xb0},
		{0x65, 0x0a},
		{0x66, 0x06},
		{0x67, 0x84},
		{0x69, 0x08},
		{0x6a, 0x25},
		{0x6b, 0x01},
		{0x6c, 0x00},
		{0x6d, 0x02},
		{0x6e, 0xf0},//30
		{0x6f, 0x80},
		{0x76, 0x7a},
		{0x78, 0xff},
		{0x79, 0x75},
		{0x7a, 0x40},
		{0x7b, 0x68},
		{0x7c, 0x00},
		{0x90, 0xc9},
		{0x91, 0xbe},
		{0x92, 0xe6},
		{0x93, 0xca},
		{0x95, 0x23},
		{0x96, 0xe7},
		{0x97, 0x52},
		{0x98, 0x24},
		{0x9a, 0x52},
		{0x9b, 0x24},
		{0x9c, 0x94},
		{0x9d, 0x52},
		{0x9f, 0xc7},
		{0xa0, 0xc8},
		{0xa1, 0x00},
		{0xa2, 0x00},
		{0x86, 0x00},
		{0x87, 0x00},
		{0x88, 0x00},
		{0x89, 0x00},
		{0xa4, 0xb9},
		{0xa5, 0xa0},
		{0xa6, 0xb8},
		{0xa7, 0x95},
		{0xa9, 0xb6},
		{0xaa, 0x89},
		{0xab, 0x9d},
		{0xac, 0x80},
		{0xae, 0xb3},
		{0xaf, 0x9e},
		{0xb0, 0xbe},
		{0xb1, 0x97},
		{0xb3, 0xb7},
		{0xb4, 0x7f},
		{0xb5, 0x00},
		{0xb6, 0x00},
		{0x8b, 0x00},
		{0x8c, 0x00},
		{0x8d, 0x00},
		{0x8e, 0x00},
		{0x94, 0x55},
		{0x99, 0xa6},
		{0x9e, 0xaa},
		{0xa3, 0x0a},
		{0x8a, 0x00},
		{0xa8, 0x55},
		{0xad, 0x55},
		{0xb2, 0x55},
		{0xb7, 0x05},
		{0x8f, 0x00},
		{0xb8, 0xcb},
		{0xb9, 0x9b},
		/////////////////////////////////////////////////
		////////////////////  CC ////////////////////////
		/////////////////////////////////////////////////

		{0xfe, 0x01},
		{0xd0, 0x38},
		{0xd1, 0x00},
		{0xd2, 0x02},
		{0xd3, 0x04},
		{0xd4, 0x38},
		{0xd5, 0x12},

		{0xd6, 0x30},
		{0xd7, 0x00},
		{0xd8, 0x0a},
		{0xd9, 0x16},
		{0xda, 0x39},
		{0xdb, 0xf8}, 
										   
		/////////////////////////////////////////////////
		////////////////////   LSC	 ////////////////////
		/////////////////////////////////////////////////
		{0xfe,0x01}, 
		{0xc1,0x3c}, 
		{0xc2,0x50}, 
		{0xc3,0x00}, 
		{0xc4,0x88}, 
		{0xc5,0x58}, 
		{0xc6,0x58}, 
		{0xc7,0x40}, 
		{0xc8,0x20}, 
		{0xc9,0x20}, 
		{0xdc,0x20}, 
		{0xdd,0x10}, 
		{0xdf,0x00}, 
		{0xde,0x00}, 
									
		/////////////////////////////////////////////////
		///////////////////  Histogram	/////////////////
		/////////////////////////////////////////////////
		{0x01,0x10}, 
		{0x0b,0x31}, 
		{0x0e,0x50}, 
		{0x0f,0x0f}, 
		{0x10,0x6e}, 
		{0x12,0xa0}, 
		{0x15,0x60}, 
		{0x16,0x60}, 
		{0x17,0xe0}, 
									
		/////////////////////////////////////////////////
		//////////////	 Measure Window   ///////////////
		/////////////////////////////////////////////////
		{0xcc,0x0c}, 
		{0xcd,0x10}, 
		{0xce,0xa0}, 
		{0xcf,0xe6}, 
									
		/////////////////////////////////////////////////
		/////////////////	dark sun   //////////////////
		/////////////////////////////////////////////////
		{0x45,0xf7},
		{0x46,0xff}, 
		{0x47,0x15},
		{0x48,0x03}, 
		{0x4f,0x60}, 
									
		/////////////////////////////////////////////////
		///////////////////  banding  ///////////////////
		/////////////////////////////////////////////////
		{0xfe,0x00},
		{0x05,0x01},
		{0x06,0x18}, //HB
		{0x07,0x00},
		{0x08,0x10}, //VB
		{0xfe,0x01},
		{0x25,0x00}, //step 
		{0x26,0x9a}, 
		{0x27,0x01}, //30fps
		{0x28,0xce},  
		{0x29,0x04}, //20fps
		{0x2a,0xd0}, 
		{0x2b,0x08}, //10fps
		{0x2c,0x6c}, 
		{0x2d,0x0b}, //8fps
		{0x2e,0x6e},
		{0x3c,0x20},
		{0xfe,0x00},
	/////////////////////////////////////////////////
	///////////////////   MIPI	 ////////////////////
	/////////////////////////////////////////////////
		{0xfe,0x03},
		{0x40,0x08},
		{0x42,0x00},
		{0x43,0x00},
		{0x01,0x03},
		{0x10,0x84},
		{0x01,0x03},             
		{0x02,0x00},             
		{0x03,0x94},             
		{0x04,0x01},            
		{0x05,0x00},             
		{0x06,0x80},             
		{0x11,0x1e},             
		{0x12,0x00},      
		{0x13,0x05},             
		{0x15,0x10},                                                                    
		{0x21,0x10},             
		{0x22,0x01},             
		{0x23,0x10},                                             
		{0x24,0x02},                                             
		{0x25,0x10},                                             
		{0x26,0x03},                                             
		{0x29,0x02},                                             
		{0x2a,0x0a},                                             
		{0x2b,0x04}, 
 
};

static struct msm_camera_i2c_reg_conf gc0310_reg_saturation[11][3] = {
	{
		{0xfe,0x00},{0xd1,0x10},{0xd2,0x10},
	},
	{
		{0xfe,0x00},{0xd1,0x18},{0xd2,0x18},
	},
	{
		{0xfe,0x00},{0xd1,0x20},{0xd2,0x20},
	},
	{
		{0xfe,0x00},{0xd1,0x28},{0xd2,0x28},
	},
	{
		{0xfe,0x00},{0xd1,0x2c},{0xd2,0x2c},
	},
	{
		{0xfe,0x00},{0xd1,0x28},{0xd2,0x28},  
	},
	{
		{0xfe,0x00},{0xd1,0x3a},{0xd2,0x3a},
	},
	{
		{0xfe,0x00},{0xd1,0x40},{0xd2,0x40},
	},
	{
		{0xfe,0x00},{0xd1,0x48},{0xd2,0x48},
	},
	{
		{0xfe,0x00},{0xd1,0x50},{0xd2,0x50},
	},
	{
		{0xfe,0x00},{0xd1,0x58},{0xd2,0x58},
	},
};

static struct msm_camera_i2c_reg_conf gc0310_reg_contrast[11][3] = {
	{
		{0xfe, 0x00},{0xd3, 0x1d},{0xfe, 0x00},
	},
	{
		{0xfe, 0x00},{0xd3, 0x20},{0xfe, 0x00},
	},
	{
		{0xfe, 0x00},{0xd3, 0x24},{0xfe, 0x00},
	},
	{
		{0xfe, 0x00},{0xd3, 0x30},{0xfe, 0x00},
	},
	{
		{0xfe, 0x00},{0xd3, 0x3d},{0xfe, 0x00},
	},
	{
		{0xfe, 0x00},{0xd3, 0x44},{0xfe, 0x00},  
	},
	{
		{0xfe, 0x00},{0xd3, 0x54},{0xfe, 0x00},
	},
	{
		{0xfe, 0x00},{0xd3, 0x5d},{0xfe, 0x00},
	},
	{
		{0xfe, 0x00},{0xd3, 0x64},{0xfe, 0x00},
	},
	{
		{0xfe, 0x00},{0xd3, 0x6d},{0xfe, 0x00},
	},
	{
		{0xfe, 0x00},{0xd3, 0x74},{0xfe, 0x00},
	},
};

static struct msm_camera_i2c_reg_conf gc0310_reg_sharpness[6][3] = {
	{{0xfe, 0x00},{0x95, 0x33},{0xfe, 0x00}},//Sharpness -2
	{{0xfe, 0x00},{0x95, 0x44},{0xfe, 0x00}},//Sharpness -1
	{{0xfe, 0x00},{0x95, 0x65},{0xfe, 0x00}},//Sharpness
	{{0xfe, 0x00},{0x95, 0x77},{0xfe, 0x00}},//Sharpness +1
	{{0xfe, 0x00},{0x95, 0x88},{0xfe, 0x00}},//Sharpness +2
	{{0xfe, 0x00},{0x95, 0x99},{0xfe, 0x00}},//Sharpness +3
};

static struct msm_camera_i2c_reg_conf gc0310_reg_iso[7][2] = {
//not supported
	/* auto */
	{
		{0xfe, 0x00},
		{0xfe, 0x00},
	},
	/* auto hjt */  
	{
		{0xfe, 0x00},
		{0xfe, 0x00},
	},
	/* iso 100 */
	{
		{0xfe, 0x00},
		{0xfe, 0x00},
	},
	/* iso 200 */
	{
		{0xfe, 0x00},
		{0xfe, 0x00},
	},
	/* iso 400 */
	{
		{0xfe, 0x00},
		{0xfe, 0x00},
	},
	/* iso 800 */
	{
		{0xfe, 0x00},
		{0xfe, 0x00},
	},
	/* iso 1600 */
	{
		{0xfe, 0x00},
		{0xfe, 0x00},
	},
};

static struct msm_camera_i2c_reg_conf gc0310_reg_exposure_compensation[5][3] = {
	{{0xfe, 0x01},{0x13, 0x18},{0xfe, 0x00}},//Exposure -2
	{{0xfe, 0x01},{0x13, 0x28},{0xfe, 0x00}},//Exposure -1
	{{0xfe, 0x01},{0x13, 0x38},{0xfe, 0x00}},//Exposure  0x68
	{{0xfe, 0x01},{0x13, 0x48},{0xfe, 0x00}},//Exposure +1
	{{0xfe, 0x01},{0x13, 0x58},{0xfe, 0x00}},//Exposure +2
};

static struct msm_camera_i2c_reg_conf gc0310_reg_antibanding[4][17] = {
	/* OFF */  //60-1  50-2   auto-off NC
	{
	    {0xfe,0x00},
		{0x05,0x01},
		{0x06,0x18}, //HB
		{0x07,0x00},
		{0x08,0x10}, //VB
		{0xfe,0x01},
		{0x25,0x00}, //step 
		{0x26,0x9a}, 
		{0x27,0x01}, //30fps
		{0x28,0xce},  
		{0x29,0x04}, //20fps
		{0x2a,0xd0}, 
		{0x2b,0x08}, //10fps
		{0x2c,0x6c}, 
		{0x2d,0x0b}, //8fps
		{0x2e,0x6e},
		{0xfe,0x00},
	}, /*ANTIBANDING 60HZ*/
	
	/* 60Hz */
	{
		{0xfe, 0x00}, 
		{0x05, 0x01}, 	
		{0x06, 0x13}, 
		{0x07, 0x00},
		{0x08, 0x10},
		{0xfe, 0x01},
		{0x25, 0x00},   //anti-flicker step [11:8]
		{0x26, 0x81},   //anti-flicker step [7:0]
		{0x27, 0x02},   //exp level 0  30fps
		{0x28, 0x04}, 
		{0x29, 0x03},   //exp level 1  20fps
		{0x2a, 0x06}, 
		{0x2b, 0x05},   //exp level 2  10fps
		{0x2c, 0x8b}, 
		{0x2d, 0x07},   //exp level 3  5.55fps
		{0x2e, 0x0e}, 	
		{0xfe, 0x00},	
	}, /*ANTIBANDING 50HZ*/

	/* 50Hz */
	{
		{0xfe,0x00},
		{0x05,0x01},
		{0x06,0x18}, //HB
		{0x07,0x00},
		{0x08,0x10}, //VB
		{0xfe,0x01},
		{0x25,0x00}, //step 
		{0x26,0x9a}, 
		{0x27,0x01}, //30fps
		{0x28,0xce},  
		{0x29,0x04}, //20fps
		{0x2a,0xd0}, 
		{0x2b,0x08}, //10fps
		{0x2c,0x6c}, 
		{0x2d,0x0b}, //8fps
		{0x2e,0x6e},
		{0xfe,0x00},

	}, /*ANTIBANDING 60HZ*/
	
	/* AUTO */
	{
		{0xfe,0x00},
		{0x05,0x01},
		{0x06,0x18}, //HB
		{0x07,0x00},
		{0x08,0x10}, //VB
		{0xfe,0x01},
		{0x25,0x00}, //step 
		{0x26,0x9a}, 
		{0x27,0x01}, //30fps
		{0x28,0xce},  
		{0x29,0x04}, //20fps
		{0x2a,0xd0}, 
		{0x2b,0x08}, //10fps
		{0x2c,0x6c}, 
		{0x2d,0x0b}, //8fps
		{0x2e,0x6e},
		{0xfe,0x00},

	},/*ANTIBANDING 50HZ*/
};

//begin effect
static struct msm_camera_i2c_reg_conf gc0310_reg_effect_normal[] = {
	/* normal: */
	{0x43, 0x00},//0xe0
};

static struct msm_camera_i2c_reg_conf gc0310_reg_effect_black_white[] = {
	/* B&W: */
	{0x43, 0x02},
	{0xda, 0x00},
	{0xdb, 0x00},
};

static struct msm_camera_i2c_reg_conf gc0310_reg_effect_negative[] = {
	/* Negative: */
	{0x43, 0x01},
};

static struct msm_camera_i2c_reg_conf gc0310_reg_effect_old_movie[] = {
	/* Sepia(antique): */
	{0x43, 0x02},
	{0xda, 0xd0},
	{0xdb, 0x28},
};

static struct msm_camera_i2c_reg_conf gc0310_reg_effect_solarize[] = {
	{0x43, 0x02},
	{0xda, 0xc0},
	{0xdb, 0xc0},
};
// end effect

//begin scene, not realised
static struct msm_camera_i2c_reg_conf gc0310_reg_scene_auto[] = {
	/* <SCENE_auto> */
	{0x43, 0x00},//0xe0
};

static struct msm_camera_i2c_reg_conf gc0310_reg_scene_portrait[] = {
	/* <CAMTUNING_SCENE_PORTRAIT> */
	{0x43, 0x00},//0xe0
};

static struct msm_camera_i2c_reg_conf gc0310_reg_scene_landscape[] = {
	/* <CAMTUNING_SCENE_LANDSCAPE> */
	{0x43, 0x00},//0xe0
};

static struct msm_camera_i2c_reg_conf gc0310_reg_scene_night[] = {
	/* <SCENE_NIGHT> */
	{0x43, 0x00},//0xe0
};
//end scene

//begin white balance
static struct msm_camera_i2c_reg_conf gc0310_reg_wb_auto[] = {
	/* Auto: */
{0x42, 0xfe},{0xfe, 0x00},{0xfe, 0x00},{0xfe, 0x00},
};

static struct msm_camera_i2c_reg_conf gc0310_reg_wb_sunny[] = {
	/* Sunny: */
{0x42, 0xfd},{0x77, 0x74},{0x78, 0x52},{0x79, 0x40},
};

static struct msm_camera_i2c_reg_conf gc0310_reg_wb_cloudy[] = {
	/* Cloudy: */
{0x42, 0xfd},{0x77, 0x8c},{0x78, 0x50},{0x79, 0x40},
};

static struct msm_camera_i2c_reg_conf gc0310_reg_wb_office[] = {
	/* Office: */
{0x42, 0xfd},{0x77, 0x48},{0x78, 0x40},{0x79, 0x5c},
};

static struct msm_camera_i2c_reg_conf gc0310_reg_wb_home[] = {
	/* Home: */
{0x42, 0xfd},{0x77, 0x40},{0x78, 0x54},{0x79, 0x70},
};
//end white balance

static struct v4l2_subdev_info gc0310_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id gc0310_i2c_id[] = {
	{GC0310_SENSOR_NAME, (kernel_ulong_t)&gc0310_s_ctrl},
	{ }
};

static int32_t msm_gc0310_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	pr_err("%s, enter\n", __func__);
	return msm_sensor_i2c_probe(client, id, &gc0310_s_ctrl);
}

static struct i2c_driver gc0310_i2c_driver = {
	.id_table = gc0310_i2c_id,
	.probe  = msm_gc0310_i2c_probe,
	.driver = {
		.name = GC0310_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client gc0310_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static const struct of_device_id gc0310_dt_match[] = {
	{.compatible = GC0310_PLATFORM_NAME, .data = &gc0310_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, gc0310_dt_match);

static int32_t gc0310_platform_probe(struct platform_device *pdev)
{
	int32_t rc;
	const struct of_device_id *match;
	match = of_match_device(gc0310_dt_match, &pdev->dev);
	if (!match)
		return -EFAULT;
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static struct platform_driver gc0310_platform_driver = {
	.driver = {
		.name = GC0310_PLATFORM_NAME,
		.owner = THIS_MODULE,
		.of_match_table = gc0310_dt_match,
	},
	.probe = gc0310_platform_probe,
};

static int __init gc0310_init_module(void)
{
	int32_t rc = 0;
	pr_err("%s:%d\n", __func__, __LINE__);
	rc = i2c_add_driver(&gc0310_i2c_driver);
	if (!rc)
		return rc;
	pr_err("%s:%d rc \n", __func__, __LINE__);
	return platform_driver_register(&gc0310_platform_driver);
}

static void __exit gc0310_exit_module(void)
{
	pr_err("%s:%d\n", __func__, __LINE__);
	if (gc0310_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&gc0310_s_ctrl);
		platform_driver_unregister(&gc0310_platform_driver);
	} else
		i2c_del_driver(&gc0310_i2c_driver);
	return;
}

static void gc0310_i2c_write_table(struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_camera_i2c_reg_conf *table,
		int num)
{
	int i = 0;
	int rc = 0;
	
	for (i = 0; i < num; ++i) 
	{
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client, table->reg_addr,
			table->reg_data, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0) 
		{
			msleep(100);
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				s_ctrl->sensor_i2c_client, table->reg_addr,
				table->reg_data, MSM_CAMERA_I2C_BYTE_DATA);
		}
		table++;
	}
}

static void gc0310_set_saturation(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	gc0310_i2c_write_table(s_ctrl, &gc0310_reg_saturation[value][0],
		ARRAY_SIZE(gc0310_reg_saturation[value]));		
}

static void gc0310_set_contrast(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	gc0310_i2c_write_table(s_ctrl, &gc0310_reg_contrast[value][0],
		ARRAY_SIZE(gc0310_reg_contrast[value]));
}

static void gc0310_set_sharpness(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	int val = value / 6;
	
	gc0310_i2c_write_table(s_ctrl, &gc0310_reg_sharpness[val][0],
		ARRAY_SIZE(gc0310_reg_sharpness[val]));
}

static void gc0310_set_iso(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	gc0310_i2c_write_table(s_ctrl, &gc0310_reg_iso[value][0],
		ARRAY_SIZE(gc0310_reg_iso[value]));
}

static void gc0310_set_exposure_compensation(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	int val = (value + 12) / 6;
	
	gc0310_i2c_write_table(s_ctrl, &gc0310_reg_exposure_compensation[val][0],
		ARRAY_SIZE(gc0310_reg_exposure_compensation[val]));	   
}

static void gc0310_set_effect(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	switch (value) {
	case MSM_CAMERA_EFFECT_MODE_OFF: {
		gc0310_i2c_write_table(s_ctrl, &gc0310_reg_effect_normal[0],
			ARRAY_SIZE(gc0310_reg_effect_normal));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_MONO: {
		gc0310_i2c_write_table(s_ctrl, &gc0310_reg_effect_black_white[0],
			ARRAY_SIZE(gc0310_reg_effect_black_white));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_NEGATIVE: {
		gc0310_i2c_write_table(s_ctrl, &gc0310_reg_effect_negative[0],
			ARRAY_SIZE(gc0310_reg_effect_negative));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SEPIA: {
		gc0310_i2c_write_table(s_ctrl, &gc0310_reg_effect_old_movie[0],
			ARRAY_SIZE(gc0310_reg_effect_old_movie));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SOLARIZE: {
		gc0310_i2c_write_table(s_ctrl, &gc0310_reg_effect_solarize[0],
			ARRAY_SIZE(gc0310_reg_effect_solarize));
		break;
	}
	default:
		gc0310_i2c_write_table(s_ctrl, &gc0310_reg_effect_normal[0],
			ARRAY_SIZE(gc0310_reg_effect_normal));
		break;
	}
}

static void gc0310_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	gc0310_i2c_write_table(s_ctrl, &gc0310_reg_antibanding[value][0],
		ARRAY_SIZE(gc0310_reg_antibanding[value]));
}

static void gc0310_set_scene_mode(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	switch (value) {
	case MSM_CAMERA_SCENE_MODE_OFF: {
		gc0310_i2c_write_table(s_ctrl, &gc0310_reg_scene_auto[0],
			ARRAY_SIZE(gc0310_reg_scene_auto));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_NIGHT: {
		gc0310_i2c_write_table(s_ctrl, &gc0310_reg_scene_night[0],
			ARRAY_SIZE(gc0310_reg_scene_night));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_LANDSCAPE: {
		gc0310_i2c_write_table(s_ctrl, &gc0310_reg_scene_landscape[0],
			ARRAY_SIZE(gc0310_reg_scene_landscape));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_PORTRAIT: {
		gc0310_i2c_write_table(s_ctrl, &gc0310_reg_scene_portrait[0],
			ARRAY_SIZE(gc0310_reg_scene_portrait));
		break;
	}
	default:
		gc0310_i2c_write_table(s_ctrl, &gc0310_reg_scene_auto[0],
			ARRAY_SIZE(gc0310_reg_scene_auto));
		break;
	}
}

static void gc0310_set_white_balance_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	switch (value) {
	case MSM_CAMERA_WB_MODE_AUTO: {
		gc0310_i2c_write_table(s_ctrl, &gc0310_reg_wb_auto[0],
			ARRAY_SIZE(gc0310_reg_wb_auto));
		break;
	}
	case MSM_CAMERA_WB_MODE_INCANDESCENT: {
		gc0310_i2c_write_table(s_ctrl, &gc0310_reg_wb_home[0],
			ARRAY_SIZE(gc0310_reg_wb_home));
		break;
	}
	case MSM_CAMERA_WB_MODE_DAYLIGHT: {
		gc0310_i2c_write_table(s_ctrl, &gc0310_reg_wb_sunny[0],
			ARRAY_SIZE(gc0310_reg_wb_sunny));
		break;
	}
	case MSM_CAMERA_WB_MODE_FLUORESCENT: {
		gc0310_i2c_write_table(s_ctrl, &gc0310_reg_wb_office[0],
			ARRAY_SIZE(gc0310_reg_wb_office));
		break;
	}
	case MSM_CAMERA_WB_MODE_CLOUDY_DAYLIGHT: {
		gc0310_i2c_write_table(s_ctrl, &gc0310_reg_wb_cloudy[0],
			ARRAY_SIZE(gc0310_reg_wb_cloudy));
		break;
	}
	default:
		gc0310_i2c_write_table(s_ctrl, &gc0310_reg_wb_auto[0],
			ARRAY_SIZE(gc0310_reg_wb_auto));
		break;
	}
}

static int32_t gc0310_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	int32_t i = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++)
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
		cdata->cfg.sensor_info.is_mount_angle_valid =
			s_ctrl->sensordata->sensor_info->is_mount_angle_valid;
		cdata->cfg.sensor_info.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++)
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);
		CDBG("%s:%d mount angle valid %d value %d\n", __func__,
			__LINE__, cdata->cfg.sensor_info.is_mount_angle_valid,
			cdata->cfg.sensor_info.sensor_mount_angle);

		break;
	case CFG_SET_INIT_SETTING:
		/* 1. Write Recommend settings */
		/* 2. Write change settings */
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client, gc0310_recommend_settings,
			ARRAY_SIZE(gc0310_recommend_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
			
		//Begin add by zhaohong.chen@tcl.com for yuv camera dynamic debug
		//pr_err("%s:%d rc=%ld",__func__, __LINE__,rc);
		if(dynamic_regs_debug)
			rc = camera_dynamic_regs_debug(s_ctrl);
		//End add
		break;

	case CFG_SET_RESOLUTION: {
#if 0
	/*copy from user the desired resoltuion*/
		enum msm_sensor_resolution_t res = MSM_SENSOR_INVALID_RES;
		if (copy_from_user(&res, (void *)cdata->cfg.setting,
			sizeof(enum msm_sensor_resolution_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		pr_err("%s:%d  res =%d\n", __func__, __LINE__, res);

		if (res == MSM_SENSOR_RES_FULL) {
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_conf_tbl(
				s_ctrl->sensor_i2c_client, gc0310_recommend_settings,
				ARRAY_SIZE(gc0310_recommend_settings),
				MSM_CAMERA_I2C_BYTE_DATA);
				pr_err("%s:%d res =%d\n gc0310_recommend_settings ",
				__func__, __LINE__, res);
		} else {
			pr_err("%s:%d failed resoultion set\n", __func__,
				__LINE__);
			rc = -EFAULT;
		}
#endif
	}
		break;
	case CFG_SET_STOP_STREAM:
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client, gc0310_stop_settings,
			ARRAY_SIZE(gc0310_stop_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
		break;
	case CFG_SET_START_STREAM:
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client, gc0310_start_settings,
			ARRAY_SIZE(gc0310_start_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		cdata->cfg.sensor_init_params.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_init_params.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		pr_err("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;
	case CFG_SET_SLAVE_INFO: {
		struct msm_camera_sensor_slave_info *sensor_slave_info;
		struct msm_camera_power_ctrl_t *p_ctrl;
		uint16_t size;
		int slave_index = 0;
		sensor_slave_info = kmalloc(sizeof(struct msm_camera_sensor_slave_info)
				      * 1, GFP_KERNEL);

		if (!sensor_slave_info) {
			pr_err("%s: failed to alloc mem\n", __func__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(sensor_slave_info,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_sensor_slave_info))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		/* Update sensor slave address */
		if (sensor_slave_info->slave_addr)
			s_ctrl->sensor_i2c_client->cci_client->sid =
				sensor_slave_info->slave_addr >> 1;

		/* Update sensor address type */
		s_ctrl->sensor_i2c_client->addr_type =
			sensor_slave_info->addr_type;

		/* Update power up / down sequence */
		p_ctrl = &s_ctrl->sensordata->power_info;
		size = sensor_slave_info->power_setting_array.size;
		if (p_ctrl->power_setting_size < size) {
			struct msm_sensor_power_setting *tmp;
			tmp = kmalloc(sizeof(struct msm_sensor_power_setting)
				      * size, GFP_KERNEL);
			if (!tmp) {
				pr_err("%s: failed to alloc mem\n", __func__);
				rc = -ENOMEM;
				break;
			}
			kfree(p_ctrl->power_setting);
			p_ctrl->power_setting = tmp;
		}
		p_ctrl->power_setting_size = size;

		rc = copy_from_user(p_ctrl->power_setting, (void *)
			sensor_slave_info->power_setting_array.power_setting,
			size * sizeof(struct msm_sensor_power_setting));
		if (rc) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		for (slave_index = 0; slave_index <
			p_ctrl->power_setting_size; slave_index++) {
			CDBG("%s i %d power setting %d %d %ld %d\n", __func__,
				slave_index,
				p_ctrl->power_setting[slave_index].seq_type,
				p_ctrl->power_setting[slave_index].seq_val,
				p_ctrl->power_setting[slave_index].config_val,
				p_ctrl->power_setting[slave_index].delay);
		}
		break;
	}
	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		if (!conf_array.size ||
			conf_array.size > I2C_REG_DATA_MAX) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &conf_array);
		kfree(reg_setting);
		break;
	}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		if (!conf_array.size ||
			conf_array.size > I2C_SEQ_REG_DATA_MAX) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_POWER_UP:
		if (s_ctrl->func_tbl->sensor_power_up)
			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_POWER_DOWN:
		if (s_ctrl->func_tbl->sensor_power_down)
			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		if (copy_from_user(stop_setting, (void *)cdata->cfg.setting,
		    sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = stop_setting->reg_setting;
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
		    (void *)reg_setting, stop_setting->size *
		    sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
		}
	case CFG_SET_STREAM_TYPE: {
		enum msm_camera_stream_type_t stream_type = MSM_CAMERA_STREAM_INVALID;
		if (copy_from_user(&stream_type, (void *)cdata->cfg.setting,
			sizeof(enum msm_camera_stream_type_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		s_ctrl->camera_stream_type = stream_type;
		break;
	}
	case CFG_SET_SATURATION: 
	{
		int32_t sat_lev;
		//Begin add by zhaohong.chen@tcl.com for yuv camera dynamic debug
		if(dynamic_regs_debug)
		{
			pr_err("%s,%d,dynamic_regs_debug = %d\n",__func__,__LINE__,dynamic_regs_debug);
			break;
		}
		//End add
		if (copy_from_user(&sat_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		gc0310_set_saturation(s_ctrl, sat_lev);
	}
		break;
	case CFG_SET_CONTRAST: 
	{
		int32_t con_lev;
		//Begin add by zhaohong.chen@tcl.com for yuv camera dynamic debug
		if(dynamic_regs_debug)
		{
			pr_err("%s,%d,dynamic_regs_debug = %d\n",__func__,__LINE__,dynamic_regs_debug);
			break;
		}
		//End add
		if (copy_from_user(&con_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		gc0310_set_contrast(s_ctrl, con_lev);
	}
		break;
	case CFG_SET_SHARPNESS: 
	{
		int32_t shp_lev;
		//Begin add by zhaohong.chen@tcl.com for yuv camera dynamic debug
		if(dynamic_regs_debug)
		{
			pr_err("%s,%d,dynamic_regs_debug = %d\n",__func__,__LINE__,dynamic_regs_debug);
			break;
		}
		//End add
		if (copy_from_user(&shp_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		gc0310_set_sharpness(s_ctrl, shp_lev);
	}
		break;
	case CFG_SET_ISO: 
	{
		int32_t iso_lev;
		//Begin add by zhaohong.chen@tcl.com for yuv camera dynamic debug
		if(dynamic_regs_debug)
		{
			pr_err("%s,%d,dynamic_regs_debug = %d\n",__func__,__LINE__,dynamic_regs_debug);
			break;
		}
		//End add
		if (copy_from_user(&iso_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		gc0310_set_iso(s_ctrl, iso_lev);
	}
		break;
	case CFG_SET_EXPOSURE_COMPENSATION: 
	{
		int32_t ec_lev;
		//Begin add by zhaohong.chen@tcl.com for yuv camera dynamic debug
		if(dynamic_regs_debug)
		{
			pr_err("%s,%d,dynamic_regs_debug = %d\n",__func__,__LINE__,dynamic_regs_debug);
			break;
		}
		//End add
		if (copy_from_user(&ec_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		gc0310_set_exposure_compensation(s_ctrl, ec_lev);
	}
		break;
	case CFG_SET_EFFECT: 
	{
		int32_t effect_mode;
		if (copy_from_user(&effect_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		gc0310_set_effect(s_ctrl, effect_mode);
	}
		break;
	case CFG_SET_ANTIBANDING: 
	{
		int32_t antibanding_mode;
		if (copy_from_user(&antibanding_mode,
			(void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		gc0310_set_antibanding(s_ctrl, antibanding_mode);
	}
		break;
	case CFG_SET_BESTSHOT_MODE: 
	{
		int32_t bs_mode;
		if (copy_from_user(&bs_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		gc0310_set_scene_mode(s_ctrl, bs_mode);
	}
		break;
	case CFG_SET_WHITE_BALANCE: 
	{
		int32_t wb_mode;
		//Begin add by zhaohong.chen@tcl.com for yuv camera dynamic debug
		if(dynamic_regs_debug)
		{
			pr_err("%s,%d,dynamic_regs_debug = %d\n",__func__,__LINE__,dynamic_regs_debug);
			break;
		}
		//End add
		if (copy_from_user(&wb_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		gc0310_set_white_balance_mode(s_ctrl, wb_mode);
	}
		break;
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

static struct msm_sensor_fn_t gc0310_sensor_func_tbl = {
	.sensor_config = gc0310_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
};

static struct msm_sensor_ctrl_t gc0310_s_ctrl = {
	.sensor_i2c_client = &gc0310_sensor_i2c_client,
	.power_setting_array.power_setting = gc0310_power_setting,
	.power_setting_array.size = ARRAY_SIZE(gc0310_power_setting),
	.power_setting_array.power_down_setting = gc0310_power_down_setting,
	.power_setting_array.size_down = ARRAY_SIZE(gc0310_power_down_setting),
	.msm_sensor_mutex = &gc0310_mut,
	.sensor_v4l2_subdev_info = gc0310_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(gc0310_subdev_info),
	.func_tbl = &gc0310_sensor_func_tbl,
};

module_init(gc0310_init_module);
module_exit(gc0310_exit_module);
MODULE_DESCRIPTION("GC0310 VGA YUV sensor driver");
MODULE_LICENSE("GPL v2");

