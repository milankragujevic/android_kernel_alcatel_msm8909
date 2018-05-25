/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
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

#define SP0A20_SENSOR_NAME "sp0a20_sunrise"
#define SP0A20_PLATFORM_NAME "qcom,sp0a20_sunrise"

//#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define FILE_INIT_REG_DEBUG

//heq
#define SP0A20_P1_0x10		0x8a		 //ku_outdoor
#define SP0A20_P1_0x11		0x8a		//ku_nr
#define SP0A20_P1_0x12		0x8a		 //ku_dummy
#define SP0A20_P1_0x13		0x8a		 //ku_low  
#define SP0A20_P1_0x14		0x8e		//c4 //kl_outdoor 
#define SP0A20_P1_0x15		0x8c		//c4 //kl_nr      
#define SP0A20_P1_0x16		0x8a		//c4 //kl_dummy    
#define SP0A20_P1_0x17		0x88		//c4 //kl_low   

//sharpness  
#define SP0A20_P2_0xe8      0x24 //0x10//10//;sharp_fac_pos_outdoor
#define SP0A20_P2_0xec      0x24 //0x20//20//;sharp_fac_neg_outdoor

#define SP0A20_P2_0xe9      0x34 //0x0a//0a//;sharp_fac_pos_nr
#define SP0A20_P2_0xed      0x3a //0x20//20//;sharp_fac_neg_nr

#define SP0A20_P2_0xea      0x24 //0x08//08//;sharp_fac_pos_dummy
#define SP0A20_P2_0xee      0x24 //0x18//18//;sharp_fac_neg_dummy

#define SP0A20_P2_0xeb      0x20 //0x08//08//;sharp_fac_pos_low
#define SP0A20_P2_0xef      0x24 //0x08//18//;sharp_fac_neg_low

//saturation
//#define SP0A20_P1_0xd3		0x66
//#define SP0A20_P1_0xd4		0x6a
//#define SP0A20_P1_0xd5  	0x56
//#define SP0A20_P1_0xd6  	0x44
//
//#define SP0A20_P1_0xd7		0x66
//#define SP0A20_P1_0xd8		0x6a
//#define SP0A20_P1_0xd9   	0x56
//#define SP0A20_P1_0xda   	0x44

DEFINE_MSM_MUTEX(sp0a20_mut);
static struct msm_sensor_ctrl_t sp0a20_s_ctrl;

static struct msm_sensor_power_setting sp0a20_power_setting[] = {
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

static struct msm_sensor_power_setting sp0a20_power_down_setting[] = {
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

static struct msm_camera_i2c_reg_conf sp0a20_start_settings[] = {
	{0xfd, 0x00},
	{0x92, 0x71},//01 mipi stream on
	{0xfd, 0x01},
	{0x36, 0x00},	
	{0xfd, 0x00},
};

static struct msm_camera_i2c_reg_conf sp0a20_stop_settings[] = {
	{0xfd, 0x00},
	{0x92, 0x70},
	{0xfd, 0x01},
	{0x36, 0x02},
	{0xfd, 0x00},
	{0xe7, 0x03},
	{0xe7, 0x00},
	{0xfd, 0x00},
};

static struct msm_camera_i2c_reg_conf sp0a20_recommend_settings[] = {
    {0xfd,0x01},
	{0x36,0x02},
	{0xfd,0x00},
	{0x92,0x70},//before LP11 keep LP00
	{0x0c,0x00},//mipi ldo power on
	{0x12,0x02},
	{0x13,0x2f},
	{0x6d,0x32},
	{0x6c,0x32},
	{0x6f,0x33},
	{0x6e,0x34},
	{0x99,0x04},
	{0x16,0x38},
	{0x17,0x38},
	{0x70,0x3a},
	{0x14,0x02},
	{0x15,0x20},
	{0x71,0x23},
	{0x69,0x25},
	{0x6a,0x1a},
	{0x72,0x1c},
	{0x75,0x1e},
	{0x73,0x3c},
	{0x74,0x21},
	{0x79,0x00},
	{0x77,0x10},
	{0x1a,0x4d},
	{0x1b,0x27},
	{0x1c,0x07},
	{0x1e,0x15},
	{0x21,0x0e},
	{0x22,0x28},
	{0x26,0x66},
	{0x28,0x0b},
	{0x37,0x5a},
	//pre_gain
	{0xfd,0x02},
	{0x01,0x80},
	{0x50,0xb0},//
    {0x51,0x08},//
    {0x52,0x03},//
    {0x53,0x03},//
    {0x54,0x00},//
    {0x56,0x03},//
	{0xfd,0x01},//blacklevel
	{0x41,0x00},
	{0x42,0x00},
	{0x43,0x00},
	{0x44,0x00},
	//ae setting 50HZ 24M 8-15fps
	{0xfd,0x00},
	{0x03,0x01},
	{0x04,0xc2},
	{0x05,0x00},
	{0x06,0x00},
	{0x07,0x00},
	{0x08,0x00},
	{0x09,0x02},
	{0x0a,0xf4},
	{0xfd,0x01},
	{0xf0,0x00},
	{0xf7,0x4b},
	{0x02,0x0c},
	{0x03,0x01},
	{0x06,0x4b},
	{0x07,0x00},
	{0x08,0x01},
	{0x09,0x00},
	{0xfd,0x02},
	{0xbe,0x84},
	{0xbf,0x03},
	{0xd0,0x84},
	{0xd1,0x03},

	//ae gain &status
	{0xfd,0x01},
	{0x5a,0x40},//dp rpc   
	{0xfd,0x02},
	{0xbc,0x70},//rpc_heq_low
	{0xbd,0x50},//rpc_heq_dummy
	{0xb8,0x66},//mean_normal_dummy
	{0xb9,0x88},//mean_dummy_normal
	{0xba,0x30},//mean_dummy_low
	{0xbb,0x45},//mean low_dummy

	//rpc
	{0xfd,0x01},//rpc                   
	{0xe0,0x44},//0x4c;rpc_1base_max
	{0xe1,0x36},//0x3c;rpc_2base_max
	{0xe2,0x30},//0x34;rpc_3base_max
	{0xe3,0x2a},//0x2e;rpc_4base_max
	{0xe4,0x2a},//0x2e;rpc_5base_max
	{0xe5,0x28},//0x2c;rpc_6base_max
	{0xe6,0x28},//0x2c;rpc_7base_max
	{0xe7,0x26},//0x2a;rpc_8base_max
	{0xe8,0x26},//0x2a;rpc_9base_max
	{0xe9,0x26},//0x2a;rpc_10base_max
	{0xea,0x24},//0x28;rpc_11base_max
	{0xf3,0x24},//0x28;rpc_12base_max
	{0xf4,0x24},//0x28;rpc_13base_max
	//ae min gain  
	{0xfd,0x01},
	{0x04,0xa0},//rpc_max_indr
	{0x05,0x24},//rpc_min_indr 
	{0x0a,0xa0},//rpc_max_outdr
	{0x0b,0x24},//rpc_min_outdr

	//target
	{0xfd,0x01},
	{0xeb,0x76},//target indr
	{0xec,0x76},//target outdr
	{0xed,0x04},//lock range
	{0xee,0x08},//hold range



	{0xfd,0x01},
	{0xf2,0x4d},
	{0xfd,0x02},
	{0x5b,0x05},//dp status
	{0x5c,0xa0},

	//lens shading
	{0xfd,0x01},
	{0x26,0x80},
	{0x27,0x4f},
	{0x28,0x00},
	{0x29,0x20},
	{0x2a,0x00},
	{0x2b,0x03},
	{0x2c,0x00},
	{0x2d,0x20},
	{0x30,0x00},
	{0x31,0x00},

	//lsc 1
	{0xfd,0x01},//lsc 1
	{0xa1,0x1d},//2d;2c;r
	{0xa2,0x1d},//2f;2c;
	{0xa3,0x1b},//2d;2c;
	{0xa4,0x1b},//2d;2c;
	{0xa5,0x13},//23;2c;g
	{0xa6,0x13},//21;2c;
	{0xa7,0x13},//26;2c;
	{0xa8,0x13},//26;2c;
	{0xa9,0x0d},//21;2c;b
	{0xaa,0x0d},//21;2c;
	{0xab,0x0d},//26;2c;
	{0xac,0x0d},//26;2c;
	{0xad,0x00},//10;12;r
	{0xae,0x00},//10;12;
	{0xaf,0x00},//0e;12;
	{0xb0,0x00},//0e;12;
	{0xb1,0x00},//00;00;g
	{0xb2,0x00},//00;00;
	{0xb3,0x00},//00;00;
	{0xb4,0x00},//00;00;
	{0xb5,0x00},//00;00;b
	{0xb6,0x00},//00;00;
	{0xb7,0x00},//00;00;
	{0xb8,0x00},//00;00;
	//skin detect
	{0xfd,0x02},//skin detect
	{0x08,0x00},
	{0x09,0x06},
	{0x1d,0x03},
	{0x1f,0x05},
	//awb
	{0xfd,0x01},//awb
	{0x32,0x00},
	{0xfd,0x02},
	{0x26,0xbf},
	{0x27,0xa3},
	{0x10,0x02},
	{0x11,0x00},
	{0x1b,0x80},
	{0x1a,0x80},
	{0x18,0x27},
	{0x19,0x26},
	{0x2a,0x00},
	{0x2b,0x00},
	{0x28,0xf8},
	{0x29,0x08},

	//d65 10
	{0x66,0x45},//d65 10
	{0x67,0x65},
	{0x68,0xdc},
	{0x69,0xf7},
	{0x6a,0xa5},

	//indoor 11
	{0x7c,0x30},
	{0x7d,0x4b},
	{0x7e,0xf7},
	{0x7f,0x13},
	{0x80,0xa6},

	//cwf   12
	{0x70,0x24},
	{0x71,0x3d},
	{0x72,0x24},
	{0x73,0x49},
	{0x74,0xaa},

	//tl84  13
	{0x6b,0x0b},//tl84  13
	{0x6c,0x24},
	{0x6d,0x30},
	{0x6e,0x4b},
	{0x6f,0xaa},

	//f   14
	{0x61,0xf7},//f14
	{0x62,0x14},
	{0x63,0x4b},
	{0x64,0x68},
	{0x65,0x6a},

	{0x75,0x80},
	{0x76,0x09},
	{0x77,0x02},
	{0x24,0x25},
	{0x0e,0x16},
	{0x3b,0x09},

	// sharp

	{0xfd,0x02},//sharp
	{0xde,0x0f},
	{0xd7,0x06},//sharp_flat_thr 轮廓判断
	{0xd8,0x06},
	{0xd9,0x10},
	{0xda,0x14},
	{0xe8,0x24},//sharp_fac_pos 轮廓强度
	{0xe9,0x34},
	{0xea,0x24},
	{0xeb,0x20},
	{0xec,0x24},//sharp_fac_neg
	{0xed,0x3a},
	{0xee,0x24},
	{0xef,0x24},

	{0xd3,0x20}, //sharp_ofst_pos
	{0xd4,0x38}, //sharp_ofst_neg
	{0xd5,0x20}, //sharp_ofst_min
	{0xd6,0x08}, //sharp_k_val



	//skin sharpen        
	{0xfd,0x01},
	{0xd1,0x20},//skin_sharp_delta
	{0xfd,0x02},
	{0xdc,0x07},//肤色降锐化 skin_sharp_sel
	{0x05,0x20},//排除肤色降`锐化对分辨率卡引起的干扰

	//BPC
	{0xfd,0x02},
	{0x81,0x00},//bpc_ratio_vt
	{0xfd,0x01},
	{0xfc,0x00},//bpc_median_en
	{0x7d,0x05},//bpc_med_thr
	{0x7e,0x05},
	{0x7f,0x09},
	{0x80,0x08},

	//dns
	{0xfd,0x02}, 
	{0xdd,0x0f},//enable
	{0xfd,0x01},//沿方向边缘平滑阈值，越小越弱

	{0x6d,0x08},//dns_flat_dif 强平滑（平坦）区域平滑阈值  0x81↓
	{0x6e,0x08},
	{0x6f,0x10},
	{0x70,0x18},
	{0x86,0x18},//dark
	{0x71,0x0a},//dns_edge_dif 弱轮廓（非平坦）区域平滑阈值	 0x81↑
	{0x72,0x0a},
	{0x73,0x14},
	{0x74,0x14},

	{0x75,0x08},//dns_edge_gdif
	{0x76,0x0a},
	{0x77,0x06},
	{0x78,0x06},
	{0x79,0x56},//raw_flat_fac, raw_edge_fac  
	{0x7a,0x55},
	{0x7b,0x33},
	{0x7c,0x22},

	{0x81,0x0d},//2x;dns_flat_thr 根据增益判定区域阈值
	{0x82,0x18},//4x
	{0x83,0x20},//8x
	{0x84,0x24},//16x
	//dem
	{0xfd,0x02},//dem  
	{0x83,0x12},//dem_morie_thr, dem_hfmax_thr}
	{0x84,0x14},//dem_grad_thr 
	{0x86,0x04},//dem_grad_dif
	//pf
	{0xfd,0x01},//pf
	{0x61,0x60},
	{0x62,0x28},
	{0x8a,0x10},

	//gamma  
	{0xfd,0x01},
	{0x8b,0x00},
	{0x8c,0x0a},
	{0x8d,0x18},
	{0x8e,0x29},
	{0x8f,0x39},
	{0x90,0x4f},
	{0x91,0x62},
	{0x92,0x71},
	{0x93,0x7f},
	{0x94,0x93},
	{0x95,0xa3},
	{0x96,0xb0},
	{0x97,0xbd},
	{0x98,0xc8},
	{0x99,0xd1},
	{0x9a,0xd9},
	{0x9b,0xe1},
	{0x9c,0xe8},
	{0x9d,0xee},
	{0x9e,0xf4},
	{0x9f,0xfa},
	{0xa0,0xff},

	//CCM
	{0xfd,0x02},//CCM
	{0x15,0xcc},//d4;b>th
	{0x16,0x8c},//a1;r<th  
	//!F
	{0xa0,0x8c},//66
	{0xa1,0xed},//4c
	{0xa2,0x06},//cd
	{0xa3,0xe3},//e0
	{0xa4,0xa0},//c0
	{0xa5,0xfe},//e0
	{0xa6,0xed},//f4
	{0xa7,0xba},//cd
	{0xa8,0xd9},//c0
	{0xa9,0x0c},//30
	{0xaa,0x33},//33
	{0xab,0x0f},//0f

	{0xac,0x80},//80
	{0xad,0xed},//06
	{0xae,0x13},//fa
	{0xaf,0xcd},//da
	{0xb0,0x99},//d9
	{0xb1,0x19},//cd
	{0xb2,0xc7},//da
	{0xb3,0x9a},//c0
	{0xb4,0x20},//e6
	{0xb5,0x0c},//30
	{0xb6,0x03},//33
	{0xb7,0x1f},//0f


	//sat u 
	{0xfd,0x01},//sat u 
	{0xd3,0x76},
	{0xd4,0x7a},
	{0xd5,0x66},
	{0xd6,0x54},
	//sat v
	{0xd7,0x76},     
	{0xd8,0x7a},
	{0xd9,0x66},
	{0xda,0x54},
	//auto_sat
	{0xfd,0x01},//auto_sat
	{0xdd,0x30},
	{0xde,0x10},
	{0xdf,0xff},//a0;y_mean_th
	{0x00,0x00},//status

	//uv_th
	{0xfd,0x01},//白色物体表面有彩色噪声降低此值;uv_th
	{0xc2,0x88},//0xaa
	{0xc3,0x66},//0x88
	{0xc4,0x55},//0x77
	{0xc5,0x44},//0x66

	//low_lum_offset
	{0xfd,0x01},
	{0xcd,0x10},
	{0xce,0x1f},
	{0xcf,0x30},
	{0xd0,0x45},

	//gw
	{0xfd,0x02},//low_lum_offset
	{0x31,0x60},
	{0x32,0x60},
	{0x33,0xc0},
	{0x35,0x60},
	{0x37,0x13},

	//heq//modify by lwy_20141022
	{0xfd,0x01},//heq                         
	{0x0e,0x80},      
	{0x0f,0x20},//k_max
	{0x10,0x8a},//ku_outdoor
	{0x11,0x8a},//ku_nr
	{0x12,0x8a},//ku_dummy
	{0x13,0x8e},//ku_low
	{0x14,0x8e},//kl_outdoor 
	{0x15,0x8c},//kl_nr      
	{0x16,0x8a},//kl_dummy    
	{0x17,0x88},//kl_low        

	{0xfd,0x00},
	{0x31,0x00},
	{0xfd,0x01},
	{0x32,0x15},
	{0x33,0xef},
	{0x34,0x07},
	{0xd2,0x01}, 
	{0xfb,0x25},
	{0xf2,0x49},
	{0x35,0x40},
	{0x5d,0x11}, 
	{0xfd,0x00},//out en
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_saturation[11][10] = {
#ifndef FILE_INIT_REG_DEBUG
	{
		//Saturation level 0
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3 - 0x3c},
		{0xd4, SP0A20_P1_0xd4 - 0x3c},
		{0xd5, SP0A20_P1_0xd5 - 0x3c},
		{0xd6, SP0A20_P1_0xd6 - 0x3c},
		{0xd7, SP0A20_P1_0xd7 - 0x3c},
		{0xd8, SP0A20_P1_0xd8 - 0x3c},
		{0xd9, SP0A20_P1_0xd9 - 0x3c},
		{0xda, SP0A20_P1_0xda - 0x3c},
		{0xfd, 0x00},
	}, /* SATURATION LEVEL0*/

	{
			//Saturation level 1
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3 - 0x38},
		{0xd4, SP0A20_P1_0xd4 - 0x38},
		{0xd5, SP0A20_P1_0xd5 - 0x38},
		{0xd6, SP0A20_P1_0xd6 - 0x38},
		{0xd7, SP0A20_P1_0xd7 - 0x38},
		{0xd8, SP0A20_P1_0xd8 - 0x38},
		{0xd9, SP0A20_P1_0xd9 - 0x38},
		{0xda, SP0A20_P1_0xda - 0x38},
		{0xfd, 0x00},  
	}, /* SATURATION LEVEL1*/

	{
				//Saturation level 2
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3 - 0x30},
		{0xd4, SP0A20_P1_0xd4 - 0x30},
		{0xd5, SP0A20_P1_0xd5 - 0x30},
		{0xd6, SP0A20_P1_0xd6 - 0x30},
		{0xd7, SP0A20_P1_0xd7 - 0x30},
		{0xd8, SP0A20_P1_0xd8 - 0x30}, 
		{0xd9, SP0A20_P1_0xd9 - 0x30},
		{0xda, SP0A20_P1_0xda - 0x30},
		{0xfd, 0x00},
	}, /* SATURATION LEVEL2*/

	{
	//Saturation level 3
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3 - 0x20},
		{0xd4, SP0A20_P1_0xd4 - 0x20},
		{0xd5, SP0A20_P1_0xd5 - 0x20},
		{0xd6, SP0A20_P1_0xd6 - 0x20},
		{0xd7, SP0A20_P1_0xd7 - 0x20},
		{0xd8, SP0A20_P1_0xd8 - 0x20},
		{0xd9, SP0A20_P1_0xd9 - 0x20},
		{0xda, SP0A20_P1_0xda - 0x20},
		{0xfd, 0x00},
	}, /* SATURATION LEVEL3*/

	{
			//Saturation level 4
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3 - 0x10},
		{0xd4, SP0A20_P1_0xd4 - 0x10},
		{0xd5, SP0A20_P1_0xd5 - 0x10},
		{0xd6, SP0A20_P1_0xd6 - 0x10},
		{0xd7, SP0A20_P1_0xd7 - 0x10},
		{0xd8, SP0A20_P1_0xd8 - 0x10},
		{0xd9, SP0A20_P1_0xd9 - 0x10},
		{0xda, SP0A20_P1_0xda - 0x10},
		{0xfd, 0x00},
	}, /* SATURATION LEVEL4*/

	{
			//Saturation level 5 (default)
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3},
		{0xd4, SP0A20_P1_0xd4},
		{0xd5, SP0A20_P1_0xd5},
		{0xd6, SP0A20_P1_0xd6},
		{0xd7, SP0A20_P1_0xd7},
		{0xd8, SP0A20_P1_0xd8},
		{0xd9, SP0A20_P1_0xd9},
		{0xda, SP0A20_P1_0xda},
		{0xfd, 0x00},
	}, /* SATURATION LEVEL5*/

	{
		//Saturation level 6
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3 + 0x10},
		{0xd4, SP0A20_P1_0xd4 + 0x10},
		{0xd5, SP0A20_P1_0xd5 + 0x10},
		{0xd6, SP0A20_P1_0xd6 + 0x10},
		{0xd7, SP0A20_P1_0xd7 + 0x10},
		{0xd8, SP0A20_P1_0xd8 + 0x10},
		{0xd9, SP0A20_P1_0xd9 + 0x10},
		{0xda, SP0A20_P1_0xda + 0x10},
		{0xfd, 0x00},
	}, /* SATURATION LEVEL6*/

	{
		//Saturation level 7
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3 + 0x20},
		{0xd4, SP0A20_P1_0xd4 + 0x20},
		{0xd5, SP0A20_P1_0xd5 + 0x20},
		{0xd6, SP0A20_P1_0xd6 + 0x20},
		{0xd7, SP0A20_P1_0xd7 + 0x20},
		{0xd8, SP0A20_P1_0xd8 + 0x20},
		{0xd9, SP0A20_P1_0xd9 + 0x20},
		{0xda, SP0A20_P1_0xda + 0x20},
		{0xfd, 0x00},
	}, /* SATURATION LEVEL7*/

	{
		//Saturation level 8
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3 + 0x30},
		{0xd4, SP0A20_P1_0xd4 + 0x30},
		{0xd5, SP0A20_P1_0xd5 + 0x30},
		{0xd6, SP0A20_P1_0xd6 + 0x30},
		{0xd7, SP0A20_P1_0xd7 + 0x30},
		{0xd8, SP0A20_P1_0xd8 + 0x30},
		{0xd9, SP0A20_P1_0xd9 + 0x30},
		{0xda, SP0A20_P1_0xda + 0x30},
		{0xfd, 0x00},
	}, /* SATURATION LEVEL8*/

	{
		//Saturation level 9
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3 + 0x40},
		{0xd4, SP0A20_P1_0xd4 + 0x40},
		{0xd5, SP0A20_P1_0xd5 + 0x40},
		{0xd6, SP0A20_P1_0xd6 + 0x40},
		{0xd7, SP0A20_P1_0xd7 + 0x40},
		{0xd8, SP0A20_P1_0xd8 + 0x40},
		{0xd9, SP0A20_P1_0xd9 + 0x40},
		{0xda, SP0A20_P1_0xda + 0x40},
		{0xfd, 0x00},
	}, /* SATURATION LEVEL9*/

	{
			//Saturation level 10
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3 + 0x50},
		{0xd4, SP0A20_P1_0xd4 + 0x50},
		{0xd5, SP0A20_P1_0xd5 + 0x50},
		{0xd6, SP0A20_P1_0xd6 + 0x50},
		{0xd7, SP0A20_P1_0xd7 + 0x50},
		{0xd8, SP0A20_P1_0xd8 + 0x50},
		{0xd9, SP0A20_P1_0xd9 + 0x50},
		{0xda, SP0A20_P1_0xda + 0x50},
		{0xfd, 0x00},
	}, /* SATURATION LEVEL10*/
#endif
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_contrast[11][10] = {
#ifndef FILE_INIT_REG_DEBUG
	{
		//standard curve   3.0
		//Contrast -5
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10 - 0x28},
		{0x11, SP0A20_P1_0x11 - 0x28},
		{0x12, SP0A20_P1_0x12 - 0x28},
		{0x13, SP0A20_P1_0x13 - 0x28},
		{0x14, SP0A20_P1_0x14 - 0x28},
		{0x15, SP0A20_P1_0x15 - 0x28},
		{0x16, SP0A20_P1_0x16 - 0x28},
		{0x17, SP0A20_P1_0x17 - 0x28},
		{0xfd, 0x00},
	}, /* CONTRAST L0*/
	{
		//standard curve   3.5
		//Contrast -4
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10 - 0x20},
		{0x11, SP0A20_P1_0x11 - 0x20},
		{0x12, SP0A20_P1_0x12 - 0x20},
		{0x13, SP0A20_P1_0x13 - 0x20},
		{0x14, SP0A20_P1_0x14 - 0x20},
		{0x15, SP0A20_P1_0x15 - 0x20},
		{0x16, SP0A20_P1_0x16 - 0x20},
		{0x17, SP0A20_P1_0x17 - 0x20},
		{0xfd, 0x00},
	}, /* CONTRAST L1*/
	{
		//standard curve   4.0
		//Contrast -3
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10 - 0x18},
		{0x11, SP0A20_P1_0x11 - 0x18},
		{0x12, SP0A20_P1_0x12 - 0x18},
		{0x13, SP0A20_P1_0x13 - 0x18},
		{0x14, SP0A20_P1_0x14 - 0x18},
		{0x15, SP0A20_P1_0x15 - 0x18},
		{0x16, SP0A20_P1_0x16 - 0x18},
		{0x17, SP0A20_P1_0x17 - 0x18},
		{0xfd, 0x00},
	}, /* CONTRAST L2*/
	{
		//standard curve   4.5
		//Contrast -2
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10 - 0x10},
		{0x11, SP0A20_P1_0x11 - 0x10},
		{0x12, SP0A20_P1_0x12 - 0x10},
		{0x13, SP0A20_P1_0x13 - 0x10},
		{0x14, SP0A20_P1_0x14 - 0x10},
		{0x15, SP0A20_P1_0x15 - 0x10},
		{0x16, SP0A20_P1_0x16 - 0x10},
		{0x17, SP0A20_P1_0x17 - 0x10},
		{0xfd, 0x00},
	}, /* CONTRAST L3*/
	{
		//standard curve   5.0
		//Contrast -1
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10 - 0x08},
		{0x11, SP0A20_P1_0x11 - 0x08},
		{0x12, SP0A20_P1_0x12 - 0x08},
		{0x13, SP0A20_P1_0x13 - 0x08},
		{0x14, SP0A20_P1_0x14 - 0x08},
		{0x15, SP0A20_P1_0x15 - 0x08},
		{0x16, SP0A20_P1_0x16 - 0x08},
		{0x17, SP0A20_P1_0x17 - 0x08},
		{0xfd, 0x00},
	}, /* CONTRAST L4*/
	{
		//default
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10},
		{0x11, SP0A20_P1_0x11},
		{0x12, SP0A20_P1_0x12},
		{0x13, SP0A20_P1_0x13},
		{0x14, SP0A20_P1_0x14 },
		{0x15, SP0A20_P1_0x15},
		{0x16, SP0A20_P1_0x16},
		{0x17, SP0A20_P1_0x17},
		{0xfd, 0x00},
	}, /* CONTRAST L5*/
	{
		//standard curve   6.0
		//Contrast 1
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10 + 0x08},
		{0x11, SP0A20_P1_0x11 + 0x08},
		{0x12, SP0A20_P1_0x12 + 0x08},
		{0x13, SP0A20_P1_0x13 + 0x08},
		{0x14, SP0A20_P1_0x14 + 0x08},
		{0x15, SP0A20_P1_0x15 + 0x08},
		{0x16, SP0A20_P1_0x16 + 0x08},
		{0x17, SP0A20_P1_0x17 + 0x08},
		{0xfd, 0x00},
	}, /* CONTRAST L6*/
	{
		//standard curve   6.5
		//Contrast 2
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10 + 0x10},
		{0x11, SP0A20_P1_0x11 + 0x10},
		{0x12, SP0A20_P1_0x12 + 0x10},
		{0x13, SP0A20_P1_0x13 + 0x10},
		{0x14, SP0A20_P1_0x14 + 0x10},
		{0x15, SP0A20_P1_0x15 + 0x10},
		{0x16, SP0A20_P1_0x16 + 0x10},
		{0x17, SP0A20_P1_0x17 + 0x10},
		{0xfd, 0x00},
	}, /* CONTRAST L7*/
	{
		//standard curve   7.0
		//Contrast 3
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10 + 0x18},
		{0x11, SP0A20_P1_0x11 + 0x18},
		{0x12, SP0A20_P1_0x12 + 0x18},
		{0x13, SP0A20_P1_0x13 + 0x18},
		{0x14, SP0A20_P1_0x14 + 0x18},
		{0x15, SP0A20_P1_0x15 + 0x18},
		{0x16, SP0A20_P1_0x16 + 0x18},
		{0x17, SP0A20_P1_0x17 + 0x18},
		{0xfd, 0x00},
	}, /* CONTRAST L8*/
	{
		//standard curve   7.5
		//Contrast 4
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10 + 0x20},
		{0x11, SP0A20_P1_0x11 + 0x20},
		{0x12, SP0A20_P1_0x12 + 0x20},
		{0x13, SP0A20_P1_0x13 + 0x20},
		{0x14, SP0A20_P1_0x14 + 0x20},
		{0x15, SP0A20_P1_0x15 + 0x20},
		{0x16, SP0A20_P1_0x16 + 0x20},
		{0x17, SP0A20_P1_0x17 + 0x20},
		{0xfd, 0x00},
	}, /* CONTRAST L9*/
	{
		//standard curve   8.0
		//Contrast 5
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10 + 0x28},
		{0x11, SP0A20_P1_0x11 + 0x28},
		{0x12, SP0A20_P1_0x12 + 0x28},
		{0x13, SP0A20_P1_0x13 + 0x28},
		{0x14, SP0A20_P1_0x14 + 0x28},
		{0x15, SP0A20_P1_0x15 + 0x28},
		{0x16, SP0A20_P1_0x16 + 0x28},
		{0x17, SP0A20_P1_0x17 + 0x28},
		{0xfd, 0x00},
	},/* CONTRAST L10*/
#endif
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_sharpness[7][10] = {
#ifndef FILE_INIT_REG_DEBUG
	{
		//Sharpness 0
		{0xfd, 0x02},
		{0xe8, SP0A20_P2_0xe8 - 0x0c},
		{0xec, SP0A20_P2_0xec - 0x0c},
		{0xe9, SP0A20_P2_0xe9 - 0x0c},
		{0xed, SP0A20_P2_0xed - 0x0c},
		{0xea, SP0A20_P2_0xea - 0x0c},
		{0xee, SP0A20_P2_0xee - 0x0c},
		{0xeb, SP0A20_P2_0xeb - 0x0c},
		{0xef, SP0A20_P2_0xef  - 0x0c},
		{0xfd, 0x00},
	}, /* SHARPNESS LEVEL 0*/
	{
		//Sharpness 1
		{0xfd, 0x02},
		{0xe8, SP0A20_P2_0xe8 - 0x08},
		{0xec, SP0A20_P2_0xec - 0x08},
		{0xe9, SP0A20_P2_0xe9 - 0x08},
		{0xed, SP0A20_P2_0xed - 0x08},
		{0xea, SP0A20_P2_0xea - 0x08},
		{0xee, SP0A20_P2_0xee - 0x08},
		{0xeb, SP0A20_P2_0xeb - 0x08},
		{0xef, SP0A20_P2_0xef  - 0x08},
		{0xfd, 0x00},
	}, /* SHARPNESS LEVEL 1*/
	{
		//Sharpness 2
		{0xfd, 0x02},
		{0xe8, SP0A20_P2_0xe8 - 0x04},
		{0xec, SP0A20_P2_0xec - 0x04},
		{0xe9, SP0A20_P2_0xe9 - 0x04},
		{0xed, SP0A20_P2_0xed - 0x04},
		{0xea, SP0A20_P2_0xea - 0x04},
		{0xee, SP0A20_P2_0xee - 0x04},
		{0xeb, SP0A20_P2_0xeb - 0x04},
		{0xef, SP0A20_P2_0xef  - 0x04},
		{0xfd, 0x00},
	}, /* SHARPNESS LEVEL 2*/
	{
		         //Sharpness Auto (Default)
		{0xfd, 0x02},
		{0xe8, SP0A20_P2_0xe8},
		{0xec, SP0A20_P2_0xec},
		{0xe9, SP0A20_P2_0xe9},
		{0xed, SP0A20_P2_0xed},
		{0xea, SP0A20_P2_0xea},
		{0xee, SP0A20_P2_0xee},
		{0xeb, SP0A20_P2_0xeb},
		{0xef, SP0A20_P2_0xef}, 
		{0xfd, 0x00},
	}, /* SHARPNESS LEVEL 3*/
	{
		//Sharpness 4
		{0xfd, 0x02},
		{0xe8, SP0A20_P2_0xe8 + 0x08},
		{0xec, SP0A20_P2_0xec + 0x08},
		{0xe9, SP0A20_P2_0xe9 + 0x08},
		{0xed, SP0A20_P2_0xed + 0x08},
		{0xea, SP0A20_P2_0xea + 0x08},
		{0xee, SP0A20_P2_0xee + 0x08},
		{0xeb, SP0A20_P2_0xeb + 0x08},
		{0xef, SP0A20_P2_0xef + 0x08},
		{0xfd, 0x00},
	}, /* SHARPNESS LEVEL 4*/
	{
		//Sharpness 5
		{0xfd, 0x02},
		{0xe8, SP0A20_P2_0xe8 + 0x10},
		{0xec, SP0A20_P2_0xec + 0x10},
		{0xe9, SP0A20_P2_0xe9 + 0x10},
		{0xed, SP0A20_P2_0xed + 0x10},
		{0xea, SP0A20_P2_0xea + 0x10},
		{0xee, SP0A20_P2_0xee + 0x10},
		{0xeb, SP0A20_P2_0xeb + 0x10},
		{0xef, SP0A20_P2_0xef + 0x10},
		{0xfd, 0x00},
	}, /* SHARPNESS LEVEL 5*/
	{
		//Sharpness 6
		{0xfd, 0x02},
		{0xe8, SP0A20_P2_0xe8 + 0x20},
		{0xec, SP0A20_P2_0xec + 0x20},
		{0xe9, SP0A20_P2_0xe9 + 0x20},
		{0xed, SP0A20_P2_0xed + 0x20},
		{0xea, SP0A20_P2_0xea + 0x20},
		{0xee, SP0A20_P2_0xee + 0x20},
		{0xeb, SP0A20_P2_0xeb + 0x20},
		{0xef, SP0A20_P2_0xef + 0x20},
		{0xfd, 0x00},
	}, /* SHARPNESS LEVEL 6*/
#endif
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_iso[7][3] = {
#ifndef FILE_INIT_REG_DEBUG
	/* auto */
	{
		{0xfd, 0x00},		
	},
	/* auto hjt */
	{
		{0xfd, 0x00},	
	},
	/* iso 100 */
	{
		{0xfd, 0x00},
		{0x24, 0x20},
		{0xfd, 0x00},
	},
	/* iso 200 */
	{
		{0xfd, 0x00},
		{0x24, 0x30},
		{0xfd, 0x00},
	},
	/* iso 400 */
	{
		{0xfd, 0x00},
		{0x24, 0x40},
		{0xfd, 0x00},
	},
	/* iso 800 */
	{
		{0xfd, 0x00},
		{0x24, 0x50},
		{0xfd, 0x00},
	},
	/* iso 1600 */
	{
		{0xfd, 0x00},
		{0x24, 0x60},
		{0xfd, 0x00},
	},
#endif
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_exposure_compensation[5][3] = {
#ifndef FILE_INIT_REG_DEBUG
	{
		                	//@@ +2EV
		{0xfd, 0x01},
		{0xdb, 0xe0},
		{0xfd, 0x00},
	}, /*EXPOSURECOMPENSATIONN2*/
	{
				       	//@@ +1EV
		{0xfd, 0x01},
		{0xdb, 0xf0},  
		{0xfd, 0x00},
	}, /*EXPOSURECOMPENSATIONN1*/
	{
			  	//@@ default
		{0xfd, 0x01},
		{0xdb, 0x00},  
		{0xfd, 0x00},
	}, /*EXPOSURECOMPENSATIOND*/
	{
			//@@ -1EV
		{0xfd, 0x01},
		{0xdb, 0x10},	 
		{0xfd, 0x00},
	}, /*EXPOSURECOMPENSATIONP1*/
	{
				//@@ -2EV
		{0xfd, 0x01},
		{0xdb, 0x20}, 
		{0xfd, 0x00},
	}, /*EXPOSURECOMPENSATIONP2*/
#endif
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_antibanding[][33] = {
	/* OFF */
	{
		//24M 50Hz 6-20fps
	/*	{0xfd,0x00},        
		{0x03,0x01},        
		{0x04,0xc2},        
		{0x05,0x00},        
		{0x06,0x00},        
		{0x07,0x00},        
		{0x08,0x00},        
		{0x09,0x02},        
		{0x0a,0xf4},        
		{0xfd,0x01},        
		{0xf0,0x00},        
		{0xf7,0x4b},        
		{0x02,0x0e},        
		{0x03,0x01},        
		{0x06,0x4b},        
		{0x07,0x00},        
		{0x08,0x01},        
		{0x09,0x00},        
		{0xfd,0x02},        
		{0xbe,0x1a},        
		{0xbf,0x04},        
		{0xd0,0x1a},        
		{0xd1,0x04},        
		{0xfd,0x00}, */
	},

	{
	//60Hz 24M 8-15fps    	/* 60Hz */
		{0xfd,0x00},
		{0x03,0x01},
		{0x04,0x7a},
		{0x05,0x00},
		{0x06,0x00},
		{0x07,0x00},
		{0x08,0x00},
		{0x09,0x02},
		{0x0a,0xe7},
		{0xfd,0x01},
		{0xf0,0x00},
		{0xf7,0x3f},
		{0x02,0x0f},
		{0x03,0x01},
		{0x06,0x3f},
		{0x07,0x00},
		{0x08,0x01},
		{0x09,0x00},
		{0xfd,0x02},
		{0xbe,0xb1},
		{0xbf,0x03},
		{0xd0,0xb1},
		{0xd1,0x03},
		{0xfd,0x00},  
	},
	
	{
		//24M 50Hz 8-15fps /* 50Hz */
		{0xfd,0x00},
		{0x03,0x01},
		{0x04,0xc2},
		{0x05,0x00},
		{0x06,0x00},
		{0x07,0x00},
		{0x08,0x00},
		{0x09,0x02},
		{0x0a,0xf4},
		{0xfd,0x01},
		{0xf0,0x00},
		{0xf7,0x4b},
		{0x02,0x0c},
		{0x03,0x01},
		{0x06,0x4b},
		{0x07,0x00},
		{0x08,0x01},
		{0x09,0x00},
		{0xfd,0x02},
		{0xbe,0x84},
		{0xbf,0x03},
		{0xd0,0x84},
		{0xd1,0x03},
		{0xfd,0x00}, 
	},
	
	{
		//24M 50Hz 8-15fps /* AUTO */
		{0xfd,0x00},
		{0x03,0x01},
		{0x04,0xc2},
		{0x05,0x00},
		{0x06,0x00},
		{0x07,0x00},
		{0x08,0x00},
		{0x09,0x02},
		{0x0a,0xf4},
		{0xfd,0x01},
		{0xf0,0x00},
		{0xf7,0x4b},
		{0x02,0x0c},
		{0x03,0x01},
		{0x06,0x4b},
		{0x07,0x00},
		{0x08,0x01},
		{0x09,0x00},
		{0xfd,0x02},
		{0xbe,0x84},
		{0xbf,0x03},
		{0xd0,0x84},
		{0xd1,0x03},
		{0xfd,0x00}, 
 },
};

//begin effect
static struct msm_camera_i2c_reg_conf sp0a20_reg_effect_normal[] = {
		{0xfd,0x01},
		{0x66,0x00},
		{0x67,0x80},
		{0x68,0x80},
		{0xdf,0x00},
		{0xfd,0x02},
		{0x14,0x00},
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_effect_black_white[] = {
		/* B&W: */
		{0xfd,0x01},
		{0x66,0x20},
		{0x67,0x80},
		{0x68,0x80},
		{0xdf,0x00},
		{0xfd,0x02},
		{0x14,0x00},
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_effect_negative[] = {
		/* Negative: */
		{0xfd,0x01},
		{0x66,0x04},
		{0x67,0x80},
		{0x68,0x80},
		{0xdf,0x00},
		{0xfd,0x02},
		{0x14,0x00},
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_effect_old_movie[] = {
		/* Sepia(antique): */
		{0xfd,0x01},
		{0x66,0x10},
		{0x67,0x98},
		{0x68,0x58},
		{0xdf,0x00},
		{0xfd,0x02},
		{0x14,0x00},
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_effect_sepiablue[] = {
		/* Sepiabule: */
		{0xfd,0x01},
		{0x66,0x10},
		{0x67,0x80},
		{0x68,0xb0},
		{0xdf,0x00},
		{0xfd,0x02},
		{0x14,0x00},
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_effect_solarize[] = {
		{0xfd,0x01},
		{0x66,0x80},
		{0x67,0x80},
		{0x68,0x80},
		{0xdf,0x80},
		{0xfd,0x02},
		{0x14,0x00},
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_effect_emboss[] = {
        {0xfd,0x01},
		{0x66,0x01},
		{0x67,0x80},
		{0x68,0x80},
		{0xdf,0x00},
		{0xfd,0x02},
		{0x14,0x00},
};
// end effect

#if 0
static struct msm_camera_i2c_reg_conf sp0a20_reg_scene_auto[] = {
	/* <SCENE_auto> */
		//24M 50Hz 7-15fps
		{0xfd,0x00},
		{0x03,0x01},
		{0x04,0xc2},
		{0x05,0x00},
		{0x06,0x00},
		{0x07,0x00},
		{0x08,0x00},
		{0x09,0x02},
		{0x0a,0xf4},
		{0xfd,0x01},
		{0xf0,0x00},
		{0xf7,0x4b},
		{0x02,0x0e},
		{0x03,0x01},
		{0x06,0x4b},
		{0x07,0x00},
		{0x08,0x01},
		{0x09,0x00},
		{0xfd,0x02},
		{0xbe,0x1a},
		{0xbf,0x04},
		{0xd0,0x1a},
		{0xd1,0x04},
		{0xfd,0x00},
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_scene_portrait[] = {
	/* <CAMTUNING_SCENE_PORTRAIT> */
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_scene_landscape[] = {
	/* <CAMTUNING_SCENE_LANDSCAPE> */
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_scene_night[] = {
	/* <SCENE_NIGHT> */
	//24M 50Hz 6-20fps  
		{0xfd,0x00},      
		{0x03,0x01},      
		{0x04,0xf2},      
		{0x05,0x00},      
		{0x06,0x00},      
		{0x07,0x00},      
		{0x08,0x00},      
		{0x09,0x01},      
		{0x0a,0x66},      
		{0xfd,0x01},      
		{0xf0,0x00},      
		{0xf7,0x53},      
		{0x02,0x14},      
		{0x03,0x01},      
		{0x06,0x53},      
		{0x07,0x00},      
		{0x08,0x01},      
		{0x09,0x00},      
		{0xfd,0x02},      
		{0xbe,0x7c},      
		{0xbf,0x06},      
		{0xd0,0x7c},      
		{0xd1,0x06},  
		{0xfd,0x00},
};
#endif

//begin white balance
static struct msm_camera_i2c_reg_conf sp0a20_reg_wb_auto[] = {
		/* Auto: */
		{0xfd, 0x02},
		{0x26, 0xbf},
		{0x27, 0xa3},
		{0xfd, 0x01},
		{0x32, 0x15},
		{0xfd, 0x00},
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_wb_sunny[] = {
		/*DAYLIGHT*/
		{0xfd, 0x01},
		{0x32, 0x05},
		{0xfd, 0x02},
		{0x26, 0xd1},
		{0x27, 0x85},
		{0xfd, 0x00},
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_wb_cloudy[] = {
		/*CLOUDY*/
		{0xfd, 0x01},
		{0x32, 0x05},
		{0xfd, 0x02},
		{0x26, 0xdb},
		{0x27, 0x70},
		{0xfd, 0x00},
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_wb_office[] = {
		/* Office: *//*INCANDISCENT*/
		{0xfd, 0x01},
		{0x32, 0x05},
		{0xfd, 0x02},
		{0x26, 0x88},
		{0x27, 0xd0},
		{0xfd, 0x00},
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_wb_home[] = {
		/* Home: */
		{0xfd, 0x01},
		{0x32, 0x05},
		{0xfd, 0x02},
		{0x26, 0xb4},
		{0x27, 0xc4},
		{0xfd, 0x00},	
};
//end white balance

static struct v4l2_subdev_info sp0a20_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id sp0a20_i2c_id[] = {
	{SP0A20_SENSOR_NAME, (kernel_ulong_t)&sp0a20_s_ctrl},
	{ }
};

static int32_t msm_sp0a20_i2c_probe(struct i2c_client *client,
	   const struct i2c_device_id *id)
{
    CDBG("%s, enter\n", __FUNCTION__);
	return msm_sensor_i2c_probe(client, id, &sp0a20_s_ctrl);
}

static struct i2c_driver sp0a20_i2c_driver = {
	.id_table = sp0a20_i2c_id,
	.probe  = msm_sp0a20_i2c_probe,
	.driver = {
		.name = SP0A20_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client sp0a20_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static const struct of_device_id sp0a20_dt_match[] = {
	{.compatible = SP0A20_PLATFORM_NAME, .data = &sp0a20_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, sp0a20_dt_match);

static int32_t sp0a20_platform_probe(struct platform_device *pdev)
{
	int32_t rc;
	const struct of_device_id *match;
	match = of_match_device(sp0a20_dt_match, &pdev->dev);
	if (!match)
		return -EFAULT;
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static struct platform_driver sp0a20_platform_driver = {
	.driver = {
		.name = SP0A20_PLATFORM_NAME,
		.owner = THIS_MODULE,
		.of_match_table = sp0a20_dt_match,
	},
	.probe = sp0a20_platform_probe,
};
static int __init sp0a20_init_module(void)
{
	int32_t rc;
	pr_err("%s:%d\n", __func__, __LINE__);
	rc = i2c_add_driver(&sp0a20_i2c_driver);
	if (!rc)
		return rc;
	pr_err("%s:%d rc \n", __func__, __LINE__);
	return platform_driver_register(&sp0a20_platform_driver);
}

static void __exit sp0a20_exit_module(void)
{
	pr_err("%s:%d\n", __func__, __LINE__);
	if (sp0a20_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&sp0a20_s_ctrl);
		platform_driver_unregister(&sp0a20_platform_driver);
	} else
		i2c_del_driver(&sp0a20_i2c_driver);
	return;
}

static void sp0a20_i2c_write_table(struct msm_sensor_ctrl_t *s_ctrl,
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

static void sp0a20_set_saturation(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_saturation[value][0],
		ARRAY_SIZE(sp0a20_reg_saturation[value]));		
}

static void sp0a20_set_contrast(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_contrast[value][0],
		ARRAY_SIZE(sp0a20_reg_contrast[value]));
}

static void sp0a20_set_sharpness(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	int val = value / 6;
	
	sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_sharpness[val][0],
		ARRAY_SIZE(sp0a20_reg_sharpness[val]));
}

static void sp0a20_set_iso(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_iso[value][0],
		ARRAY_SIZE(sp0a20_reg_iso[value]));
}

static void sp0a20_set_exposure_compensation(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	int val = (value + 12) / 6;
	
	sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_exposure_compensation[val][0],
		ARRAY_SIZE(sp0a20_reg_exposure_compensation[val]));	   
		
}

static void sp0a20_set_effect(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	switch (value) {
	case MSM_CAMERA_EFFECT_MODE_OFF: {
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_effect_normal[0],
			ARRAY_SIZE(sp0a20_reg_effect_normal));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_MONO: {
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_effect_black_white[0],
			ARRAY_SIZE(sp0a20_reg_effect_black_white));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_NEGATIVE: {
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_effect_negative[0],
			ARRAY_SIZE(sp0a20_reg_effect_negative));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SEPIA: {
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_effect_old_movie[0],
			ARRAY_SIZE(sp0a20_reg_effect_old_movie));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_POSTERIZE:
	{
		
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_effect_emboss[0],
			ARRAY_SIZE(sp0a20_reg_effect_emboss));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_AQUA: {
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_effect_sepiablue[0],
			ARRAY_SIZE(sp0a20_reg_effect_sepiablue));
		break;
	}  
	case MSM_CAMERA_EFFECT_MODE_SOLARIZE: {
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_effect_solarize[0],
			ARRAY_SIZE(sp0a20_reg_effect_solarize));
		break;
	}
	default:
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_effect_normal[0],
			ARRAY_SIZE(sp0a20_reg_effect_normal));
		break;
	}
}

static void sp0a20_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_antibanding[value][0],
		ARRAY_SIZE(sp0a20_reg_antibanding[value]));
}
#if 0
static void sp0a20_set_scene_mode(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	switch (value) {
	case MSM_CAMERA_SCENE_MODE_OFF: {
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_scene_auto[0],
			ARRAY_SIZE(sp0a20_reg_scene_auto));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_NIGHT: {
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_scene_night[0],
			ARRAY_SIZE(sp0a20_reg_scene_night));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_LANDSCAPE: {
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_scene_landscape[0],
			ARRAY_SIZE(sp0a20_reg_scene_landscape));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_PORTRAIT: {
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_scene_portrait[0],
			ARRAY_SIZE(sp0a20_reg_scene_portrait));
		break;
	}
	default:
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_scene_auto[0],
			ARRAY_SIZE(sp0a20_reg_scene_auto));
		break;
	}
}
#endif
static void sp0a20_set_white_balance_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	switch (value) {
	case MSM_CAMERA_WB_MODE_AUTO: {
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_wb_auto[0],
			ARRAY_SIZE(sp0a20_reg_wb_auto));
		break;
	}
	case MSM_CAMERA_WB_MODE_INCANDESCENT: {
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_wb_home[0],
			ARRAY_SIZE(sp0a20_reg_wb_home));
		break;
	}
	case MSM_CAMERA_WB_MODE_DAYLIGHT: {
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_wb_sunny[0],
			ARRAY_SIZE(sp0a20_reg_wb_sunny));
		break;
	}
	case MSM_CAMERA_WB_MODE_FLUORESCENT: {
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_wb_office[0],
			ARRAY_SIZE(sp0a20_reg_wb_office));
		break;
	}
	case MSM_CAMERA_WB_MODE_CLOUDY_DAYLIGHT: {
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_wb_cloudy[0],
			ARRAY_SIZE(sp0a20_reg_wb_cloudy));
		break;
	}
	default:
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_wb_auto[0],
			ARRAY_SIZE(sp0a20_reg_wb_auto));
		break;
	}
}
#ifdef FILE_INIT_REG_DEBUG
static int32_t sp0a20_read_init_file(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	struct file *file_p = NULL;
	uint32_t file_init_size = 0;
	mm_segment_t fs = {0};
	loff_t pos = 0;
	uint8_t *data_buf = NULL;
	uint8_t *data_current_p = NULL;
	struct msm_camera_i2c_reg_conf *file_init_reg = NULL;
	uint32_t file_init_reg_num = 0;
	uint16_t reg_buf = 0; //addr, data buf
	uint8_t parse_state = 0; //000:before parse addr, 001:parse addr, 010:before parse data, 100:parse data

	printk(KERN_DEBUG"%s,%d\n",__func__,__LINE__);
	file_p = filp_open("/data/init_reg.txt", O_RDONLY , 0); //file open
	if (IS_ERR(file_p))
	{ 
		printk(KERN_ERR"%s,open init reg file error\n",__func__); 
		return -1; 
	}
	fs = get_fs(); 
	set_fs(KERNEL_DS); 
	file_init_size = vfs_llseek(file_p, 0, SEEK_END);
	if (file_init_size)
	{
	    data_buf = (uint8_t*)kzalloc(file_init_size + 1, GFP_KERNEL);
	    if (data_buf)
	    {
		    vfs_read(file_p, data_buf, file_init_size, &pos);
	    }
	}
	filp_close(file_p, NULL); //file close
	set_fs(fs);
	if (!file_init_size)
	{
	    printk(KERN_ERR"%s,init file is empty\n",__func__);
	    rc = -1;
	    goto end;
	}
	if (!data_buf)
	{
		printk(KERN_ERR"%s,kzalloc data buf fail\n",__func__);
		rc = -1;
		goto end;
	}
	file_init_reg = (struct msm_camera_i2c_reg_conf*)kzalloc(1000 * sizeof(struct msm_camera_i2c_reg_conf), GFP_KERNEL);
	if (!file_init_reg)
	{
		printk(KERN_ERR"%s,kzalloc init reg fail\n",__func__);
		rc = -1;
		goto end;
	}
	//parse file data
	data_current_p = data_buf;
	while (data_current_p < (data_buf + file_init_size))
	{
		//skip "//"
		if (*data_current_p == '/' && *(data_current_p + 1) == '/')
		{
			uint8_t *cs_p = strchr(data_current_p, '\n');
			if (cs_p)
			{
				data_current_p = cs_p + 1;
			}
			else
			{
				data_current_p = data_buf + file_init_size;
			}
			continue;
		}
		//skip "/*", "*/"
		if (*data_current_p == '/' && *(data_current_p + 1) == '*')
		{
			uint8_t *cs_p = strstr(data_current_p, "*/");
			if (cs_p)
			{
				data_current_p = cs_p + 2;
			}
			else
			{
				data_current_p = data_buf + file_init_size;
			}
			continue;
		}
		//skip "0x", "0X"
		if (*data_current_p == '0' && 
			(*(data_current_p + 1) == 'x' || *(data_current_p + 1) == 'X'))
		{
			data_current_p += 2;
			continue;
		}
		if (!(*data_current_p >= '0' && *data_current_p <= '9') && 
			!(*data_current_p >= 'a' && *data_current_p <= 'f') && 
			!(*data_current_p >= 'A' && *data_current_p <= 'F'))
		{
			if (parse_state == 1) //after parse addr
			{
				parse_state = (1 << 1);
				//set reg_addr
				file_init_reg[file_init_reg_num].reg_addr = reg_buf;
			}
			else if (parse_state == (1 << 2)) //after parse data
			{
				parse_state = 0;
				//set reg_data
				file_init_reg[file_init_reg_num].reg_data = reg_buf;
				printk(KERN_DEBUG"%s,id:%u,addr:0x%x,data:0x%x\n", 
					__func__,
					file_init_reg_num, 
					file_init_reg[file_init_reg_num].reg_addr,
					file_init_reg[file_init_reg_num].reg_data);
				file_init_reg_num++; //init reg number increase 1
			}
			data_current_p++;
			continue;
		}
		if (parse_state == 0 || parse_state == (1 << 1)) //before parse, set buf 0
		{
			reg_buf = 0;
		}
		if (parse_state == 0 || parse_state == 1) //parse addr
		{
			parse_state = 1;
		} 
		else if (parse_state == (1 << 1) || parse_state == (1 << 2)) //parse data
		{
			parse_state = (1 << 2);
		} 
		else
		{
			printk(KERN_ERR"%s,parse state error:%u\n",__func__,parse_state);
			rc = -1;
			goto end;
		}
		reg_buf *= 16;
		if (*data_current_p >= '0' && *data_current_p <= '9')
		{
			reg_buf += *data_current_p - '0';
		}
		else if (*data_current_p >= 'a' && *data_current_p <= 'f')
		{
			reg_buf += *data_current_p - 'a' + 10;
		}
		else if (*data_current_p >= 'A' && *data_current_p <= 'F')
		{
			reg_buf += *data_current_p - 'A' + 10;
		}
		else
		{
			printk(KERN_ERR"%s,parse reg buf error:%c\n",__func__,*data_current_p);
			rc = -1;
			goto end;
		}
		data_current_p++;
	}
	if (!file_init_reg_num)
	{
		printk(KERN_ERR"%s,file init reg number is zero\n",__func__);
		rc = -1;
	}
	else
	{
		//init setting
		sp0a20_i2c_write_table(s_ctrl, file_init_reg, file_init_reg_num);
	}
	printk(KERN_DEBUG"%s,%d\n",__func__,__LINE__);
end:
	if (data_buf)
		kfree(data_buf); //free data buf
	if (file_init_reg)
		kfree(file_init_reg); //free init reg buf
	return rc;
}
#endif //FILE_INIT_REG_DEBUG
static int32_t sp0a20_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
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
			s_ctrl->sensor_i2c_client, sp0a20_recommend_settings,
			ARRAY_SIZE(sp0a20_recommend_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
			
#ifdef FILE_INIT_REG_DEBUG
		sp0a20_read_init_file(s_ctrl);
#endif
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
				s_ctrl->sensor_i2c_client, sp0a20_recommend_settings,
				ARRAY_SIZE(sp0a20_recommend_settings),
				MSM_CAMERA_I2C_BYTE_DATA);
				pr_err("%s:%d res =%d\n sp0a20_recommend_settings ",
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
			s_ctrl->sensor_i2c_client, sp0a20_stop_settings,
			ARRAY_SIZE(sp0a20_stop_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
		break;
	case CFG_SET_START_STREAM:
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client, sp0a20_start_settings,
			ARRAY_SIZE(sp0a20_start_settings),
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
		if (copy_from_user(&sat_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		sp0a20_set_saturation(s_ctrl, sat_lev);
	}
		break;
	case CFG_SET_CONTRAST: 
	{
		int32_t con_lev;
		if (copy_from_user(&con_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		sp0a20_set_contrast(s_ctrl, con_lev);
	}
		break;
	case CFG_SET_SHARPNESS: 
	{
		int32_t shp_lev;
		if (copy_from_user(&shp_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		sp0a20_set_sharpness(s_ctrl, shp_lev);
	}
		break;
	case CFG_SET_ISO: 
	{
		int32_t iso_lev;
		if (copy_from_user(&iso_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		sp0a20_set_iso(s_ctrl, iso_lev);
	}
		break;
	case CFG_SET_EXPOSURE_COMPENSATION: 
	{
		int32_t ec_lev;
		if (copy_from_user(&ec_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		sp0a20_set_exposure_compensation(s_ctrl, ec_lev);
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
		sp0a20_set_effect(s_ctrl, effect_mode);
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
		sp0a20_set_antibanding(s_ctrl, antibanding_mode);
	}
		break;
	case CFG_SET_BESTSHOT_MODE: 
	{
		#if 0
		int32_t bs_mode;
		if (copy_from_user(&bs_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		sp0a20_set_scene_mode(s_ctrl, bs_mode);
		#endif
	}
		break;
	case CFG_SET_WHITE_BALANCE: 
	{
		int32_t wb_mode;
		if (copy_from_user(&wb_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		sp0a20_set_white_balance_mode(s_ctrl, wb_mode);
	}
		break;
	default:
		rc = -EFAULT;
		break;
	}
	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

static int32_t sp0a20_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc = 0;
    uint16_t chipid = 0;

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                    s_ctrl->sensor_i2c_client,
                    s_ctrl->sensordata->slave_info->sensor_id_reg_addr,
                    &chipid, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
            pr_err("%s: %s: read id failed\n", __func__, s_ctrl->sensordata->sensor_name);
            return rc;
    }
    pr_err("%s, read id=0x%x, expected id=0x%x\n",__func__, chipid, s_ctrl->sensordata->slave_info->sensor_id);
    if (chipid != s_ctrl->sensordata->slave_info->sensor_id) {
            pr_err("msm_sensor_match_id chip id doesnot match\n");
            return -ENODEV;
    }

    return rc;
}

static struct msm_sensor_fn_t sp0a20_sensor_func_tbl = {
	.sensor_config = sp0a20_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = sp0a20_sensor_match_id,
};

static struct msm_sensor_ctrl_t sp0a20_s_ctrl = {
	.sensor_i2c_client = &sp0a20_sensor_i2c_client,
	.power_setting_array.power_setting = sp0a20_power_setting,
	.power_setting_array.size = ARRAY_SIZE(sp0a20_power_setting),
	.power_setting_array.power_down_setting = sp0a20_power_down_setting,
	.power_setting_array.size_down = ARRAY_SIZE(sp0a20_power_down_setting),
	.msm_sensor_mutex = &sp0a20_mut,
	.sensor_v4l2_subdev_info = sp0a20_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(sp0a20_subdev_info),
	.func_tbl = &sp0a20_sensor_func_tbl,
};

module_init(sp0a20_init_module);
module_exit(sp0a20_exit_module);
MODULE_DESCRIPTION("SP0A20 VGA YUV sensor driver");
MODULE_LICENSE("GPL v2");

