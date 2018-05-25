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
#include <linux/module.h>
#include <linux/export.h>
#include "msm_camera_io_util.h"
#include "msm_led_flash.h"

#define FLASH_NAME "ti,lm3643"

#define CONFIG_MSMB_CAMERA_DEBUG 
#ifdef CONFIG_MSMB_CAMERA_DEBUG 
#define LM3643_DBG(fmt, args...) pr_err(fmt, ##args)
#else
#define LM3643_DBG(fmt, args...)
#endif

#define LM3643_FLASH_CURRENT_DEBUG 1

#if LM3643_FLASH_CURRENT_DEBUG
#include <linux/proc_fs.h>
static struct proc_dir_entry *g_lm3643_debug_current_proc = NULL;
#define LM3643_CURRENT_PROC_FILE "lm3643_current"

#endif

int lct_camera_id;

#define LCT_FLASHLIGHT_CONTROL_NODE  1   //if enable this macro,please enable LM3643_FLASH_CURRENT_DEBUG too

#if LCT_FLASHLIGHT_CONTROL_NODE
#include <linux/slab.h>

#include <linux/proc_fs.h>
static struct proc_dir_entry *g_lct_flashlight_on_proc = NULL;
#define LCT_FLASHLIGHT_ON_PROC_FILE "torch_on"

static struct proc_dir_entry *g_lct_flashlight_off_proc = NULL;
#define LCT_FLASHLIGHT_OFF_PROC_FILE "torch_off"

#endif


static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver lm3643_i2c_driver;

int msm_flash_lm3643_led_init(struct msm_led_flash_ctrl_t *fctrl);

static struct msm_camera_i2c_reg_array lm3643_init_array[] = {
	{0x03, 0x3f}, //LED1 FLASH current  729ma
	{0x04, 0x3f}, //LED2 FLASH current  729ma
	{0x05, 0x3f}, //LED1 torch  current  89.3ma
	{0x06, 0x3f}, //LED2 torch  current  89.3ma
	{0x01, 0x03},//bit0 LED1 enalbe bit1 LED1 enable
	{0x08, 0x1f},//change timeout to 400ms
};

static struct msm_camera_i2c_reg_array lm3643_off_array[] = {
	{0x01, 0x00},
};

static struct msm_camera_i2c_reg_array lm3643_release_array[] = {
	{0x01, 0x00},
};

static struct msm_camera_i2c_reg_array lm3643_low_array[] = {
	{0x05, 0x3f}, 
	{0x06, 0x3f}, 
	{0x01, 0x0b}, //bit2 bit3 torch mode 10 ,flash mode 11 ,standby 00
	{0x08, 0x1f},//change timeout to 400ms
};

static struct msm_camera_i2c_reg_array lm3643_high_array[] = {
	{0x03, 0x3f}, 
	{0x04, 0x3f}, 
	{0x1, 0x0f}, //bit2 bit3 torch mode 10 ,flash mode 11 ,standby 00
	{0x08, 0x1f},//change timeout to 400ms
};


static const struct of_device_id lm3643_i2c_trigger_dt_match[] = {
	{.compatible = "ti,lm3643"},
	{}
};

MODULE_DEVICE_TABLE(of, lm3643_i2c_trigger_dt_match);
static const struct i2c_device_id lm3643_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

static void msm_led_torch_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	msm_flash_lm3643_led_init(&fctrl);
	if (value > LED_OFF) {
		if(fctrl.func_tbl->flash_led_low)
			fctrl.func_tbl->flash_led_low(&fctrl);
	} else {
		if(fctrl.func_tbl->flash_led_off)
			fctrl.func_tbl->flash_led_off(&fctrl);
	}
};

static struct led_classdev msm_torch_led = {
	.name			= "torch-light",
	.brightness_set	= msm_led_torch_brightness_set,
	.brightness		= LED_OFF,
};

static int32_t msm_lm3643_torch_create_classdev(struct device *dev ,
				void *data)
{
	int rc;
	msm_led_torch_brightness_set(&msm_torch_led, LED_OFF);
	rc = led_classdev_register(dev, &msm_torch_led);
	if (rc) {
		pr_err("Failed to register led dev. rc = %d\n", rc);
		return rc;
	}

	return 0;
};

int msm_flash_lm3643_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	LM3643_DBG("%s:%d called SENSOR_GPIO_FL_RESET =%d\n", __func__, __LINE__,power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_RESET]);
	#if 1
	#if 0
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
	#endif
	
	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
	{
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_HIGH);
	}
	#endif

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->init_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
	{
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_LOW);
	}
	return rc;
}

int msm_flash_lm3643_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	LM3643_DBG("%s:%d called\n", __func__, __LINE__);
	if (!fctrl || !fctrl->flashdata) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->release_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	#if 1
	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
	{
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_LOW);
	}
	#endif

	return 0;
}

int msm_flash_lm3643_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	LM3643_DBG("%s:%d called\n", __func__, __LINE__);

	if (!fctrl || !fctrl->flashdata) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;


	
#if 1
	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
	{
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_HIGH);
	}
#endif

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	#if 1
	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
	{
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET], GPIO_OUT_LOW);
	}
	#endif


	return rc;
}


int msm_flash_lm3643_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	//int i = 0;
	//int current_op = 0;
	//uint16_t enable_bit = 0x0b ;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	LM3643_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	#if 1
	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
	{
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_HIGH);
	}
	#endif
	#if 0
	for ( i = 0; i < fctrl->torch_num_sources; i++)
	{ 
		LM3643_DBG("%s:%d fctrl->torch_op_current[%d]=%d\n", __func__, __LINE__,i,fctrl->torch_op_current[i]);
		  
	     lm3643_low_array[i].reg_data = fctrl->torch_op_current[i];
		
		current_op += lm3643_low_array[i].reg_data ;
		if(fctrl->torch_op_current[i] <= 0)
		{
			if(i == 0)
			{
				enable_bit = enable_bit&0x0e;
			}
			else
			{
				enable_bit = enable_bit&0x0d;
			}
		}
	}

	lm3643_low_array[2].reg_data = enable_bit;
	
	
	if(current_op == 0) //workround for flashlight and midtest
	{
		lm3643_low_array[0].reg_data = 0x1f;
		lm3643_low_array[1].reg_data = 0x1f;
		lm3643_low_array[2].reg_data = 0x0b;
	}
	#endif
	LM3643_DBG("%s:%d lct_camera_id=%x\n", __func__, __LINE__,lct_camera_id);
	#if 1
	if(lct_camera_id == 2) //front led1
	{
		lm3643_low_array[0].reg_data = 0x7f;
		lm3643_low_array[2].reg_data = 0x09;
	}
	else if(lct_camera_id == 1) //back led2
	{
		lm3643_low_array[1].reg_data = 0x7f;
		lm3643_low_array[2].reg_data = 0xa;
	}
	#endif
	LM3643_DBG("%s:%d enable_bit=%x\n", __func__, __LINE__,lm3643_low_array[2].reg_data);
	#if 0
	if (current_op > 0x7f)
	{
		if((enable_bit&0x1) == 0)
		{
			lm3643_low_array[1].reg_data = 0x7f;
		}
		else if((enable_bit&0x2) == 0)
		{
			lm3643_low_array[0].reg_data = 0x7f;
		}
		else
		{
			lm3643_low_array[0].reg_data = 0x3f;
			lm3643_low_array[1].reg_data = 0x3f;
		}
	}
	#endif

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->low_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}

int msm_flash_lm3643_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	//int i = 0;
	//int current_op = 0;
	//uint16_t enable_bit = 0x0f ;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	LM3643_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;

	power_info = &flashdata->power_info;

	#if 1
	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
	{
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_HIGH);
	}
	#endif

	#if 0
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);
	#endif
	#if 0
	for ( i = 0; i < fctrl->flash_num_sources; i++)
	{ 
		
		LM3643_DBG("%s:%d fctrl->flash_op_current[%d]=%d\n", __func__, __LINE__,i,fctrl->flash_op_current[i]);
	 	
		lm3643_high_array[i].reg_data = fctrl->flash_op_current[i];
	
		current_op += lm3643_high_array[i].reg_data ;
		if(fctrl->flash_op_current[i] <= 0)
		{
			if(i == 0)
			{
				enable_bit = enable_bit&0x0e;
			}
			else
			{
				enable_bit = enable_bit&0x0d;
			}
		}
	}
	lm3643_high_array[2].reg_data = enable_bit;
	LM3643_DBG("%s:%d enable_bit=%x\n", __func__, __LINE__,enable_bit);
	if (current_op > 0x7f)
	{
		if((enable_bit&0x1) == 0)
		{
			lm3643_high_array[1].reg_data = 0x7f;
		}
		else if((enable_bit&0x2) == 0)
		{
			lm3643_high_array[0].reg_data = 0x7f;
		}
		else
		{
			lm3643_high_array[0].reg_data = 0x3f;
			lm3643_high_array[1].reg_data = 0x3f;
		}
	}
	#endif
	LM3643_DBG("%s:%d lct_camera_id=%x\n", __func__, __LINE__,lct_camera_id);
	#if 1
	if(lct_camera_id == 2) //front led1
	{
		lm3643_high_array[0].reg_data = 0x7f;
		lm3643_high_array[2].reg_data = 0xd;
	}
	else if(lct_camera_id == 1) //back led2
	{
		lm3643_high_array[1].reg_data = 0x7f;
		lm3643_high_array[2].reg_data = 0xe;
	}
	#endif
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}
#if LM3643_FLASH_CURRENT_DEBUG
struct msm_camera_i2c_reg_array lm3643_current_proc_init_array[] = {
	{0x03, 0x3f}, //LED1 FLASH current  729ma
	{0x04, 0x3f}, //LED2 FLASH current  729ma
	{0x05, 0x3f}, //LED1 torch  current  89.3ma
	{0x06, 0x3f}, //LED2 torch  current  89.3ma
	{0x01, 0x03},//bit0 LED1 enalbe bit1 LED1 enable
	};

static struct msm_camera_i2c_reg_setting lm3643_current_proc_init_setting = {
	.reg_setting = lm3643_current_proc_init_array,
	.size = ARRAY_SIZE(lm3643_current_proc_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_array lm3643_current_proc_off_array[] = {
	{0x01, 0x00},
};

static struct msm_camera_i2c_reg_setting lm3643_current_proc_off_setting = {
	.reg_setting = lm3643_current_proc_off_array,
	.size = ARRAY_SIZE(lm3643_current_proc_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_array lm3643_current_proc_torch_array[] = {
	{0x01, 0x0b}, //bit2 bit3 torch mode 10 ,flash mode 11 ,standby 00
};


static struct msm_camera_i2c_reg_setting lm3643_current_proc_torch_setting = {
	.reg_setting = lm3643_current_proc_torch_array,
	.size = ARRAY_SIZE(lm3643_current_proc_torch_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_array lm3643_current_proc_flash_array[] = {
	{0x1, 0x0f}, //bit2 bit3 torch mode 10 ,flash mode 11 ,standby 00
};

static struct msm_camera_i2c_reg_setting lm3643_current_proc_flash_setting = {
	.reg_setting = lm3643_current_proc_flash_array,
	.size = ARRAY_SIZE(lm3643_current_proc_flash_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};



//flash brightness:(Brightness Code ¡Á 11.725 mA) + 10.9 mA  Brightness Code 7bit 
//torch brightness: (Brightness Code ¡Á 1.4 mA) + 0.977 mA  Brightness Code 7bit
//sigle led max current: flash 749ma  torch:89.3ma
//dual led max current: flash:1.5A  torch:179ma
//usage: echo "50:100:1" > proc/lm3643_current
static ssize_t lm3643_current_proc_write(struct file *file, const char __user *buf, size_t size, loff_t *ppos)
{
	char data[64] = {0};
	char tmp_data[64] = {0};
	char *led1 = NULL;
	char *led2 = NULL;
	char *flash_mode = NULL;
	int led1_current = 0;
	int led2_current = 0;
	int mode = 0; // 1 torch 2 flash
	int rc;
	uint16_t flash_enable = 0x0f;
	uint16_t torch_enable = 0x0b;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	
	if(copy_from_user(tmp_data, buf, size))
    {
        printk("copy_from_user() fail.\n");
        return -EFAULT;
    }

	 printk("lm3643_current_proc_write tmp_data = %s\n",tmp_data);
	 strcpy(data,tmp_data);
	 led2 = strchr(tmp_data,':');
	 if(led2 != NULL)
	 {
	 	data[led2-tmp_data] = '\0';
		led1 = data;
		led2++;
		flash_mode = strchr(led2,':');
		if(flash_mode == NULL)
		{
			printk("lm3643_current_proc_write invalid param\n");
			return size;
		}
		flash_mode++;
		
	 }
	 if(led1 != NULL)
	 {
	 	led1_current =  simple_strtoul(led1, NULL, 10);
	 	printk("lm3643_current_proc_write led1 = %s,led1_current=%d\n",led1,led1_current);
	 }
	 if(led2 != NULL)
	 {
	  	led2_current = simple_strtoul(led2, NULL, 10);
	  	printk("lm3643_current_proc_write led2 = %s,led2_current=%d\n",led2,led2_current);
	 }
	 if(flash_mode != NULL)
	 {
	 	mode = simple_strtoul(flash_mode, NULL, 10);
	  	printk("lm3643_current_proc_write flash_mode = %s,mode=%d\n",flash_mode,mode);
	 }
	
	if(mode > 2)
	{
		printk("lm3643_current_proc_write invalid flash mode\n");
		return size;
	}

	if((led1_current + led2_current) > 127)
	{
		printk("lm3643_current_proc_write current too large\n");
		return size;
	}

	power_info = &fctrl.flashdata->power_info;
	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
	{
		printk("lm3643_current_proc_write power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_RESET]=%d\n",power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_RESET]);
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_HIGH);
	}

	
	if((led1_current == 0 ) && (led2_current != 0)) //only enalbe led2
	{
		lm3643_current_proc_flash_array[0].reg_data = flash_enable&0xFE;
		lm3643_current_proc_torch_array[0].reg_data = torch_enable&0xFE;
		
	}
	else if((led1_current != 0 ) && (led2_current == 0))//only enalbe led1
	{
		lm3643_current_proc_flash_array[0].reg_data = flash_enable&0xFD;
		lm3643_current_proc_torch_array[0].reg_data = torch_enable&0xFD;
	}
	else //both led1 and led2 enable
	{
		lm3643_current_proc_flash_array[0].reg_data = flash_enable&0x0F;
		lm3643_current_proc_torch_array[0].reg_data = torch_enable&0x0F;
	}

	printk("lm3643_current_proc_write lm3643_current_proc_flash_array[0].reg_data=%x,lm3643_current_proc_torch_array[0].reg_data=%x\n",lm3643_current_proc_flash_array[0].reg_data,lm3643_current_proc_torch_array[0].reg_data);

	lm3643_current_proc_init_array[0].reg_data = led1_current; //LED1 FLASH current  729ma
	lm3643_current_proc_init_array[1].reg_data = led2_current;//LED2 FLASH current  729ma
	lm3643_current_proc_init_array[2].reg_data = led1_current;//LED1 torch  current  89.3ma
	lm3643_current_proc_init_array[3].reg_data = led2_current;//LED2 torch  current  89.3ma

	if (fctrl.flash_i2c_client && fctrl.reg_setting) {
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl.flash_i2c_client,
			&lm3643_current_proc_init_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	
	if(mode == 1) //torch
	{
		if (fctrl.flash_i2c_client && fctrl.reg_setting) {
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl.flash_i2c_client,
			&lm3643_current_proc_torch_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
		}
	}
	else if(mode == 2) //flash
	{
		if (fctrl.flash_i2c_client && fctrl.reg_setting) {
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl.flash_i2c_client,
			&lm3643_current_proc_flash_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
		}
	}
    else
    {
    	
    }

	if((led1 == 0) && (led2 == 0))
	{		
	  	if (fctrl.flash_i2c_client && fctrl.reg_setting) {
			rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write_table(
				fctrl.flash_i2c_client,
				&lm3643_current_proc_off_setting);
			if (rc < 0)
				pr_err("%s:%d failed\n", __func__, __LINE__);
		}
		if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
		{
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_RESET],
				GPIO_OUT_LOW);
		}
	}
	return size;
}

static ssize_t lm3643_current_proc_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	int cnt= 0;
    char *page = NULL;
	page = kzalloc(128, GFP_KERNEL);
	cnt = sprintf(page, "%s", "usage: echo \"50:100:1\" > proc/lm3643_current\n");   
	cnt = simple_read_from_buffer(buf, size, ppos, page, cnt);
	kfree(page);
	return cnt;
}

static const struct file_operations lm3643_current_proc_fops = {
	.read		= lm3643_current_proc_read,
	.write		= lm3643_current_proc_write,
};
#endif


#if LCT_FLASHLIGHT_CONTROL_NODE
static ssize_t lct_flashlight_on_proc_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	int cnt= 0;
	char *page = NULL;
	int rc = 0;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	
	printk("torch on\n");


	power_info = &fctrl.flashdata->power_info;

	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_LOW);
	}
	msleep(10);

	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_HIGH);
	}

	if (fctrl.flash_i2c_client && fctrl.reg_setting) {
	rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write_table(
		fctrl.flash_i2c_client,
		&lm3643_current_proc_init_setting);
	if (rc < 0){
			pr_err("%s:%d failed\n", __func__, __LINE__);
			goto err;
		}
	}

	if (fctrl.flash_i2c_client && fctrl.reg_setting) {
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl.flash_i2c_client,
			&lm3643_current_proc_torch_setting);
		if (rc < 0){
				pr_err("%s:%d failed\n", __func__, __LINE__);
				goto err;
			}
		}
	err:
	page = kzalloc(64, GFP_KERNEL);
	cnt = sprintf(page, "%s", "torch on\n");   
	cnt = simple_read_from_buffer(buf, size, ppos, page, cnt);
	kfree(page);
	return 0;
}

static const struct file_operations lct_flashlight_on_proc_fops = {
	.read		= lct_flashlight_on_proc_read,
	.write		= NULL,
};

	
static ssize_t lct_flashlight_off_proc_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	int cnt= 0;
	char *page = NULL;
	int rc = 0;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	
	printk("torch off\n");
	
	power_info = &fctrl.flashdata->power_info;

	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_HIGH);
	}

	if (fctrl.flash_i2c_client && fctrl.reg_setting) {
			rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write_table(
				fctrl.flash_i2c_client,
				&lm3643_current_proc_off_setting);
			if (rc < 0){
					pr_err("%s:%d failed\n", __func__, __LINE__);
					goto err;
				}
		}
	err:
	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_LOW);
	}
	page = kzalloc(64, GFP_KERNEL);
	cnt = sprintf(page, "%s", "torch off\n");   
	cnt = simple_read_from_buffer(buf, size, ppos, page, cnt);
	kfree(page);
	return 0;
}


static const struct file_operations lct_flashlight_off_proc_fops = {
	.read		= lct_flashlight_off_proc_read,
	.write		= NULL,
};

#endif


static int msm_flash_lm3643_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0 ;
	LM3643_DBG("%s entry\n", __func__);
	if (!id) {
		pr_err("msm_flash_lm3643_i2c_probe: id is NULL");
		id = lm3643_i2c_id;
	}
	rc = msm_flash_i2c_probe(client, id);

	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}
	LM3643_DBG("%s fctrl.pinctrl_info.use_pinctrl = %d\n", __func__,fctrl.pinctrl_info.use_pinctrl);
	if (fctrl.pinctrl_info.use_pinctrl == true) {
		pr_err("%s:%d PC:: flash pins setting to active state",
				__func__, __LINE__);
		rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
				fctrl.pinctrl_info.gpio_state_active);
		if (rc)
			pr_err("%s:%d cannot set pin to active state",
					__func__, __LINE__);
	}

	if (!rc)
		msm_lm3643_torch_create_classdev(&(client->dev),NULL);

	#if LM3643_FLASH_CURRENT_DEBUG
	g_lm3643_debug_current_proc = proc_create_data(LM3643_CURRENT_PROC_FILE, 0660, NULL, &lm3643_current_proc_fops, NULL);
	if (IS_ERR_OR_NULL(g_lm3643_debug_current_proc))
	{
		printk("create_proc_entry g_lm3643_debug_current_proc failed\n");
	}
	else
	{
		printk("create_proc_entry g_lm3643_debug_current_proc success\n");
	}
	#endif

	
#if LCT_FLASHLIGHT_CONTROL_NODE
		g_lct_flashlight_on_proc = proc_create_data(LCT_FLASHLIGHT_ON_PROC_FILE, 0444, NULL, &lct_flashlight_on_proc_fops, NULL);
		if (IS_ERR_OR_NULL(g_lct_flashlight_on_proc))
		{
			printk("create_proc_entry g_lct_flashlight_on_proc failed\n");
		}
		else
		{
			printk("create_proc_entry g_lct_flashlight_on_proc success\n");
		}
	
		g_lct_flashlight_off_proc = proc_create_data(LCT_FLASHLIGHT_OFF_PROC_FILE, 0444, NULL, &lct_flashlight_off_proc_fops, NULL);
		if (IS_ERR_OR_NULL(g_lct_flashlight_off_proc))
		{
			LM3643_DBG("create_proc_entry g_lct_flashlight_off_proc failed\n");
		}
		else
		{
			LM3643_DBG("create_proc_entry g_lct_flashlight_off_proc success\n");
		}
		
#endif
	return rc;
}

static int msm_flash_lm3643_i2c_remove(struct i2c_client *client)
{
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0 ;
	LM3643_DBG("%s entry\n", __func__);
	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl.pinctrl_info.use_pinctrl == true) {
		rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
				fctrl.pinctrl_info.gpio_state_suspend);
		if (rc)
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
	}
	return rc;
}


static struct i2c_driver lm3643_i2c_driver = {
	.id_table = lm3643_i2c_id,
	.probe  = msm_flash_lm3643_i2c_probe,
	.remove = msm_flash_lm3643_i2c_remove,
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm3643_i2c_trigger_dt_match,
	},
};

static int __init msm_flash_lm3643_init(void)
{
	LM3643_DBG("%s entry\n", __func__);
	return i2c_add_driver(&lm3643_i2c_driver);
}

static void __exit msm_flash_lm3643_exit(void)
{
	LM3643_DBG("%s entry\n", __func__);
	i2c_del_driver(&lm3643_i2c_driver);
	return;
}


static struct msm_camera_i2c_client lm3643_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting lm3643_init_setting = {
	.reg_setting = lm3643_init_array,
	.size = ARRAY_SIZE(lm3643_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3643_off_setting = {
	.reg_setting = lm3643_off_array,
	.size = ARRAY_SIZE(lm3643_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3643_release_setting = {
	.reg_setting = lm3643_release_array,
	.size = ARRAY_SIZE(lm3643_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3643_low_setting = {
	.reg_setting = lm3643_low_array,
	.size = ARRAY_SIZE(lm3643_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3643_high_setting = {
	.reg_setting = lm3643_high_array,
	.size = ARRAY_SIZE(lm3643_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t lm3643_regs = {
	.init_setting = &lm3643_init_setting,
	.off_setting = &lm3643_off_setting,
	.low_setting = &lm3643_low_setting,
	.high_setting = &lm3643_high_setting,
	.release_setting = &lm3643_release_setting,
};

static struct msm_flash_fn_t lm3643_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_lm3643_led_init,
	.flash_led_release = msm_flash_lm3643_led_release,
	.flash_led_off = msm_flash_lm3643_led_off,
	.flash_led_low = msm_flash_lm3643_led_low,
	.flash_led_high = msm_flash_lm3643_led_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &lm3643_i2c_client,
	.reg_setting = &lm3643_regs,
	.func_tbl = &lm3643_func_tbl,
};

module_init(msm_flash_lm3643_init);
module_exit(msm_flash_lm3643_exit);
MODULE_DESCRIPTION("lm3643 FLASH");
MODULE_LICENSE("GPL v2");
