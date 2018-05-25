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

#define FLASH_NAME "sy7806"

#define CONFIG_MSMB_CAMERA_DEBUG 
#ifdef CONFIG_MSMB_CAMERA_DEBUG 
#define SY7806_DBG(fmt, args...) pr_err("[CAM_LED]"fmt, ##args)
#else
#define SY7806_DBG(fmt, args...)
#endif

//Flashlight Proc Debug Begin
#define SY7806_FLASH_CURRENT_DEBUG 1

#if SY7806_FLASH_CURRENT_DEBUG
#include <linux/proc_fs.h>
static struct proc_dir_entry *g_sy7806_debug_current_proc = NULL;
#define SY7806_CURRENT_PROC_FILE "sy7806_current"

#endif

#define LCT_FLASHLIGHT_CONTROL_NODE  1   //if enable this macro,please enable SY7806_FLASH_CURRENT_DEBUG too

#if LCT_FLASHLIGHT_CONTROL_NODE
#include <linux/slab.h>

#include <linux/proc_fs.h>
static struct proc_dir_entry *g_lct_flashlight_on_proc = NULL;
#define LCT_FLASHLIGHT_ON_PROC_FILE "torch_on"

static struct proc_dir_entry *g_lct_flashlight_off_proc = NULL;
#define LCT_FLASHLIGHT_OFF_PROC_FILE "torch_off"

#endif
//Flashlight Proc Debug End

static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver sy7806_i2c_driver;

int msm_flash_sy7806_led_init(struct msm_led_flash_ctrl_t *fctrl);

//decrease the flash current to 703mA by zhaohong.chen@tcl.com, 20151222
static struct msm_camera_i2c_reg_array sy7806_init_array[] = {
	//{0x03, 0x3b}, //LED1 FLASH current  703ma
	{0x03, 0x05}, //69.5ma
	{0x04, 0x3b}, //LED2 FLASH current  703ma
	//{0x05, 0x3f}, //LED1 torch  current  89.3ma
	{0x05, 0x06}, //9.4ma
	{0x06, 0x3f}, //LED2 torch  current  89.3ma
	{0x01, 0x03}, //bit0 LED1 enalbe bit1 LED1 enable
	{0x08, 0x1f}, //change timeout to 400ms
};

static struct msm_camera_i2c_reg_array sy7806_off_array[] = {
	{0x01, 0x00},
};

static struct msm_camera_i2c_reg_array sy7806_release_array[] = {
	{0x01, 0x00},
};

static struct msm_camera_i2c_reg_array sy7806_low_array[] = {
	{0x01, 0x0b}, //bit2 bit3 torch mode 10 ,flash mode 11 ,standby 00
};

static struct msm_camera_i2c_reg_array sy7806_high_array[] = {
	{0x01, 0x0f}, //bit2 bit3 torch mode 10 ,flash mode 11 ,standby 00
};


static const struct of_device_id sy7806_i2c_trigger_dt_match[] = {
	{.compatible = "sy7806"},
	{}
};

MODULE_DEVICE_TABLE(of, sy7806_i2c_trigger_dt_match);
static const struct i2c_device_id sy7806_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

static void sy7806_read_id(struct i2c_client *client,struct msm_camera_power_ctrl_t *power_info)
{
	uint16_t device_id = 0;
	int rc = 0;
	
	gpio_set_value_cansleep(power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_RESET], GPIO_OUT_HIGH);

	rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_read(
			fctrl.flash_i2c_client,
			(uint16_t)fctrl.flashdata->slave_info->sensor_id_reg_addr,
			&device_id, 
			MSM_CAMERA_I2C_BYTE_DATA);
		
	if( rc < 0 )
	{
		SY7806_DBG("read 0x%x failed!\n",(uint16_t)fctrl.flashdata->slave_info->sensor_id_reg_addr);
	}
	else
	{
		SY7806_DBG("%s:read [ID] = 0x%x\n",__func__,device_id);	
	}

}

int msm_flash_sy7806_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	SY7806_DBG("%s:%d called\n", __func__, __LINE__);
	
	if (!fctrl || !fctrl->flashdata) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	power_info = &(fctrl->flashdata->power_info);
	
	//SY7806_DBG("%s:%d called SENSOR_GPIO_FL_RESET =%d\n", __func__, __LINE__,power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_RESET]);

	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_HIGH);
	}

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->init_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	
	return rc;
}

int msm_flash_sy7806_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	SY7806_DBG("%s:%d called\n", __func__, __LINE__);

	if (!fctrl || !fctrl->flashdata) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	
	power_info = &(fctrl->flashdata->power_info);

	/*
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->release_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	*/
	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_LOW);
	}
	
	return rc;
}

int msm_flash_sy7806_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	SY7806_DBG("%s:%d called\n", __func__, __LINE__);
	
	if (!fctrl || !fctrl->flashdata) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	
	power_info = &(fctrl->flashdata->power_info);

	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_HIGH);
	}
	
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	
	return rc;
}

uint8_t fl_pos = -1;

int msm_flash_sy7806_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	
	if (!fctrl || !fctrl->flashdata || (fl_pos < 0)) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	SY7806_DBG("%s:%d led=%d\n", __func__, __LINE__,fl_pos);
	
	power_info = &(fctrl->flashdata->power_info);

	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_HIGH);
	}
	
	sy7806_low_array[0].reg_data = fl_pos? 0x09:0x0a; //Front LED1 : Back LED2

	//SY7806_DBG("%s:%d enable_bit=%x\n", __func__, __LINE__,sy7806_low_array[0].reg_data);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->low_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	
	return rc;
}

int msm_flash_sy7806_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	
	if (!fctrl || !fctrl->flashdata || (fl_pos < 0)) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	
	SY7806_DBG("%s:%d led=%d\n", __func__, __LINE__,fl_pos);
	
	power_info = &(fctrl->flashdata->power_info);

	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_HIGH);
	}

	sy7806_high_array[0].reg_data = fl_pos? 0x0d:0x0e; //Front LED1 : Back LED2
    
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
			
	return rc;
}
#if SY7806_FLASH_CURRENT_DEBUG
struct msm_camera_i2c_reg_array sy7806_current_proc_init_array[] = {
	{0x03, 0x3f}, //LED1 FLASH current  729ma
	{0x04, 0x3f}, //LED2 FLASH current  729ma
	{0x05, 0x3f}, //LED1 torch  current  89.3ma
	{0x06, 0x3f}, //LED2 torch  current  89.3ma
	{0x01, 0x03},//bit0 LED1 enalbe bit1 LED1 enable
	};

static struct msm_camera_i2c_reg_setting sy7806_current_proc_init_setting = {
	.reg_setting = sy7806_current_proc_init_array,
	.size = ARRAY_SIZE(sy7806_current_proc_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_array sy7806_current_proc_off_array[] = {
	{0x01, 0x00},
};

static struct msm_camera_i2c_reg_setting sy7806_current_proc_off_setting = {
	.reg_setting = sy7806_current_proc_off_array,
	.size = ARRAY_SIZE(sy7806_current_proc_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_array sy7806_current_proc_torch_array[] = {
	{0x01, 0x0b}, //bit2 bit3 torch mode 10 ,flash mode 11 ,standby 00
};


static struct msm_camera_i2c_reg_setting sy7806_current_proc_torch_setting = {
	.reg_setting = sy7806_current_proc_torch_array,
	.size = ARRAY_SIZE(sy7806_current_proc_torch_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_array sy7806_current_proc_flash_array[] = {
	{0x1, 0x0f}, //bit2 bit3 torch mode 10 ,flash mode 11 ,standby 00
};

static struct msm_camera_i2c_reg_setting sy7806_current_proc_flash_setting = {
	.reg_setting = sy7806_current_proc_flash_array,
	.size = ARRAY_SIZE(sy7806_current_proc_flash_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};



//flash brightness:(Brightness Code \A1\C1 11.725 mA) + 10.9 mA  Brightness Code 7bit 
//torch brightness: (Brightness Code \A1\C1 1.4 mA) + 0.977 mA  Brightness Code 7bit
//sigle led max current: flash 749ma  torch:89.3ma
//dual led max current: flash:1.5A  torch:179ma
//usage: echo "50:100:1" > proc/sy7806_current
static ssize_t sy7806_current_proc_write(struct file *file, const char __user *buf, size_t size, loff_t *ppos)
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

	 printk("sy7806_current_proc_write tmp_data = %s\n",tmp_data);
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
			printk("sy7806_current_proc_write invalid param\n");
			return size;
		}
		flash_mode++;
		
	 }
	 if(led1 != NULL)
	 {
	 	led1_current =  simple_strtoul(led1, NULL, 10);
	 	printk("sy7806_current_proc_write led1 = %s,led1_current=%d\n",led1,led1_current);
	 }
	 if(led2 != NULL)
	 {
	  	led2_current = simple_strtoul(led2, NULL, 10);
	  	printk("sy7806_current_proc_write led2 = %s,led2_current=%d\n",led2,led2_current);
	 }
	 if(flash_mode != NULL)
	 {
	 	mode = simple_strtoul(flash_mode, NULL, 10);
	  	printk("sy7806_current_proc_write flash_mode = %s,mode=%d\n",flash_mode,mode);
	 }
	
	if(mode > 2)
	{
		printk("sy7806_current_proc_write invalid flash mode\n");
		return size;
	}

	if((led1_current + led2_current) > 127)
	{
		printk("sy7806_current_proc_write current too large\n");
		return size;
	}

	power_info = &fctrl.flashdata->power_info;
	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
	{
		printk("sy7806_current_proc_write power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_RESET]=%d\n",power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_RESET]);
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_HIGH);
	}

	
	if((led1_current == 0 ) && (led2_current != 0)) //only enalbe led2
	{
		sy7806_current_proc_flash_array[0].reg_data = flash_enable&0xFE;
		sy7806_current_proc_torch_array[0].reg_data = torch_enable&0xFE;
		
	}
	else if((led1_current != 0 ) && (led2_current == 0))//only enalbe led1
	{
		sy7806_current_proc_flash_array[0].reg_data = flash_enable&0xFD;
		sy7806_current_proc_torch_array[0].reg_data = torch_enable&0xFD;
	}
	else //both led1 and led2 enable
	{
		sy7806_current_proc_flash_array[0].reg_data = flash_enable&0x0F;
		sy7806_current_proc_torch_array[0].reg_data = torch_enable&0x0F;
	}

	printk("sy7806_current_proc_write sy7806_current_proc_flash_array[0].reg_data=%x,sy7806_current_proc_torch_array[0].reg_data=%x\n",sy7806_current_proc_flash_array[0].reg_data,sy7806_current_proc_torch_array[0].reg_data);

	sy7806_current_proc_init_array[0].reg_data = led1_current; //LED1 FLASH current  729ma
	sy7806_current_proc_init_array[1].reg_data = led2_current;//LED2 FLASH current  729ma
	sy7806_current_proc_init_array[2].reg_data = led1_current;//LED1 torch  current  89.3ma
	sy7806_current_proc_init_array[3].reg_data = led2_current;//LED2 torch  current  89.3ma

	if (fctrl.flash_i2c_client && fctrl.reg_setting) {
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl.flash_i2c_client,
			&sy7806_current_proc_init_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	
	if(mode == 1) //torch
	{
		if (fctrl.flash_i2c_client && fctrl.reg_setting) {
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl.flash_i2c_client,
			&sy7806_current_proc_torch_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
		}
	}
	else if(mode == 2) //flash
	{
		if (fctrl.flash_i2c_client && fctrl.reg_setting) {
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl.flash_i2c_client,
			&sy7806_current_proc_flash_setting);
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
				&sy7806_current_proc_off_setting);
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

static ssize_t sy7806_current_proc_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	int cnt= 0;
    char *page = NULL;
	page = kzalloc(128, GFP_KERNEL);
	cnt = sprintf(page, "%s", "usage: echo \"50:100:1\" > proc/sy7806_current\n");   
	cnt = simple_read_from_buffer(buf, size, ppos, page, cnt);
	kfree(page);
	return cnt;
}

static const struct file_operations sy7806_current_proc_fops = {
	.read		= sy7806_current_proc_read,
	.write		= sy7806_current_proc_write,
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
		&sy7806_current_proc_init_setting);
	if (rc < 0){
			pr_err("%s:%d failed\n", __func__, __LINE__);
			goto err;
		}
	}

	if (fctrl.flash_i2c_client && fctrl.reg_setting) {
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl.flash_i2c_client,
			&sy7806_current_proc_torch_setting);
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
				&sy7806_current_proc_off_setting);
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

static void sy7806_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	if (value > LED_OFF && value <= LED_FULL)
	{
		if (fctrl.func_tbl->flash_led_init)
			fctrl.func_tbl->flash_led_init(&fctrl);
		if (value > LED_HALF) 
		{
			if (fctrl.func_tbl->flash_led_high)
				fctrl.func_tbl->flash_led_high(&fctrl);
		}
		else if (value > LED_OFF) 
		{
			if (fctrl.func_tbl->flash_led_low)
				fctrl.func_tbl->flash_led_low(&fctrl);
		}
	}
	else 
	{
		if (fctrl.func_tbl->flash_led_off)
			fctrl.func_tbl->flash_led_off(&fctrl);
		if (fctrl.func_tbl->flash_led_release)
			fctrl.func_tbl->flash_led_release(&fctrl);
	}

}
static void back_sy7806_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{	
	int8_t temp = fl_pos;
	fl_pos = 0;
	sy7806_brightness_set(led_cdev,value);
	fl_pos = temp;
};

static void front_sy7806_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	int8_t temp = fl_pos;
	fl_pos = 1;
	sy7806_brightness_set(led_cdev,value);
	fl_pos = temp;
};

static struct led_classdev back_sy7806 = {
	.name			= "torch-light0",
	.brightness_set	= back_sy7806_brightness_set,
	.brightness		= LED_OFF,
};

static struct led_classdev front_sy7806 = {
	.name			= "torch-light1",
	.brightness_set	= front_sy7806_brightness_set,
	.brightness		= LED_OFF,
};


static int32_t sy7806_torch_create_classdev(struct device *dev ,
				void *data)
{
	int rc;
	back_sy7806_brightness_set(&back_sy7806, LED_OFF);
	front_sy7806_brightness_set(&front_sy7806, LED_OFF);
	rc = led_classdev_register(dev, &back_sy7806);
	if (rc) {
		pr_err("Failed to register back_sy7806 . rc = %d\n", rc);
		return rc;
	}
	rc = led_classdev_register(dev, &front_sy7806);
	if (rc) {
		pr_err("Failed to register front_sy7806 . rc = %d\n", rc);
		return rc;
	}

	return 0;
};

static int msm_flash_sy7806_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0 ;
	
	SY7806_DBG("%s entry\n", __func__);
	
	if (!id) {
		pr_err("msm_flash_sy7806_i2c_probe: id is NULL");
		id = sy7806_i2c_id;
	}
	
	rc = msm_flash_i2c_probe(client, id);
	if(rc < 0){
		pr_err("%s: msm_flash_i2c_probe failed!\n", __func__);
		return rc;
	}

	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;
	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	rc = sy7806_torch_create_classdev(&(client->dev),NULL);
	if(rc < 0){
		pr_err("%s: sy7806_torch_create_classdev failed!\n", __func__);
		return rc;
	}
	
	SY7806_DBG("%s fctrl.pinctrl_info.use_pinctrl = %d\n", __func__,fctrl.pinctrl_info.use_pinctrl);
	if (fctrl.pinctrl_info.use_pinctrl == true) {
		pr_err("%s:%d PC:: flash pins setting to active state",
				__func__, __LINE__);
		rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
				fctrl.pinctrl_info.gpio_state_active);
		if (rc < 0)
		{
			devm_pinctrl_put(fctrl.pinctrl_info.pinctrl);
			pr_err("%s:%d cannot set pin to active state",
					__func__, __LINE__);
		}
	}

	msleep(20);

	sy7806_read_id(client,power_info);
	
	SY7806_DBG("%s: GPIOS DOWN!\n",__func__);	
	gpio_set_value_cansleep(power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_RESET], GPIO_OUT_LOW);
	gpio_set_value_cansleep(power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_EN], GPIO_OUT_LOW);
	gpio_set_value_cansleep(power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW], GPIO_OUT_LOW);
	gpio_set_value_cansleep(power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_CUSTOM1], GPIO_OUT_LOW);

	#if SY7806_FLASH_CURRENT_DEBUG
	g_sy7806_debug_current_proc = proc_create_data(SY7806_CURRENT_PROC_FILE, 0660, NULL, &sy7806_current_proc_fops, NULL);
	if (IS_ERR_OR_NULL(g_sy7806_debug_current_proc))
	{
		printk("create_proc_entry g_sy7806_debug_current_proc failed\n");
	}
	else
	{
		printk("create_proc_entry g_sy7806_debug_current_proc success\n");
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
		SY7806_DBG("create_proc_entry g_lct_flashlight_off_proc failed\n");
	}
	else
	{
		SY7806_DBG("create_proc_entry g_lct_flashlight_off_proc success\n");
	}
	#endif
	
	return rc;
}

static int msm_flash_sy7806_i2c_remove(struct i2c_client *client)
{
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0 ;
	SY7806_DBG("%s entry\n", __func__);
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
		if (rc < 0)
		{
			devm_pinctrl_put(fctrl.pinctrl_info.pinctrl);
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
		}
	}
	return rc;
}


static struct i2c_driver sy7806_i2c_driver = {
	.id_table = sy7806_i2c_id,
	.probe  = msm_flash_sy7806_i2c_probe,
	.remove = msm_flash_sy7806_i2c_remove,
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = sy7806_i2c_trigger_dt_match,
	},
};

static int __init msm_flash_sy7806_init(void)
{
	int32_t res = 0;
	SY7806_DBG("%s entry\n", __func__);
	res = i2c_add_driver(&sy7806_i2c_driver);
	pr_err("%s, res:%d\n", __FUNCTION__, res);

	return res;
}

static void __exit msm_flash_sy7806_exit(void)
{
	SY7806_DBG("%s entry\n", __func__);
	i2c_del_driver(&sy7806_i2c_driver);
	return;
}


static struct msm_camera_i2c_client sy7806_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting sy7806_init_setting = {
	.reg_setting = sy7806_init_array,
	.size = ARRAY_SIZE(sy7806_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7806_off_setting = {
	.reg_setting = sy7806_off_array,
	.size = ARRAY_SIZE(sy7806_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7806_release_setting = {
	.reg_setting = sy7806_release_array,
	.size = ARRAY_SIZE(sy7806_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7806_low_setting = {
	.reg_setting = sy7806_low_array,
	.size = ARRAY_SIZE(sy7806_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7806_high_setting = {
	.reg_setting = sy7806_high_array,
	.size = ARRAY_SIZE(sy7806_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t sy7806_regs = {
	.init_setting = &sy7806_init_setting,
	.off_setting = &sy7806_off_setting,
	.low_setting = &sy7806_low_setting,
	.high_setting = &sy7806_high_setting,
	.release_setting = &sy7806_release_setting,
};

static struct msm_flash_fn_t sy7806_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_sy7806_led_init,
	.flash_led_release = msm_flash_sy7806_led_release,
	.flash_led_off = msm_flash_sy7806_led_off,
	.flash_led_low = msm_flash_sy7806_led_low,
	.flash_led_high = msm_flash_sy7806_led_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &sy7806_i2c_client,
	.reg_setting = &sy7806_regs,
	.func_tbl = &sy7806_func_tbl,
};

module_init(msm_flash_sy7806_init);
module_exit(msm_flash_sy7806_exit);
MODULE_DESCRIPTION("sy7806 FLASH");
MODULE_LICENSE("GPL v2");
