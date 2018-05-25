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

#define FLASH_NAME "sgm3784"

#define CONFIG_MSMB_CAMERA_DEBUG 
#ifdef CONFIG_MSMB_CAMERA_DEBUG 
#define SGM3784_DBG(fmt, args...) pr_err("[CAM_LED]"fmt, ##args)
#else
#define SGM3784_DBG(fmt, args...)
#endif

static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver sgm3784_i2c_driver;

int msm_flash_sgm3784_led_init(struct msm_led_flash_ctrl_t *fctrl);

static struct msm_camera_i2c_reg_array sgm3784_init_array[] = {
	{0x0f, 0x00}
};

static struct msm_camera_i2c_reg_array sgm3784_off_array[] = {
	{0x0f, 0x00}, //bit0 LED1 enalbe bit1 LED1 enable
};

static struct msm_camera_i2c_reg_array sgm3784_release_array[] = {

};

static struct msm_camera_i2c_reg_array sgm3784_low_array[] = {
	{0x01, 0xe8},
	{0x02, 0xff},
	{0x03, 0x48},
	{0x08, 0x04}, //LED1 torch  current  89.3ma
	{0x0b, 0x04}, //LED2 torch  current  89.3ma
	{0x0f, 0x03}, //bit0 LED1 enalbe bit1 LED1 enable
};

static struct msm_camera_i2c_reg_array sgm3784_high_array[] = {
	{0x01, 0xfb},
	{0x02, 0xcf},
	{0x03, 0x48},
	{0x06, 0x04}, //LED1 FLASH current  738.2ma  ---0x28
	{0x09, 0x04}, //LED2 FLASH current  738.2ma
	{0x0f, 0x03}, //bit0 LED1 enalbe bit1 LED1 enable
};


static const struct of_device_id sgm3784_i2c_trigger_dt_match[] = {
	{.compatible = "sgm3784"},
	{}
};

MODULE_DEVICE_TABLE(of, sgm3784_i2c_trigger_dt_match);
static const struct i2c_device_id sgm3784_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

static void sgm3784_read_id(struct i2c_client *client,struct msm_camera_power_ctrl_t *power_info)
{
	uint16_t device_id = 0;
	int rc = 0;	

	rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_read(
			fctrl.flash_i2c_client,
			(uint16_t)fctrl.flashdata->slave_info->sensor_id_reg_addr,
			&device_id, 
			MSM_CAMERA_I2C_BYTE_DATA);
		
	if( rc < 0 )
	{
		SGM3784_DBG("read 0x%x failed!\n",(uint16_t)fctrl.flashdata->slave_info->sensor_id_reg_addr);
	}
	else
	{
		SGM3784_DBG("%s:read [ID] = 0x%x\n",__func__,device_id);	
	}

}

int msm_flash_sgm3784_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	SGM3784_DBG("%s:%d called\n", __func__, __LINE__);
	
	if (!fctrl || !fctrl->flashdata) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	power_info = &(fctrl->flashdata->power_info);
	

	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
	{
		SGM3784_DBG("%s:%d called SENSOR_GPIO_FL_RESET =%d\n", __func__, __LINE__,power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_RESET]);
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

	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_NOW] == 1)
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_NOW],
			GPIO_OUT_LOW);
	}
	
	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_EN] == 1)
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_EN],
			GPIO_OUT_LOW);
	}

	return rc;
}

int msm_flash_sgm3784_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	SGM3784_DBG("%s:%d called\n", __func__, __LINE__);

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
			GPIO_OUT_LOW);
	}

	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_NOW] == 1)
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_NOW],
			GPIO_OUT_LOW);
	}
	
	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_EN] == 1)
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_EN],
			GPIO_OUT_LOW);
	}

	return rc;
}

int msm_flash_sgm3784_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	SGM3784_DBG("%s:%d called\n", __func__, __LINE__);
	
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

int msm_flash_sgm3784_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	
	SGM3784_DBG("%s:%d called\n", __func__, __LINE__);

	if (!fctrl || !fctrl->flashdata ) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	power_info = &(fctrl->flashdata->power_info);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->low_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_NOW] == 1)
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_NOW],
			GPIO_OUT_HIGH);
	}
	
	return rc;
}

int msm_flash_sgm3784_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	
	SGM3784_DBG("%s:%d called\n", __func__, __LINE__);

	if (!fctrl || !fctrl->flashdata) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	power_info = &(fctrl->flashdata->power_info);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_EN] == 1)
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_EN],
			GPIO_OUT_HIGH);
	}
			
	return rc;
}

static int msm_flash_sgm3784_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0 ;

	SGM3784_DBG("%s entry\n", __func__);
	
	
	if (!id) {
		pr_err("msm_flash_sgm3784_i2c_probe: id is NULL");
		id = sgm3784_i2c_id;
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
	
	SGM3784_DBG("%s fctrl.pinctrl_info.use_pinctrl = %d\n", __func__,fctrl.pinctrl_info.use_pinctrl);
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
	
	sgm3784_read_id(client,power_info);
	
	SGM3784_DBG("%s: GPIOS DOWN!\n",__func__);	
	gpio_set_value_cansleep(power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_RESET], GPIO_OUT_LOW);
	gpio_set_value_cansleep(power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_EN], GPIO_OUT_LOW);
	gpio_set_value_cansleep(power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW], GPIO_OUT_LOW);

	SGM3784_DBG("%s: probe successfully!\n",__func__);
	
	return rc;
}

static int msm_flash_sgm3784_i2c_remove(struct i2c_client *client)
{
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0 ;
	
	SGM3784_DBG("%s entry\n", __func__);
	
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
			devm_pinctrl_put(fctrl.pinctrl_info.pinctrl);
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
	}
	
	return rc;
}


static struct i2c_driver sgm3784_i2c_driver = {
	.id_table = sgm3784_i2c_id,
	.probe  = msm_flash_sgm3784_i2c_probe,
	.remove = msm_flash_sgm3784_i2c_remove,
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = sgm3784_i2c_trigger_dt_match,
	},
};

static int __init msm_flash_sgm3784_init(void)
{
	int32_t res = 0;
	SGM3784_DBG("%s entry\n", __func__);
	res = i2c_add_driver(&sgm3784_i2c_driver);
	pr_err("%s, res:%d\n", __FUNCTION__, res);

	return res;
}

static void __exit msm_flash_sgm3784_exit(void)
{
	SGM3784_DBG("%s entry\n", __func__);
	i2c_del_driver(&sgm3784_i2c_driver);
	return;
}


static struct msm_camera_i2c_client sgm3784_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting sgm3784_init_setting = {
	.reg_setting = sgm3784_init_array,
	.size = ARRAY_SIZE(sgm3784_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sgm3784_off_setting = {
	.reg_setting = sgm3784_off_array,
	.size = ARRAY_SIZE(sgm3784_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sgm3784_release_setting = {
	.reg_setting = sgm3784_release_array,
	.size = ARRAY_SIZE(sgm3784_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sgm3784_low_setting = {
	.reg_setting = sgm3784_low_array,
	.size = ARRAY_SIZE(sgm3784_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sgm3784_high_setting = {
	.reg_setting = sgm3784_high_array,
	.size = ARRAY_SIZE(sgm3784_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t sgm3784_regs = {
	.init_setting = &sgm3784_init_setting,
	.off_setting = &sgm3784_off_setting,
	.low_setting = &sgm3784_low_setting,
	.high_setting = &sgm3784_high_setting,
	.release_setting = &sgm3784_release_setting,
};

static struct msm_flash_fn_t sgm3784_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_sgm3784_led_init,
	.flash_led_release = msm_flash_sgm3784_led_release,
	.flash_led_off = msm_flash_sgm3784_led_off,
	.flash_led_low = msm_flash_sgm3784_led_low,
	.flash_led_high = msm_flash_sgm3784_led_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &sgm3784_i2c_client,
	.reg_setting = &sgm3784_regs,
	.func_tbl = &sgm3784_func_tbl,
};

module_init(msm_flash_sgm3784_init);
module_exit(msm_flash_sgm3784_exit);
MODULE_DESCRIPTION("sgm3784 FLASH");
MODULE_LICENSE("GPL v2");
