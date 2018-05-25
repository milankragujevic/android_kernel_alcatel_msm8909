////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2014 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (??MStar Confidential Information??) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

/**
 *
 * @file    mstar_drv_qcom.c
 *
 * @brief   This file defines the interface of touch screen
 *
 *
 */
 
/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kobject.h>
#include <asm/irq.h>
#include <asm/io.h>


#include "mstar_drv_platform_interface.h"

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
#include <linux/regulator/consumer.h>
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

#include <linux/of_gpio.h>

/*=============================================================*/
// CONSTANT VALUE DEFINITION
/*=============================================================*/

#define MSG_TP_IC_NAME "msg2xxx" //"msg21xxA" or "msg22xx" or "msg26xxM" /* Please define the mstar touch ic name based on the mutual-capacitive ic or self capacitive ic that you are using */

/*=============================================================*/
// VARIABLE DEFINITION
/*=============================================================*/

struct i2c_client *g_I2cClient = NULL;

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
struct regulator *g_ReguVdd = NULL;
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

/*=============================================================*/
// FUNCTION DEFINITION
/*=============================================================*/

//zxzadd
unsigned global_irq_gpio;
unsigned global_reset_gpio;
struct regulator *TPvdd;
struct regulator *TPvcc_i2c;

struct msg_ts_platform_data {
    u32 panel_minx;
	u32 panel_miny;
	u32 panel_maxx;
	u32 panel_maxy;
	u32 disp_minx;
	u32 disp_miny;
	u32 disp_maxx;
	u32 disp_maxy;
	//const char *name;
	int *keycodes;
	int num_keys;
	int y_offset;
    bool no_force_update;
	bool i2c_pull_up;    
	//int disp_maxx;
	//int disp_maxy;
	int pan_maxx;
	int pan_maxy;	
	unsigned irq_gpio;
	unsigned reset_gpio;
	u32 irq_flags;
	u32 reset_flags;

};
static int tp_msg_parse_dt(struct i2c_client *client )
{

struct msg_ts_platform_data *pdata = NULL;

struct device *dev=&client->dev;
struct device_node *np = dev->of_node;
 u32 temp_val;
     int rc;

    if (client->dev.of_node) {
        pdata = devm_kzalloc(&client->dev,
            sizeof(*pdata),
            GFP_KERNEL);
        if (!pdata) {
            dev_err(&client->dev, "Failed to allocate memory\n");
            return -ENOMEM;
        }


rc = of_property_read_u32(np, "mstar,disp-maxx", &temp_val);

	pdata->reset_gpio = of_get_named_gpio_flags(np,
			"mstar,reset-gpios", 0, &pdata->reset_flags);
	pdata->irq_gpio = of_get_named_gpio_flags(np,
			"mstar,interrupt-gpios", 0, &pdata->irq_flags);

global_irq_gpio=pdata->irq_gpio;
global_reset_gpio=pdata->reset_gpio;
printk("MIKE: ######## %s : irq: %d, reset: %d #######\n",__FUNCTION__, global_irq_gpio, global_reset_gpio);

rc=0;
return rc;
	}else {
        pdata = client->dev.platform_data;
		return rc;
    	}

}

/* probe function is used for matching and initializing input device */
static int touch_driver_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
    const char *vdd_name = "vdd";
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

    DBG("*** %s ***\n", __FUNCTION__);
    
    if (client == NULL)
    {
        DBG("i2c client is NULL\n");
        return -1;
    }
    g_I2cClient = client;

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
    g_ReguVdd = regulator_get(&g_I2cClient->dev, vdd_name);
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON
	//zxzadd
	tp_msg_parse_dt(client);

    return MsDrvInterfaceTouchDeviceProbe(g_I2cClient, id);
}

/* remove function is triggered when the input device is removed from input sub-system */
static int touch_driver_remove(struct i2c_client *client)
{
    DBG("*** %s ***\n", __FUNCTION__);

    return MsDrvInterfaceTouchDeviceRemove(client);
}

/* The I2C device list is used for matching I2C device and I2C device driver. */
static const struct i2c_device_id touch_device_id[] =
{
    {MSG_TP_IC_NAME, 0},
    {}, /* should not omitted */ 
};

MODULE_DEVICE_TABLE(i2c, touch_device_id);

static struct of_device_id touch_match_table[] = {
    { .compatible = "mstar,msg2xxx",},
    {},
};

static struct i2c_driver touch_device_driver =
{
    .driver = {
        .name = MSG_TP_IC_NAME,
        .owner = THIS_MODULE,
        .of_match_table = touch_match_table,
    },
    .probe = touch_driver_probe,
    .remove = touch_driver_remove,
    .id_table = touch_device_id,
};

static int __init touch_driver_init(void)
{
    int ret;

    /* register driver */
    ret = i2c_add_driver(&touch_device_driver);
    if (ret < 0)
    {
        DBG("add touch device driver i2c driver failed.\n");
        return -ENODEV;
    }
    DBG("add touch device driver i2c driver.\n");

    return ret;
}

static void __exit touch_driver_exit(void)
{
    DBG("remove touch device driver i2c driver.\n");

    i2c_del_driver(&touch_device_driver);
}

module_init(touch_driver_init);
module_exit(touch_driver_exit);
MODULE_LICENSE("GPL");
