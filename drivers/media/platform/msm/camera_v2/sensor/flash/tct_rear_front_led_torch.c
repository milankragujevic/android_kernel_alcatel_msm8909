/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__  //(printk.h --> pr_xx)

#include <linux/module.h>
#include "msm_led_flash.h"

//#define CONFIG_TCT_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_TCT_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err("[CAM_LED]"fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug("[CAM_LED]"fmt, ##args)
#endif

static struct led_trigger *torch_trigger[2];

static void tct_rear_front_led_torch_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
        int id = -1;
        if(strstr(led_cdev->name,"0"))
        {
		id=0;
        }
        else if(strstr(led_cdev->name,"1"))
	{
		id=1;
	}
	CDBG("E");
	if ((id < 0) || (!torch_trigger[id])) {
		pr_err("No torch trigger found, can't set brightness\n");
		return;
	}
        CDBG("led_cdev->name = %s",led_cdev->name);
	led_trigger_event(torch_trigger[id], value);
	CDBG("X");
};

static struct led_classdev msm_torch_led[2] = {
	{
		.name		= "torch-light0",
		.brightness_set	= tct_rear_front_led_torch_brightness_set,
		.brightness	= LED_OFF,
	},
	{
		.name		= "torch-light1",
		.brightness_set	= tct_rear_front_led_torch_brightness_set,
		.brightness	= LED_OFF,
	},
};

int32_t tct_rear_front_led_torch_create_classdev(struct platform_device *pdev,
				void *data)
{
	int32_t i, rc = 0;
	struct msm_led_flash_ctrl_t *fctrl =
		(struct msm_led_flash_ctrl_t *)data;
	
	CDBG("E");
	if (!fctrl) {
		pr_err("%s Invalid fctrl\n",__func__);
		return -EINVAL;
	}
        //torch_num_sources==1
	for (i = 0; i < fctrl->torch_num_sources; i++) {
		if (fctrl->torch_trigger[i]) {
			torch_trigger[pdev->id] = fctrl->torch_trigger[i];
			rc = led_classdev_register(&pdev->dev,
				&msm_torch_led[pdev->id]);
			if (rc) {
				pr_err("Failed to register %d led dev. rc = %d\n",
						i, rc);
				return rc;
			}
                        //dev_set_drvdata(msm_torch_led[pdev->id].dev, (void *)&pdev->id);
			tct_rear_front_led_torch_brightness_set(&msm_torch_led[pdev->id],LED_OFF);
		} else {
			pr_err("Invalid fctrl->torch_trigger[%d]\n",i);
			return -EINVAL;
		}
	}
	CDBG("X");
	return 0;
};
