/******************************************************************************
 * emode_gpio.c - engineer mode GPIO Device Driver
 * 
 * Copyright 2013 rongxiao.deng@tcl.com 
 * 
 * DESCRIPTION:
 *     This file provid the interface for GPIO test of engineer mode 
 *
 ******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/string.h>

#include <linux/types.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/gpio.h>
#include <linux/miscdevice.h>
#include "emode_gpio.h"


/***********************/
/*---------------------------------------------------------------------------*/

/*****************************************************************************/
/* File operation                                                            */
/*****************************************************************************/

static int emode_gpio_open(struct inode *inode, struct file *file)
{
	return 0;
}
/*---------------------------------------------------------------------------*/
static int emode_gpio_release(struct inode *inode, struct file *file)
{
	return 0;
}
/*---------------------------------------------------------------------------*/
static long emode_gpio_ioctl(struct file *file, 
                             unsigned int cmd, unsigned long arg)
{
	long res;
	unsigned long pin;

	switch(cmd) 
	{
        case SET_DIR_IN:      
        {
		pin = (unsigned long)arg;
		res = gpio_direction_input(pin);
		break;
        }
        case SET_DATA_LOW:    
        {
            pin = (unsigned long)arg;
            gpio_set_value(pin, GPIO_OUT_ZERO);
            break;
        }
        case SET_DATA_HIGH:   
        {
            pin = (unsigned long)arg;
            gpio_set_value(pin, GPIO_OUT_ONE);
            break;
        }
        default:
        {
            res = -EPERM;
            break;
        }
    }

	return 0;
}

static ssize_t emode_attr_gpio_export_show(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf)
{
	return 0;	
}

static ssize_t emode_attr_gpio_export_store(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t len)
{
	int gpio_num;
	sscanf(buf, "%d", &gpio_num);
	gpio_export(gpio_num, true);

	return len;
}

static ssize_t emode_attr_gpio_unexport_show(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf)
{
	return 0;	
}

static ssize_t emode_attr_gpio_unexport_store(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t len)
{
	int gpio_num;
	sscanf(buf, "%d", &gpio_num);
	gpio_unexport(gpio_num);

	return len;
}


static DEVICE_ATTR(gpio_export, S_IRUGO | S_IWUSR,
                  emode_attr_gpio_export_show,
                  emode_attr_gpio_export_store);
static DEVICE_ATTR(gpio_unexport, S_IRUGO | S_IWUSR,
                  emode_attr_gpio_unexport_show,
                  emode_attr_gpio_unexport_store);

/*---------------------------------------------------------------------------*/
static struct file_operations emode_gpio_fops = 
{
	.owner=        THIS_MODULE,
	.unlocked_ioctl=	emode_gpio_ioctl,
	.open=         emode_gpio_open,    
	.release=      emode_gpio_release,
};
/*----------------------------------------------------------------------------*/
static struct of_device_id emode_gpio_of_match[] = {
	{ .compatible = "emode-gpio", },
	{ },
	};

MODULE_DEVICE_TABLE(of, emode_gpio_of_match);

static struct miscdevice emode_gpio_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "emode-gpio",
	.fops = &emode_gpio_fops,
};
/*---------------------------------------------------------------------------*/
static int emode_gpio_probe(struct platform_device *dev)
{
	int err;
	printk("%s\n", __func__);
	err = misc_register(&emode_gpio_device);
	if(err)
	{
		printk("register gpio failed %d\n", err);    
		goto err_create_fsnode;
	}

	err = device_create_file(emode_gpio_device.this_device, &dev_attr_gpio_export);
	if(err < 0)
		printk("emode gpio sysfs create device failed\n");

	err = device_create_file(emode_gpio_device.this_device, &dev_attr_gpio_unexport);
	if(err < 0)
		printk("emode gpio sysfs create device failed\n");
    
	printk("Registering engineer mode GPIO device\n");


	return err;
err_create_fsnode:
		device_remove_file(emode_gpio_device.this_device, &dev_attr_gpio_export);
		device_remove_file(emode_gpio_device.this_device, &dev_attr_gpio_unexport);
		misc_deregister(&emode_gpio_device);
	return -1;
}
/*---------------------------------------------------------------------------*/
static int emode_gpio_remove(struct platform_device *dev)
{
	int err;
	struct miscdevice *emode_misc = &emode_gpio_device;

	if ((err = misc_deregister(emode_misc)))
		printk("deregister emode_gpio\n");

	device_remove_file(emode_gpio_device.this_device, &dev_attr_gpio_export);
	device_remove_file(emode_gpio_device.this_device, &dev_attr_gpio_unexport);

	return err;
}
/*---------------------------------------------------------------------------*/
#ifdef CONFIG_PM 
/*---------------------------------------------------------------------------*/
static int emode_gpio_suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret = 0;
	return ret;
	}
/*---------------------------------------------------------------------------*/
static int emode_gpio_resume(struct platform_device *pdev)
{
	int ret = 0;
	return ret;
	}
/*---------------------------------------------------------------------------*/
#endif /*CONFIG_PM*/
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static struct platform_driver emode_gpio_driver = 
{
	.probe   = emode_gpio_probe,
	.remove  = emode_gpio_remove,
#ifdef CONFIG_PM    
	.suspend = emode_gpio_suspend,
	.resume  = emode_gpio_resume,
#endif
	.driver  = {
		.name   = "emode-gpio",
		.owner  = THIS_MODULE,
		.of_match_table = emode_gpio_of_match,
        },    
};
/*---------------------------------------------------------------------------*/
static int __init emode_gpio_init(void)
{
	int ret = 0;

	printk("%s\n", __func__);
	ret = platform_driver_register(&emode_gpio_driver);
	return ret;
}
/*---------------------------------------------------------------------------*/
static void __exit emode_gpio_exit(void)
{
	platform_driver_unregister(&emode_gpio_driver);
	return;
}

/*---------------------------------------------------------------------------*/
module_init(emode_gpio_init);
module_exit(emode_gpio_exit);
MODULE_AUTHOR("rongxiao <rongxiao.deng@tcl.com>");
MODULE_DESCRIPTION("Engineer mode General Purpose Driver (GPIO) ");
MODULE_LICENSE("GPL");
/*---------------------------------------------------------------------------*/



