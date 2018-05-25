/*
* Driver for pcal6416a I2C GPIO expanders
*/
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/moduleparam.h>
#include <linux/regulator/consumer.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <linux/file.h>

#include "gpio_exp.h"

struct gpio_exp_data {
	struct gpio_chip	chip;
	struct i2c_client 	*client;
	struct mutex		lock;		/* protect 'out' */
	struct work_struct	work;		/* irq demux work */
	struct irq_domain	*irq_domain;	/* for irq demux */
	spinlock_t		slock;		/* protect irq demux */
	u8			out;		/* software latch */
	u8			status;		/* current status */
	int			irq;		/* real irq number */
	u8			reg_addr;	/* register address */


	u8			output0_out;
	u8			output1_out;
	u8			config_port0_out;
	u8			config_port1_out;

	int 			gpio_irq;
	unsigned int 		irq_status;

	int (*write)(struct i2c_client *client, u8 addr, u8 value);
	int (*read)(struct i2c_client *client, u8 addr);
	
};

/*-----------------------------------------------------------------------------------*/

static int i2c_write_reg(struct i2c_client *client, u8 addr, u8 value)
{
	u8 buf[2] = {addr, value};
	int ret;

	printk("%s: addr: 0x%x, value: 0x%x\n", __func__, buf[0], buf[1]);
	ret = i2c_master_send(client, buf, 2);
	if (ret < 0) {
		printk("%s: i2c_master_send fail\n", __func__);
		return ret;
	}

	return 0;
}

static int i2c_read_reg(struct i2c_client *client, u8 addr)
{
	int ret;
	u8 val;
	
	printk("[%s]: addr: 0x%x\n", __func__, addr);

	ret = i2c_master_send(client, &addr, 1);
	if (ret < 0) {
		printk("%s: i2c_master_send fail\n", __func__);
		return ret;	
	}
	ret = i2c_master_recv(client, &val, 1);

	printk("%s: reg 0x%x value=0x%x\n", __func__, addr, val);

	return val;
}

/*---------------------------------------------------------------------------------------*/

static int gpio_exp_input(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_exp_data *data = container_of(chip, struct gpio_exp_data, chip);
	unsigned bit = 1 << offset;
	int err;
	u8 addr;

	printk("[%s]: offset = 0x%x\n", __func__, offset);

	if (offset >= 0 && offset < 8) {
		/* Configuration port_0(gpio_0 to gpio_7) register 0x06(default 0xff) 
		* Set offset bit to 1, the corresponding port pin is enabled as a high-impedance input.
		*/
		addr = 0x06;
		data->config_port0_out = data->config_port0_out | (bit & 0xff);
		mutex_lock(&data->lock);	
		err = data->write(data->client, addr, data->config_port0_out);
		if (err < 0) {
			dev_err(&data->client->dev, "configuration reg 0x%x fail err=%d\n", addr, err);
			mutex_unlock(&data->lock);
			return err;
		}
		mutex_unlock(&data->lock);
	} else if (offset >= 8 && offset < 16) {
		/* Configuration port_0(gpio_8 to gpio_15) register 0x07(default 0xff) 
		* Set offset bit to 1, the corresponding port pin is enabled as a high-impedance input.
		*/
		bit = 1 << (offset - 8);
		addr = 0x07;
		data->config_port1_out = data->config_port1_out | (bit & 0xff);
		mutex_lock(&data->lock);
		err = data->write(data->client, addr, data->config_port1_out);
		if (err < 0) {
			dev_err(&data->client->dev, "configuration reg 0x%x fail err=%d\n", addr, err);
			mutex_unlock(&data->lock);
			return err;
		}
		mutex_unlock(&data->lock);
	} else {
		dev_err(&data->client->dev, "gpio offset error\n");
	}
	return err;
}

static int gpio_exp_get_cansleep(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_exp_data *data = container_of(chip, struct gpio_exp_data, chip);
	int value;
	u8 addr;

	printk("[%s]: offset = 0x%x\n", __func__, offset);

	if (offset >= 0 && offset < 8 ) {
		addr = 0x00; /* Port_0 (gpio_0 ~ gpio_7) */
		value = data->read(data->client, addr);

		return (value < 0) ? -1 : (value & (1 << offset));
	} else if (offset >= 8 && offset < 16) {
		addr = 0x01; /* Port_1 (gpio_8 ~ gpio_15) */
		value = data->read(data->client, addr);

		//return (value < 0) ? -1: (value << 8);
		return (value < 0) ? -1: ((value & (1 << (offset-8))) << 8);
	} else {
		printk("%s: gpio offset error\n", __func__);
		return -1;
	}
}

static int gpio_exp_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct gpio_exp_data *data = container_of(chip, struct gpio_exp_data, chip);
	unsigned bit = 1 << offset;
	u8 addr;
	//u8 val;
	int err;

	printk("[%s]: offset = 0x%x, value = 0x%x\n", __func__, offset, value);

	if (offset >= 0 && offset < 8) {
		/* Configuration port_0(gpio_0 to gpio_7) register 0x06(default 0xff) 
		* Set offset bit to 0, the corresponding port pin is enabled as an output
		*/
		addr = 0x06;
		//val = val & ((~bit) & 0xff);
		data->config_port0_out = data->config_port0_out & ((~bit) & 0xff);
		mutex_lock(&data->lock);
		err = data->write(data->client, addr, data->config_port0_out);
		if (err < 0) {
			dev_err(&data->client->dev, "configuration reg 0x%x fail err=%d\n", addr, err);
			mutex_unlock(&data->lock);
			return err;
		}
		
		/* Write to gpio in port_0(gpio_0 to gpio_7) register 0x02(default 0xff) */
		addr = 0x02;
		if (value) {
			//val = val | (bit & 0xff);
			data->output0_out = data->output0_out | bit;
		} else {
			//val = val | ((~bit) & 0xff);
			data->output0_out = data->output0_out & (~bit);
		}
		err = data->write(data->client, addr, data->output0_out);
		if (err < 0) {
			dev_err(&data->client->dev, "write reg 0x%x fail err=%d\n", addr, err);
			mutex_unlock(&data->lock);
			return err;
		}
		mutex_unlock(&data->lock);

	} else if (offset >= 8 && offset < 16) {
		bit = 1 << (offset - 8);
		/* Configuration port_0(gpio_8 ~ gpio_15) register 0x07(default 0xff) */
		addr = 0x07;
		//val = val & ((~bit) & 0xff);
		data->config_port1_out = data->config_port1_out & ((~bit) & 0xff);
		mutex_lock(&data->lock);

		err = data->write(data->client, addr, data->config_port1_out);
		if (err < 0) {
			dev_err(&data->client->dev, "configuration reg 0x%x fail err=%d\n", addr, err);
			mutex_unlock(&data->lock);
			return err;
		}

		/* Write to gpio in port_1(gpio_8 to gpio_15) register 0x03(default 0xff) */
		addr = 0x03;
		if (value)
			//val = val | (bit & 0xff);
			data->output1_out = data->output1_out | bit;
		else
			//val = val | ((~bit) & 0xff);
			data->output1_out = data->output1_out & (~bit);
		err = data->write(data->client, addr, data->output1_out);
		if (err < 0) {
			dev_err(&data->client->dev, "write reg 0x%x fail err=%d\n", addr, err);
			mutex_unlock(&data->lock);
			return err;
		}
		mutex_unlock(&data->lock);
	} else {
		printk("%s: gpio offset error\n", __func__);
	}

	return err;
}

static void gpio_exp_set_cansleep(struct gpio_chip *chip, unsigned offset, int value)
{
	printk("[%s]: offset = 0x%x, value = 0x%x\n", __func__, offset, value);
	gpio_exp_output(chip, offset, value);
}

/*----------------------------------------irq--------------------------------------*/

static int gpio_exp_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_exp_data *data = container_of(chip, struct gpio_exp_data, chip);
	
	printk("%s: enter\n", __func__);
	
	return irq_create_mapping(data->irq_domain, offset);
}

static void gpio_exp_irq_demux_work(struct work_struct *work)
{
	struct gpio_exp_data *data = container_of(work,
					struct gpio_exp_data,
					work);
	unsigned long change, i, status, flags;
	u8 addr;
	u8 val[2];

	int ret;

	printk("%s: enter\n", __func__);
	addr = 0x00;
	ret = i2c_master_send(data->client, &addr, 1);
	ret = i2c_master_recv(data->client, val, 2);

	status = (val[1] << 8) | val[0];
	printk("%s: status = %ld\n", __func__, status);

	spin_lock_irqsave(&data->slock, flags);
	
	//change = data->status ^ status;
	change = 0x80;
	for_each_set_bit(i, &change, data->chip.ngpio)
		generic_handle_irq(irq_find_mapping(data->irq_domain, i));
	data->status = status;
	spin_unlock_irqrestore(&data->slock, flags);
}

static irqreturn_t gpio_exp_irq_demux(int irq, void *data)
{
	struct gpio_exp_data *gpio = data;

	printk("%s: enter\n", __func__);

	/*
	* gpio_exp can't read/write data here,
	* since i2c data access might go to sleep.
	*/
	schedule_work(&gpio->work);

	return IRQ_HANDLED;
}

static int gpio_exp_irq_domain_map(struct irq_domain *domain, unsigned int virq,
			irq_hw_number_t hw)
{
	irq_set_chip_and_handler(virq,
			&dummy_irq_chip,
			handle_level_irq);
	return 0;
}

static struct irq_domain_ops gpio_exp_irq_domain_ops = {
	.map = gpio_exp_irq_domain_map,
};

static void gpio_exp_irq_domain_cleanup(struct gpio_exp_data *data)
{
	if (data->irq_domain)
		irq_domain_remove(data->irq_domain);

	if (data->irq)
		free_irq(data->irq, data);
}

static int gpio_exp_irq_domain_init(struct gpio_exp_data *data,
			struct gpio_exp_platform_data *pdata,
			struct i2c_client *client)
{
	int status;
	
	printk("%s: enter\n", __func__);
	data->irq_domain = irq_domain_add_linear(client->dev.of_node,
				data->chip.ngpio,
				&gpio_exp_irq_domain_ops,
				NULL);
	if (!data->irq_domain)
		goto fail;

	/* enable real irq */
	status = request_irq(client->irq, gpio_exp_irq_demux, IRQF_TRIGGER_RISING,
			dev_name(&client->dev), data);
	if (status)
		goto fail;
	/* enable gpio_to_irq() */
	INIT_WORK(&data->work, gpio_exp_irq_demux_work);
	data->chip.to_irq = gpio_exp_to_irq;
	data->irq = client->irq;

	return 0;

fail:
	gpio_exp_irq_domain_cleanup(data);
	return -EINVAL;
}

/*---------------------------------------------------------------------------------*/

/* Usage: echo 0xfe > reg_value */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int ret;
	struct gpio_exp_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val)) {
		printk("%s: set reg add fail\n", __func__);
		return -EINVAL;
	}
	mutex_lock(&data->lock);
	x[0] = data->reg_addr;
	mutex_unlock(&data->lock);
	x[1] = val;

	printk("set reg value: addr=0x%x, value=0x%x\n", x[0], x[1]);	
	ret = i2c_master_send(client, x, 2);

	return size;
}

/* Usage: cat reg_value */
static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct gpio_exp_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	u8 val;
	u8 addr;

	mutex_lock(&data->lock);
	addr = data->reg_addr;
	mutex_unlock(&data->lock);
	
	printk("%s: get reg addr=0x%x value\n", __func__, addr);

	ret = i2c_master_send(client, &addr, 1);
	if (ret < 0) {
		printk("send reg addr=0x%x fail\n", addr);	
	}

	ret = i2c_master_recv(client, &val, 1);
	if (ret < 0) {
		printk("i2c_master_recv fail from reg addr=0x%x\n", addr);
		return ret;
	}
	
	ret = sprintf(buf, "recv: value=0x%x\n", val);

	return ret;
}

/* Usage: echo 0x02 > reg_addr */
static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct gpio_exp_data *data = dev_get_drvdata(dev);
	unsigned long val;
	u8 addr;

	if (strict_strtoul(buf, 16, &val)) {
		printk("%s: set reg addr fail\n", __func__);
		return -EINVAL;
	}
	addr = val;
	printk("%s: set reg addr=0x%x\n", __func__, addr);

	mutex_lock(&data->lock);
	data->reg_addr = addr;
	mutex_unlock(&data->lock);

	return size;
}

/* Usage: cat reg_addr */
static ssize_t attr_addr_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct gpio_exp_data *data = dev_get_drvdata(dev);
	u8 addr;

	mutex_lock(&data->lock);
	addr = data->reg_addr;
	mutex_unlock(&data->lock);
	
	printk("%s: current reg addr=0x%x\n", __func__, addr);
	ret = sprintf(buf, "Current reg addr: 0x%x\n", addr);

	return ret;
}
/* Usage: cat all_gpio_state */
static ssize_t attr_gpio_state_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct gpio_exp_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	u8 val[2];
	u8 addr = 0x00; // addr 0x00: Port_0 (gpio_0 to gpio_7), addr 0x01: Port_1 (gpio_8 to gpio_15)

	ret = i2c_master_send(client, &addr, 2);
	if (ret < 0) {
		printk("send reg addr=0x%x fail\n", addr);	
	}

	ret = i2c_master_recv(client, val, 2);
	if (ret < 0) {
		printk("i2c_master_recv fail from reg addr=0x%x\n", addr);
		return ret;
	}
	
	ret = sprintf(buf, "gpio 15~8 = 0x%x, gpio 7~0 = 0x%x\n", val[0], val[1]);

	return ret;
}

static struct device_attribute attributes[] = {
	__ATTR(reg_value, 0666, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0666, attr_addr_get, attr_addr_set),
	__ATTR(all_gpio_state, 0444, attr_gpio_state_get, NULL),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}
/*--------------------------------------------------------------------------------*/

static int gpio_exp_probe(struct i2c_client *client, 
				const struct i2c_device_id *id)
{
	int err = 0;
	struct gpio_exp_data 		*data;
	struct gpio_exp_platform_data	*pdata;
	//struct device_node *np = client->dev.of_node;
	u8 buf[3] = {0x4A, 0x00, 0x00};

	printk("[%s] start.\n", __func__);

	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, "no platform data\n");
	}
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		err = -EPERM;
		goto fail;
	}

	/* Allocate, initialize, and register this gpio_chip */
	data = kzalloc(sizeof(struct gpio_exp_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&client->dev, "failed to allocate memory for module data: %d\n", err);
		err = -ENOMEM;
		goto fail;
	}

	mutex_init(&data->lock);
	spin_lock_init(&data->slock);

	data->chip.base 		= pdata ? pdata->gpio_base : -1;
	data->chip.can_sleep 		= 1;
	data->chip.dev			= &client->dev;
	data->chip.owner		= THIS_MODULE;
	data->chip.get			= gpio_exp_get_cansleep;
	data->chip.set			= gpio_exp_set_cansleep;
	data->chip.direction_input 	= gpio_exp_input;
	data->chip.direction_output	= gpio_exp_output;
	data->chip.ngpio		= id->driver_data;

	printk("[%s] chip.base = %d\n", __func__, data->chip.base);

	/* enable gpio_to_irq() if platform has settings */

	data->gpio_irq = gpio_to_irq(911 + 21); // INT connect to msm gpio_21
	if (data->gpio_irq < 0) {
		printk("[%s]: gpio to irq failed\n", __func__);
		goto fail;
	}
	printk("[%s]: gpio_to_irq = %d\n", __func__, data->gpio_irq);
	client->irq = data->gpio_irq;

	if (client->irq) {
		err = gpio_exp_irq_domain_init(data, pdata, client);
		if (err < 0) {
			dev_err(&client->dev, "irq_domain init failed\n");
			goto fail;
		}
	}
	
	data->reg_addr = 0x00;
	data->irq_status = 0;
	data->write = i2c_write_reg;
	data->read  = i2c_read_reg;

	data->chip.label = client->name;	
	data->client = client;
	i2c_set_clientdata(client, data);


	/* NOTE:  these chips have strange "quasi-bidirectional" I/O pins.
	 * We can't actually know whether a pin is configured (a) as output
	 * and driving the signal low, or (b) as input and reporting a low
	 * value ... without knowing the last value written since the chip
	 * came out of reset (if any).  We can't read the latched output.
	 *
	 * In short, the only reliable solution for setting up pin direction
	 * is to do it explicitly.  The setup() method can do that, but it
	 * may cause transient glitching since it can't know the last value
	 * written (some pins may need to be driven low).
	 *
	 * Using pdata->n_latch avoids that trouble.  When left initialized
	 * to zero, our software copy of the "latch" then matches the chip's
	 * all-ones reset state.  Otherwise it flags pins to be driven low.
	 */
	data->out = 0x00;//pdata ? ~pdata->n_latch : ~0;
	data->status = data->out;
	printk("%s: data->out = 0x%x\n", __func__, data->out);
	data->output0_out = 0x00;
	data->output1_out = 0x00;
	data->config_port0_out = 0xff;
	data->config_port1_out = 0xff;

	err = gpiochip_add(&data->chip);
	if (err < 0)
		goto deinit_power_exit;

	/* Let platform code set up the GPIOs and their users.
	 * Now is the first time anyone could use them.
	 */
	if (pdata && pdata->setup) {
		err = pdata->setup(client,
			data->chip.base, data->chip.ngpio,
			pdata->context);
		if (err < 0)
			dev_warn(&client->dev, "setup --> %d\n", err);
	}

	/* Reset */
	/*pdata->reset_gpio = of_get_named_gpio(np, "gpio_exp,reset-gpio", 0);
	if ((!gpio_is_valid(pdata->reset_gpio))) {
		printk("[%s]: gpio [%d] is not valid\n", __func__, pdata->reset_gpio);
		goto deinit_power_exit;	
	}*/
	/*pdata->reset_gpio = 931;
	err = gpio_request(pdata->reset_gpio, "reset_gpio");
	if (err) {
		printk("[%s]: unable to request gpio [%d]\n", __func__, pdata->reset_gpio);
		goto deinit_power_exit;
	}
	gpio_direction_output(pdata->reset_gpio, 0);
	gpio_direction_output(pdata->reset_gpio, 1);
	gpio_free(pdata->reset_gpio);*/

	/* Config Interrupt mask register pair (4Ah, 4Bh) */
	//u8 buf[3] = {0x4A, 0x00, 0x00};

	err = i2c_master_send(client, buf, 3);
	if (err < 0) {
		printk("%s: config interrupt mask reg fail\n", __func__);
		goto deinit_power_exit;
	}

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev,
		   "device %s sysfs register failed\n", GPIO_EXP_DEV_NAME);
		goto create_sysfs_fail;
	}

	pr_err("%s: Probed\n", GPIO_EXP_DEV_NAME);

	return 0;

create_sysfs_fail:

deinit_power_exit:
/*	gpio_exp_power_deinit(data);
*/
//free_i2c_clientdata_exit:
	i2c_set_clientdata(client, NULL);
	kfree(data);

fail:
	pr_err("[%s]%s: Driver Init failed\n", __func__, GPIO_EXP_DEV_NAME);
	if (pdata && client->irq)
		gpio_exp_irq_domain_cleanup(data);

	return err;
}

static int gpio_exp_remove(struct i2c_client *client)
{
	struct gpio_exp_platform_data *pdata = client->dev.platform_data;
	struct gpio_exp_data *data = i2c_get_clientdata(client);
	int err = 0;

	if (pdata && pdata->teardown) {
		err = pdata->teardown(client,
			data->chip.base, data->chip.ngpio,
			pdata->context);
		if (err < 0) {
			dev_err(&client->dev, "%s --> %d\n",
				"teardown", err);
			return err;
		}
	}

	/*----irq---*/
	if (pdata && client->irq)
		gpio_exp_irq_domain_cleanup(data);
	/*----------*/
	
	err = gpiochip_remove(&data->chip);
	if (err)
		dev_err(&client->dev, "%s --> %d\n", "remove", err);

	remove_sysfs_interfaces(&client->dev);

	return err;
}

static const struct i2c_device_id gpio_exp_id[] = {
	{ GPIO_EXP_DEV_NAME, 16},
	{ },
};

MODULE_DEVICE_TABLE(i2c, gpio_exp_id);

static const struct of_device_id gpio_exp_of_match[] = {
	{ .compatible = "GPIO_EXP,pcal6416a", },
	{ },
};

static struct i2c_driver gpio_exp_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = GPIO_EXP_DEV_NAME,
			.of_match_table = gpio_exp_of_match,
		  },
	.probe = gpio_exp_probe,
	.remove = gpio_exp_remove,
	.id_table = gpio_exp_id,
};

static int __init gpio_exp_init(void)
{
	printk("[%s] Enter\n", __func__);
	return i2c_add_driver(&gpio_exp_driver);
}
subsys_initcall(gpio_exp_init);

static void __exit gpio_exp_exit(void)
{

	i2c_del_driver(&gpio_exp_driver);
	return;
}
module_exit(gpio_exp_exit);

MODULE_DESCRIPTION("Driver for PCAL6416A I2C GPIO Expanders");
MODULE_AUTHOR("TCL");
MODULE_LICENSE("GPL");
