#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/rtc.h>

//#define LDO_POWER_SUPPLY
#ifdef LDO_POWER_SUPPLY
#include <linux/regulator/consumer.h>
#endif


#ifdef LDO_POWER_SUPPLY
/* POWER SUPPLY VOLTAGE RANGE */
#define SN31XX_VDD_MIN_UV 2000000
#define SN31xx_VDD_MAX_UV 3300000
#define SN31XX_VIO_MIN_UV	1750000
#define SN31XX_VIO_MAX_UV	1950000
#endif
struct SN31XX_led {

    struct i2c_client *i2c;
#ifdef LDO_POWER_SUPPLY
    struct regulator *vio;
    struct regulator *vdd;
#endif
    /*
     * Making led_classdev as array is not recommended, because array
     * members prevent using 'container_of' macro. So repetitive works
     * are needed.
     */
    struct led_classdev cdev_led1g;
    unsigned int en_gpio;
#if defined (JRD_PROJECT_PIXI464G) || defined (JRD_PROJECT_PIXI464GCRICKET)
    unsigned int charge_en_gpio;
#endif
    struct mutex rw_lock;
};

static int sn31xx_parse_dt(struct device *dev, struct SN31XX_led *led)
{
    int ret = 0;

    led->en_gpio = of_get_named_gpio_flags(dev->of_node,
            "sn3191,en-gpio", 0, NULL);
    if ((!gpio_is_valid(led->en_gpio)))
        return -EINVAL;
#if defined (JRD_PROJECT_PIXI464G) || defined (JRD_PROJECT_PIXI464GCRICKET)
    led->charge_en_gpio = of_get_named_gpio_flags(dev->of_node,
            "sn3191,charge-en-gpio", 0, NULL);
    if ((!gpio_is_valid(led->charge_en_gpio)))
        return -EINVAL;
#endif
    return ret;
}

static int SN31xx_write_byte(struct i2c_client *client, u8 addr, u8 val)
{
    int ret = 0,retry=3;
    struct SN31XX_led *led = i2c_get_clientdata(client);
    mutex_lock(&led->rw_lock);
    do
    {
        udelay(500);
        ret = i2c_smbus_write_byte_data(client, addr, val);
	 if (ret < 0)
        {
            printk(KERN_ERR "%s: write i2c error retry = %d times\n", __func__,4-retry);
            retry--;
            continue;
        }
        break;
    }while(retry);
    mutex_unlock(&led->rw_lock);
    return ret;
}

static int SN31xx_power_init(struct SN31XX_led *led, bool on){
    int ret = 0 ;

#ifdef LDO_POWER_SUPPLY
    if(!on){
        if(regulator_count_voltages(led->vio) > 0)
            regulator_set_voltage(led->vio, 0, SN31XX_VIO_MAX_UV);
        regulator_put(led->vio);
    }else{
        led->vio = regulator_get(&led->i2c->dev, "vio");
        if(IS_ERR(led->vio)){
            ret = PTR_ERR(led->vio);
            dev_err(&led->i2c->dev,
                "REgulator set failed vio ret = %d\n", ret);
            return ret;
        }
    }
#endif
    return ret;

}

static int SN31xx_power_set(struct SN31XX_led *led, bool on)
{
    int ret = 0;

#ifdef LDO_POWER_SUPPLY
    if(!on){
	ret = regulator_disable(led->vio);
	if(ret){
		dev_err(&led->i2c->dev,
			"Regulator vio disable faild ret = %d\n", ret);
	}
	return ret;
    }
else{
	ret = regulator_enable(led->vio);
	if(ret){
		dev_err(&led->i2c->dev,
            "Regulator vio enable failed rc=%d\n", ret);
        return ret;
	}

  /*
     * The max time for the power supply rise time is 50ms.
     * Use 80ms to make sure it meets the requirements.
     */
    msleep(80);
    return ret;
}
#endif
    return ret;

}

static void SN31xx_unregister_led_classdev(struct SN31XX_led *led)
{

        led_classdev_unregister(&led->cdev_led1g);
}


static int SN31xx_init(struct i2c_client * client)
 {
    int ret = 0;

    ret = SN31xx_write_byte(client, 0x2f, 0x00);

    return ret;
}

void SN31xx_set_ledlg_brightness(struct led_classdev *led_cdev,
				  enum led_brightness brightness)
{
	struct SN31XX_led *led =
		container_of(led_cdev, struct SN31XX_led, cdev_led1g);

	SN31xx_power_set(led, 1);

	gpio_direction_output(led->en_gpio,1);

	printk("set led brightness is %d\n", brightness);

	if(brightness == LED_OFF){
		SN31xx_write_byte(led->i2c, 0x2f, 0x00);
		SN31xx_write_byte(led->i2c, 0x00, 0x01);
		SN31xx_power_set(led, 0);
		gpio_direction_output(led->en_gpio,0);
	}
	else{
            msleep(100);
            SN31xx_write_byte(led->i2c,0x2f, 0x00);
            SN31xx_write_byte(led->i2c,0x00, 0x20);
            SN31xx_write_byte(led->i2c,0x03, 0x08);
            SN31xx_write_byte(led->i2c,0x02, 0x00);
            SN31xx_write_byte(led->i2c,0x04, 0xff);
            SN31xx_write_byte(led->i2c,0x07, 0xff);
	}
}

static int SN31xx_register_led_classdev(struct SN31XX_led *led){
	int ret;
	/* set name for sys/class/leds/led_SN31xx */
	led->cdev_led1g.name = "led_G";
	/* set the default brightness */
	led->cdev_led1g.brightness = LED_OFF;
	/* set brightness function */
	led->cdev_led1g.brightness_set = SN31xx_set_ledlg_brightness;
	//led->cdev_led1g.blink_set = SN31xx_set_ledlg_blink;


	/*  Register class
	 *  sys/class/leds/led_SN31xx
	 */
	ret = led_classdev_register(&led->i2c->dev, &led->cdev_led1g);
	if(ret < 0){
            dev_err(&led->i2c->dev, "couldn't register LED %s\n",
                led->cdev_led1g.name);
		goto failed_unregister_led1_G;
	}

	return 0;

failed_unregister_led1_G:
    led_classdev_unregister(&led->cdev_led1g);

    return ret;

}

/*        ___          ___
*   __ /       \  __ /       \  __
*  t0  t1 t2  t3 t4  t1 t2  t3 t4
*/

static void SN31xx_set_led1g_blink(struct led_classdev *led_cdev,u8 bblink){

	struct SN31XX_led *led =
		container_of(led_cdev,struct SN31XX_led, cdev_led1g);

	struct timespec ts;
	struct rtc_time tm;
#if defined (JRD_PROJECT_PIXI464G) || defined (JRD_PROJECT_PIXI464GCRICKET)
//[Feature]-Add-BEGIN by TCTSZ. bao.li@tcl.com,2016/3/10,for ALM1740195
	int status;
	status=gpio_get_value(led->charge_en_gpio);
	if(status == 0)
	gpio_direction_output(led->charge_en_gpio,1);
//[Feature]-Add-END by TCTSZ. bao.li@tcl.com,2016/3/10,for ALM1740195
#endif
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);

	SN31xx_power_set(led, 1);
	gpio_direction_output(led->en_gpio, 1);

	msleep(100);

	//printk("blink is %d\n", bblink);

	if(bblink == 0 ){
		gpio_direction_output(led->en_gpio, 0);
		SN31xx_write_byte(led->i2c, 0x2f, 0x00);//reset
		printk("%s : %d-%02d-%02d %02d:%02d:%02d:: notification led turn off. bink is %d .\n",__func__,
     tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,tm.tm_hour, tm.tm_min, tm.tm_sec, bblink);
	}
	else if(bblink == 1){
            SN31xx_write_byte(led->i2c, 0x03, 0x08);//set current 5mA
            SN31xx_write_byte(led->i2c, 0x02, 0x20);//one program mode
            SN31xx_write_byte(led->i2c, 0x04, 0xff);//set pwm brightness max
            SN31xx_write_byte(led->i2c, 0x07, 0xff);//flush pwm register
            SN31xx_write_byte(led->i2c, 0x0a, 0x00);//set T0:0s
            SN31xx_write_byte(led->i2c, 0x10, 0x80);//set T1:2.08s T2:0s
            SN31xx_write_byte(led->i2c, 0x16, 0x88);//set T3:2.08s T4:1.04s
            SN31xx_write_byte(led->i2c, 0x1c, 0x00);//flush time register
            SN31xx_write_byte(led->i2c, 0x00, 0x20);//open output
            printk("%s : %d-%02d-%02d %02d:%02d:%02d:: events led turn on. blink is %d .\n",__func__,
     tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,tm.tm_hour, tm.tm_min, tm.tm_sec, bblink);
	}
	else if (bblink == 2){
            SN31xx_write_byte(led->i2c, 0x03, 0x08);//set current 5mA
            SN31xx_write_byte(led->i2c, 0x02, 0x20);//one program mode
            SN31xx_write_byte(led->i2c, 0x04, 0xff);//set pwm brightness max
            SN31xx_write_byte(led->i2c, 0x07, 0xff);//flush pwm register
            SN31xx_write_byte(led->i2c, 0x0a, 0x00);//set T0:0s
            SN31xx_write_byte(led->i2c, 0x10, 0x60);//set T1:1.04s T2:0s
            SN31xx_write_byte(led->i2c, 0x16, 0x66);//set T3:1.04s T4:0.52s
            SN31xx_write_byte(led->i2c, 0x1c, 0x00);//flush time register
            SN31xx_write_byte(led->i2c, 0x00, 0x20);//open output
            printk("%s : %d-%02d-%02d %02d:%02d:%02d:: low battery led turn on. blink is %d .\n",__func__,
     tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,tm.tm_hour, tm.tm_min, tm.tm_sec, bblink);
	}
	else if(bblink == 3 ){
            msleep(100);
            SN31xx_write_byte(led->i2c,0x2f, 0x00);
            SN31xx_write_byte(led->i2c,0x00, 0x20);
            SN31xx_write_byte(led->i2c,0x03, 0x08);
            SN31xx_write_byte(led->i2c,0x02, 0x00);
            SN31xx_write_byte(led->i2c,0x04, 0xff);
            SN31xx_write_byte(led->i2c,0x07, 0xff);
            printk("%s : %d-%02d-%02d %02d:%02d:%02d:: led turn on. blink is %d .\n",__func__,
     tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,tm.tm_hour, tm.tm_min, tm.tm_sec, bblink);
	}
	else{
            SN31xx_write_byte(led->i2c, 0x03, 0x08);
            SN31xx_write_byte(led->i2c, 0x02, 0x20);
            SN31xx_write_byte(led->i2c, 0x04, 0xff);
            SN31xx_write_byte(led->i2c, 0x07, 0xff);
            SN31xx_write_byte(led->i2c, 0x0a, 0x00);
            SN31xx_write_byte(led->i2c, 0x10, 0x60);
            SN31xx_write_byte(led->i2c, 0x16, 0x70);
            SN31xx_write_byte(led->i2c, 0x1c, 0x00);
            SN31xx_write_byte(led->i2c, 0x00, 0x20);
            printk("%s : %d-%02d-%02d %02d:%02d:%02d:: led turn on. blink is %d .\n",__func__,
     tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,tm.tm_hour, tm.tm_min, tm.tm_sec, bblink);
	}
	//printk("Set ledlg blink!\n");

}

static ssize_t store_blink(struct device *dev, struct device_attribute *attr,
              const char *buf, size_t count)
 {
	/* struct led_classdev->dev */
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	u8 bblink;
	pr_err("in %s,name=%s\n",__func__,led_cdev->name);

	//printk("buf is %c\n", *buf);
	if(*buf == '0')
		bblink = 0;
	else if(*buf == '1')
		bblink = 1;
	else if(*buf == '2')
		bblink = 2;
	else if(*buf == '3')
		bblink = 3;
	else
		bblink = 4;

	SN31xx_set_led1g_blink(led_cdev, bblink);

	return count;
}

static DEVICE_ATTR(blink, 0664, NULL, store_blink);

static int sn3191_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct SN31XX_led *led;

	printk("[%s]: Enter!\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_ERR "%s: No Support for I2C_FUNC_I2C\n", __func__);
		return -ENODEV;
	}
	led = kzalloc(sizeof(struct SN31XX_led), GFP_KERNEL);
	if (!led) {
		dev_err(&client->dev,
				"%s: memory allocation failed.", __func__);
		ret = -ENOMEM;
	}

	/* Configure RESET GPIO
	 * Hight: enable   Low: disable
	 */
	sn31xx_parse_dt(&client->dev, led);

	led->i2c = client;

	/* set the led to clint*/
	i2c_set_clientdata(client, led);

	if(!gpio_is_valid(led->en_gpio)){
		printk("led->en_gpio is not valid!\n");
		goto error_free;
	}

	ret = gpio_request(led->en_gpio, "sn3191_en");
	if(ret){
		printk("led->en_gpio is not valid!\n");
		goto error_free;
	}

	gpio_direction_output(led->en_gpio,0);

#if defined (JRD_PROJECT_PIXI464G) || defined (JRD_PROJECT_PIXI464GCRICKET)
	ret = gpio_request(led->charge_en_gpio, "sn3191_charge_en");
	if(ret){
		kfree(led);
		return -ENODEV;
	}
#endif

	mutex_init(&led->rw_lock);
	/* Power Congigure */
	ret = SN31xx_power_init(led, 1);
	if(ret)
            goto error_destroy_rw_lock;

	ret = SN31xx_power_set(led, 1);
	if(ret)
		goto error_power_uinit;

	/* reset the ic */
	ret = SN31xx_init(client);
	if(ret < 0)
		goto error_power_off;
	SN31xx_power_set(led, 0);
	/* Register the led classdev */
	ret = SN31xx_register_led_classdev(led);
	if(ret < 0)
		goto error_power_uinit;

	/*  Create node
	 *  sys/class/leds/led_G/blink
	 */
   ret = sysfs_create_file (&led->cdev_led1g.dev->kobj, &dev_attr_blink.attr);
   if(ret < 0)
        goto error_power_uinit;

   printk("[%s]: successful!\n", __func__);
   return 0;

error_power_off:
    SN31xx_power_set(led, 0);
error_power_uinit:
    SN31xx_power_init(led, 0);
error_destroy_rw_lock:
    mutex_destroy(&led->rw_lock);
    gpio_free(led->en_gpio);
error_free:
    kfree(led);
    printk("[%s]: failed!\n", __func__);
    return ret;
}

static int sn3191_remove(struct i2c_client *client)
{
    struct SN31XX_led *led = i2c_get_clientdata(client);

    SN31xx_unregister_led_classdev(led);
    kfree(led);
    return 0;
}

static const struct i2c_device_id sn3191_id[] = {
	{"sn3191", 0},
	{ }
};


static struct of_device_id sn3191_match_table[] = {
        { .compatible = "sn,sn3191",},
        { },
};


static struct i2c_driver sn3191_i2c_driver = {
	.driver = {
		.name	= "sn3191",
		.owner = THIS_MODULE,
		.of_match_table = sn3191_match_table,
	},
	.probe = sn3191_probe,
	.remove = sn3191_remove,
	.id_table = sn3191_id,
};

static int __init sn3191_led_init(void)
{
	return i2c_add_driver(&sn3191_i2c_driver);;
}

static void __exit sn3191_led_exit(void)
{
	i2c_del_driver(&sn3191_i2c_driver);
}

module_init(sn3191_led_init);
module_exit(sn3191_led_exit);

MODULE_LICENSE("GPL");

