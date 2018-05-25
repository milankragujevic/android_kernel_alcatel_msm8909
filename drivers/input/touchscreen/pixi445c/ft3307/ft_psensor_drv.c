#include <linux/sensors.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/delay.h>

extern int ft6x06_ts_resume(struct device *dev);

extern  struct i2c_client  *ft_g_client;
extern  u8 tpd_proximity_ft6336_enable;
#define VPS_NAME "stk3x1x-ps"

struct virtualpsensor *vps;

struct virtualpsensor {
	char const *name;
//	struct i2c_client *client;
	struct input_dev *virtualdevice;
	bool vps_enabled;
	struct sensors_classdev vps_cdev;
	bool virtual_proximity_data;
};


 struct sensors_classdev virtual_sensors_proximity_cdev = {
	.name = VPS_NAME,
	.vendor = "NULL",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.flags = 1,
};



 int vps_set_enable(struct sensors_classdev *sensors_cdev,unsigned int enable)
{
	u8 read_state;
	u8 set_state;
	int ret = -1;
	int i;
	
	//if want to enable P-sensor should resume TP first
	if(enable)
	ft6x06_ts_resume(&ft_g_client->dev);

	set_state=enable;
	
	for(i=0;i<3;i++)
	{
		ret = i2c_smbus_write_i2c_block_data(ft_g_client, 0xB0, 1, &set_state);
		i2c_smbus_read_i2c_block_data(ft_g_client, 0xB0, 1, &read_state);
	
		if(read_state==set_state)
		{
			vps->vps_enabled = read_state;
			tpd_proximity_ft6336_enable = vps->vps_enabled;
			pr_err("=====vps_set_enable==i=%d===set_state=%d==========\n",i,set_state);
			break;
		}
		
	}


	return 0;

}

 int virtual_psensor_input_register(struct i2c_client *pClient)
{
	s32 nRetVal = 0;

	pr_err("*** %s() ***\n", __func__);

	vps = kzalloc(sizeof(struct virtualpsensor), GFP_KERNEL);
//	vps->client = pClient;
//	i2c_set_clientdata(pClient, vps);

	vps->virtualdevice= input_allocate_device();
	if (vps->virtualdevice == NULL)
	{
		pr_err("*** input device allocation failed ***\n");
		return -ENOMEM;
	}

	vps->virtualdevice->name = "stk3x1x-ps";
	vps->virtualdevice->id.bustype = BUS_I2C;

	/* set the supported event type for input device */
	set_bit(EV_ABS, vps->virtualdevice->evbit);
	set_bit(ABS_DISTANCE, vps->virtualdevice->absbit);
	input_set_abs_params(vps->virtualdevice, ABS_DISTANCE, 0, 1, 0, 0);

	nRetVal = input_register_device(vps->virtualdevice);
	if (nRetVal < 0)
	{
		pr_err("*** Unable to register virtual P-sensor input device ***\n");
		return nRetVal;
	}

	vps->vps_cdev = virtual_sensors_proximity_cdev;
	vps->vps_cdev.sensors_enable = vps_set_enable;
	vps->vps_cdev.sensors_poll_delay = NULL;

	nRetVal = sensors_classdev_register(&pClient->dev, &vps->vps_cdev);
	if (nRetVal) {
		pr_err("%s: Unable to register to sensors class: %d\n",__func__, nRetVal);
	return nRetVal;
	}

	return 0;
}
