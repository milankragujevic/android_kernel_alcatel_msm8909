#ifndef __HALL_H
#define __HALL_H



#define HALL_VDD_MAX_UV 1950000
#define HALL_VDD_MIN_UV 1750000


struct hall_data {
	int irq_gpio;
	int irq_gpio_sub;
	int tp_is_suspend;
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pin_default;
	struct workqueue_struct *hall_wq;
	struct work_struct hall_work;
	struct work_struct hall_work_sub;
	
	void (*tp_set_sensitivity)(int);
	struct regulator	*vdd;
	struct platform_device	*pdev;
	int power_enabled;
};

#endif


