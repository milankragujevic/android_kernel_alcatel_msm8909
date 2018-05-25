/*
 *  Sysfs interface for the universal power supply monitor class
 *
 *  Copyright © 2007  David Woodhouse <dwmw2@infradead.org>
 *  Copyright © 2007  Anton Vorontsov <cbou@mail.ru>
 *  Copyright © 2004  Szabolcs Gyurko
 *  Copyright © 2003  Ian Molton <spyro@f2s.com>
 *
 *  Modified: 2004, Oct     Szabolcs Gyurko
 *
 *  You may use this code as per GPL version 2
 */

#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/stat.h>
//[Feature]-Add-by TCTSZ.Del qcom for print log  baili.ouyang.sz@tcl.com, 2015/10/12, for PR716604
#include "power_supply.h"
#include <linux/rtc.h>
#include <linux/time.h>
//[Feature]-Add-END by TCTSZ.baili.ouyang.sz@tcl.com, 2015/10/12, for PR PR716604
/*
 * This is because the name "current" breaks the device attr macro.
 * The "current" word resolves to "(get_current())" so instead of
 * "current" "(get_current())" appears in the sysfs.
 *
 * The source of this definition is the device.h which calls __ATTR
 * macro in sysfs.h which calls the __stringify macro.
 *
 * Only modification that the name is not tried to be resolved
 * (as a macro let's say).
 */

#define POWER_SUPPLY_ATTR(_name)					\
{									\
	.attr = { .name = #_name },					\
	.show = power_supply_show_property,				\
	.store = power_supply_store_property,				\
}

static struct device_attribute power_supply_attrs[];
//[Feature]-Add-BEGIN by TCTSZ.Del qcom for print log  baili.ouyang.sz@tcl.com,2015/10/12, for PR716604
static int batt_voltage=0, batt_curr=0xFFFF, batt_temp=-500, batt_cap=-1;
static char status[12]={};
static char health[24]={};
static char charging_type[12]={};
static int charger_input_voltage=0;
static char batt_id[128]={};//add battery id voltage
//[Feature]-Add-END by TCTSZ.baili.ouyang.sz@tcl.com, 2015/10/12, for PR716604
static ssize_t power_supply_show_property(struct device *dev,
					  struct device_attribute *attr,
					  char *buf) {
	static char *type_text[] = {
		"Unknown", "Battery", "UPS", "Mains", "USB",
		"USB_DCP", "USB_CDP", "USB_ACA", "Wireless", "BMS",
		"USB_Parallel"
	};
	static char *status_text[] = {
		"Unknown", "Charging", "Discharging", "Not charging", "Full"
	};
	static char *charge_type[] = {
		"Unknown", "N/A", "Trickle", "Fast", "Taper"
	};
	static char *health_text[] = {
		"Unknown", "Good", "Overheat", "Warm", "Dead", "Over voltage",
		"Unspecified failure", "Cold", "Cool", "Watchdog timer expire",
		"Safety timer expire"
	};
	static char *technology_text[] = {
		"Unknown", "NiMH", "Li-ion", "Li-poly", "LiFe", "NiCd",
		"LiMn"
	};
	static char *capacity_level_text[] = {
		"Unknown", "Critical", "Low", "Normal", "High", "Full"
	};
	static char *scope_text[] = {
		"Unknown", "System", "Device"
	};
	ssize_t ret = 0;
	struct power_supply *psy = dev_get_drvdata(dev);
	const ptrdiff_t off = attr - power_supply_attrs;
	union power_supply_propval value;

	if (off == POWER_SUPPLY_PROP_TYPE)
		value.intval = psy->type;
	else
		ret = psy->get_property(psy, off, &value);

	if (ret < 0) {
		if (ret == -ENODATA)
			dev_dbg(dev, "driver has no data for `%s' property\n",
				attr->attr.name);
		else if (ret != -ENODEV)
			dev_err(dev, "driver failed to report `%s' property: %zd\n",
				attr->attr.name, ret);
		return ret;
	}
//[Feature]-Add-BEGIN by TCTSZ.Del qcom for print log  baili.ouyang.sz@tcl.com, 2015/10/12, for PR716604
	if(strcmp(psy->name, "battery") == 0)
	{
		if(off == POWER_SUPPLY_PROP_VOLTAGE_NOW)
			batt_voltage=(int)value.intval;
		else if(off == POWER_SUPPLY_PROP_CURRENT_NOW)
			batt_curr=(int)value.intval;
		else if (off == POWER_SUPPLY_PROP_TEMP)
			batt_temp=(int)value.intval;
		else if (off == POWER_SUPPLY_PROP_CAPACITY)
			batt_cap=(int)value.intval;
		else if (off == POWER_SUPPLY_PROP_STATUS){
			memset(&status, 0, sizeof(status));
			sprintf(status, "%s", status_text[value.intval]);
		}
		else if (off == POWER_SUPPLY_PROP_HEALTH){
			memset(&health, 0, sizeof(health));
			sprintf(health, "%s", health_text[value.intval]);
		}
		else if (off == POWER_SUPPLY_PROP_CHARGER_INPUT_VOLTAGE){
			charger_input_voltage = (int)value.intval;
		}
		else if (off == POWER_SUPPLY_PROP_BATT_ID){
		memset(&batt_id, 0, sizeof(batt_id));
#if defined (JRD_PROJECT_POP45C)||defined (JRD_PROJECT_POP455C)
	if((value.intval>400000) && (value.intval<500000))		//BYD ID
	{
		sprintf(batt_id,"BYD_TLp025C1_%dmv",value.intval/1000);
	}
	else if((value.intval>540000) && (value.intval<670000))		//SCUD ID
	{
		sprintf(batt_id,"SCUD_TLp025C2_%dmv",value.intval/1000);
	}
#elif defined (JRD_PROJECT_POP45)|| defined(JRD_PROJECT_PIXI4554G)
//[Feature]-Add-BEGIN by TCTSZ Battery_ID baili.ouyang.sz@tcl.com, 2016/02/17, for Task 1163258
	if((value.intval>700000) && (value.intval<1100000))		//blbattery ID
	{
		sprintf(batt_id,"JIADE_JIADE2500_%dmv",value.intval/1000);
	}
	else if((value.intval>50000) && (value.intval<250000))		//veken ID
	{
		sprintf(batt_id,"VEKEN_TLp025H7_%dmv",value.intval/1000);
	}
	else if((value.intval>300000) && (value.intval<600000))		//BYD ID
	{
		sprintf(batt_id,"BYD_TLp025H1_%dmv",value.intval/1000);
	}
#else
	if(1)
		sprintf(batt_id,"OTHER_PROJECT_BATTEERY_ID_%dmv",value.intval/1000);
#endif
	else
		sprintf(batt_id, "ERROR_BATTERY_ID_%dmv", value.intval/1000);
	}
	}
	if(strcmp(psy->name, "usb") == 0)
	{
		if (off == POWER_SUPPLY_PROP_TYPE){
			memset(&charging_type, 0, sizeof(charging_type));
			sprintf(charging_type, "%s", type_text[value.intval]);
		}
	}
//[Feature]-Add-END by TCTSZ.baili.ouyang.sz@tcl.com, 2015/10/12, for PR716604

	if (off == POWER_SUPPLY_PROP_STATUS)
		return sprintf(buf, "%s\n", status_text[value.intval]);
	else if (off == POWER_SUPPLY_PROP_CHARGE_TYPE)
		return sprintf(buf, "%s\n", charge_type[value.intval]);
	else if (off == POWER_SUPPLY_PROP_HEALTH)
		return sprintf(buf, "%s\n", health_text[value.intval]);
	else if (off == POWER_SUPPLY_PROP_TECHNOLOGY)
		return sprintf(buf, "%s\n", technology_text[value.intval]);
	else if (off == POWER_SUPPLY_PROP_CAPACITY_LEVEL)
		return sprintf(buf, "%s\n", capacity_level_text[value.intval]);
	else if (off == POWER_SUPPLY_PROP_TYPE)
		return sprintf(buf, "%s\n", type_text[value.intval]);
	else if (off == POWER_SUPPLY_PROP_SCOPE)
		return sprintf(buf, "%s\n", scope_text[value.intval]);
	else if (off >= POWER_SUPPLY_PROP_MODEL_NAME)
		return sprintf(buf, "%s\n", value.strval);
	else if (off == POWER_SUPPLY_PROP_BATT_ID)
		return sprintf(buf, "%s\n", batt_id);
	if (off == POWER_SUPPLY_PROP_CHARGE_COUNTER_EXT)
		return sprintf(buf, "%lld\n", value.int64val);
	else
		return sprintf(buf, "%d\n", value.intval);
}

static ssize_t power_supply_store_property(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count) {
	ssize_t ret;
	struct power_supply *psy = dev_get_drvdata(dev);
	const ptrdiff_t off = attr - power_supply_attrs;
	union power_supply_propval value;
	long long_val;

	/* TODO: support other types than int */
	ret = strict_strtol(buf, 10, &long_val);
	if (ret < 0)
		return ret;

	value.intval = long_val;

	ret = psy->set_property(psy, off, &value);
	if (ret < 0)
		return ret;

	return count;
}

/* Must be in the same order as POWER_SUPPLY_PROP_* */
static struct device_attribute power_supply_attrs[] = {
	/* Properties of type `int' */
	POWER_SUPPLY_ATTR(status),
	POWER_SUPPLY_ATTR(charge_type),
	POWER_SUPPLY_ATTR(health),
	POWER_SUPPLY_ATTR(present),
	POWER_SUPPLY_ATTR(online),
	POWER_SUPPLY_ATTR(authentic),
	POWER_SUPPLY_ATTR(charging_enabled),
	POWER_SUPPLY_ATTR(technology),
	POWER_SUPPLY_ATTR(cycle_count),
	POWER_SUPPLY_ATTR(voltage_max),
	POWER_SUPPLY_ATTR(voltage_min),
	POWER_SUPPLY_ATTR(voltage_max_design),
	POWER_SUPPLY_ATTR(voltage_min_design),
	POWER_SUPPLY_ATTR(voltage_now),
	POWER_SUPPLY_ATTR(voltage_avg),
	POWER_SUPPLY_ATTR(voltage_ocv),
	POWER_SUPPLY_ATTR(input_voltage_regulation),
	POWER_SUPPLY_ATTR(current_max),
	POWER_SUPPLY_ATTR(input_current_max),
	POWER_SUPPLY_ATTR(input_current_trim),
	POWER_SUPPLY_ATTR(input_current_settled),
	POWER_SUPPLY_ATTR(bypass_vchg_loop_debouncer),
	POWER_SUPPLY_ATTR(current_now),
	POWER_SUPPLY_ATTR(current_avg),
	POWER_SUPPLY_ATTR(power_now),
	POWER_SUPPLY_ATTR(power_avg),
	POWER_SUPPLY_ATTR(charge_full_design),
	POWER_SUPPLY_ATTR(charge_empty_design),
	POWER_SUPPLY_ATTR(charge_full),
	POWER_SUPPLY_ATTR(charge_empty),
	POWER_SUPPLY_ATTR(charge_now),
	POWER_SUPPLY_ATTR(charge_avg),
	POWER_SUPPLY_ATTR(charge_counter),
	POWER_SUPPLY_ATTR(charge_counter_shadow),
	POWER_SUPPLY_ATTR(constant_charge_current),
	POWER_SUPPLY_ATTR(constant_charge_current_max),
	POWER_SUPPLY_ATTR(constant_charge_voltage),
	POWER_SUPPLY_ATTR(constant_charge_voltage_max),
	POWER_SUPPLY_ATTR(charge_control_limit),
	POWER_SUPPLY_ATTR(charge_control_limit_max),
	POWER_SUPPLY_ATTR(energy_full_design),
	POWER_SUPPLY_ATTR(energy_empty_design),
	POWER_SUPPLY_ATTR(energy_full),
	POWER_SUPPLY_ATTR(energy_empty),
	POWER_SUPPLY_ATTR(energy_now),
	POWER_SUPPLY_ATTR(energy_avg),
	POWER_SUPPLY_ATTR(hi_power),
	POWER_SUPPLY_ATTR(low_power),
	POWER_SUPPLY_ATTR(capacity),
	POWER_SUPPLY_ATTR(capacity_alert_min),
	POWER_SUPPLY_ATTR(capacity_alert_max),
	POWER_SUPPLY_ATTR(capacity_level),
	POWER_SUPPLY_ATTR(temp),
	POWER_SUPPLY_ATTR(temp_alert_min),
	POWER_SUPPLY_ATTR(temp_alert_max),
	POWER_SUPPLY_ATTR(temp_cool),
	POWER_SUPPLY_ATTR(temp_warm),
//[Feature]-Add-BEGIN by TCTSZ.Del qcom for different charging current set under different temperature  baili.ouyang.sz@tcl.com, 2015/10/12, for PR716604
	POWER_SUPPLY_ATTR(temp_cold),
	POWER_SUPPLY_ATTR(temp_overheat),

	POWER_SUPPLY_ATTR(temp_voltage), //add by shicuiping for therm adc voltage
	POWER_SUPPLY_ATTR(power_on_voltage), // [PLATFORM]-Add by TCTSZ.cuiping.shi, for get power on voltage, 2014/09/29
	POWER_SUPPLY_ATTR(temp_ambient),
	POWER_SUPPLY_ATTR(batt_id),//add battery id voltage PR1990499
	POWER_SUPPLY_ATTR(charger_input_voltage),
//[Feature]-Add-END by TCTSZ.baili.ouyang.sz@tcl.com, 2015/10/12, for PR716604
	POWER_SUPPLY_ATTR(temp_ambient_alert_min),
	POWER_SUPPLY_ATTR(temp_ambient_alert_max),
	POWER_SUPPLY_ATTR(time_to_empty_now),
	POWER_SUPPLY_ATTR(time_to_empty_avg),
	POWER_SUPPLY_ATTR(time_to_full_now),
	POWER_SUPPLY_ATTR(time_to_full_avg),
	POWER_SUPPLY_ATTR(type),
	POWER_SUPPLY_ATTR(scope),
	POWER_SUPPLY_ATTR(system_temp_level),
	POWER_SUPPLY_ATTR(resistance),
	POWER_SUPPLY_ATTR(resistance_capacitive),
	POWER_SUPPLY_ATTR(resistance_id),
	POWER_SUPPLY_ATTR(resistance_now),
	/* Local extensions */
	POWER_SUPPLY_ATTR(usb_hc),
	POWER_SUPPLY_ATTR(usb_otg),
	POWER_SUPPLY_ATTR(charge_enabled),
	POWER_SUPPLY_ATTR(flash_current_max),
	/* Local extensions of type int64_t */
	POWER_SUPPLY_ATTR(charge_counter_ext),
	/* Properties of type `const char *' */
	POWER_SUPPLY_ATTR(model_name),
	POWER_SUPPLY_ATTR(manufacturer),
	POWER_SUPPLY_ATTR(serial_number),
	POWER_SUPPLY_ATTR(battery_type),
};

static struct attribute *
__power_supply_attrs[ARRAY_SIZE(power_supply_attrs) + 1];

static umode_t power_supply_attr_is_visible(struct kobject *kobj,
					   struct attribute *attr,
					   int attrno)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct power_supply *psy = dev_get_drvdata(dev);
	umode_t mode = S_IRUSR | S_IRGRP | S_IROTH;
	int i;

	if (attrno == POWER_SUPPLY_PROP_TYPE)
		return mode;

	for (i = 0; i < psy->num_properties; i++) {
		int property = psy->properties[i];

		if (property == attrno) {
			if (psy->property_is_writeable &&
			    psy->property_is_writeable(psy, property) > 0)
				mode |= S_IWUSR;

			return mode;
		}
	}

	return 0;
}

static struct attribute_group power_supply_attr_group = {
	.attrs = __power_supply_attrs,
	.is_visible = power_supply_attr_is_visible,
};

static const struct attribute_group *power_supply_attr_groups[] = {
	&power_supply_attr_group,
	NULL,
};

void power_supply_init_attrs(struct device_type *dev_type)
{
	int i;

	dev_type->groups = power_supply_attr_groups;

	for (i = 0; i < ARRAY_SIZE(power_supply_attrs); i++)
		__power_supply_attrs[i] = &power_supply_attrs[i].attr;
}

static char *kstruprdup(const char *str, gfp_t gfp)
{
	char *ret, *ustr;

	ustr = ret = kmalloc(strlen(str) + 1, gfp);

	if (!ret)
		return NULL;

	while (*str)
		*ustr++ = toupper(*str++);

	*ustr = 0;

	return ret;
}

int power_supply_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	int ret = 0, j;
	char *prop_buf;
	char *attrname;

	dev_dbg(dev, "uevent\n");

	if (!psy || !psy->dev) {
		dev_dbg(dev, "No power supply yet\n");
		return ret;
	}

	dev_dbg(dev, "POWER_SUPPLY_NAME=%s\n", psy->name);

	ret = add_uevent_var(env, "POWER_SUPPLY_NAME=%s", psy->name);
	if (ret)
		return ret;

	prop_buf = (char *)get_zeroed_page(GFP_KERNEL);
	if (!prop_buf)
		return -ENOMEM;

	for (j = 0; j < psy->num_properties; j++) {
		struct device_attribute *attr;
		char *line;

		attr = &power_supply_attrs[psy->properties[j]];

		ret = power_supply_show_property(dev, attr, prop_buf);
		if (ret == -ENODEV || ret == -ENODATA) {
			/* When a battery is absent, we expect -ENODEV. Don't abort;
			   send the uevent with at least the the PRESENT=0 property */
			ret = 0;
			continue;
		}

		if (ret < 0)
			goto out;

		line = strchr(prop_buf, '\n');
		if (line)
			*line = 0;

		attrname = kstruprdup(attr->attr.name, GFP_KERNEL);
		if (!attrname) {
			ret = -ENOMEM;
			goto out;
		}

		dev_dbg(dev, "prop %s=%s\n", attrname, prop_buf);

		ret = add_uevent_var(env, "POWER_SUPPLY_%s=%s", attrname, prop_buf);
		kfree(attrname);
		if (ret)
			goto out;
	}

//[Feature]-Add-BEGIN by TCTSZ.Del qcom for print log  baili.ouyang.sz@tcl.com, 2015/10/12, for PR716604
	if(strcmp(psy->name, "battery") == 0)
	if( batt_voltage!=0 && batt_curr!=0xFFFF && batt_temp!=(-500) && batt_cap!=(-1) )
	{
		struct timespec ts;
		struct rtc_time tm;

		getnstimeofday(&ts);
		set_localtimezone_timespec(&ts);
		rtc_time_to_tm(ts.tv_sec, &tm);
		dev_info(dev, "%d-%02d-%02d %02d:%02d:%02d status:%s%s%s%s,health:%s,voltage:%dmV,capacity:%d,current:%dmA,temperature:%s%d.%dC,chgvoltage:%dmV\n",
			 tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,tm.tm_hour, tm.tm_min, tm.tm_sec,
				status,strcmp(status, "Charging")?"":"(",strcmp(status, "Charging")?"":charging_type,strcmp(status, "Charging")?"":")",health,batt_voltage/1000,batt_cap,batt_curr/1000,(batt_temp<0&&(batt_temp/10==0))?"-":"", batt_temp/10,(int)abs(batt_temp%10),charger_input_voltage/1000);
	}
//[Feature]-Add-END by TCTSZ.baili.ouyang.sz@tcl.com, 2015/10/12, for PR716604
out:
	free_page((unsigned long)prop_buf);

	return ret;
}
