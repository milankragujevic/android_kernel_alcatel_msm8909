/*
 *
 * FocalTech fts TouchScreen driver.
 * 
 * Copyright (c) 2010-2015, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

 /*******************************************************************************
*
* File Name: Focaltech_ex_fun.c
*
* Author: Xu YongFeng
*
* Created: 2015-01-29
*   
* Modify by mshl on 2015-06-11
*
* Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include "focaltech_core.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
/*create apk debug channel*/
#define PROC_UPGRADE			0
#define PROC_READ_REGISTER		1
#define PROC_WRITE_REGISTER	2
#define PROC_AUTOCLB			4
#define PROC_UPGRADE_INFO		5
#define PROC_WRITE_DATA		6
#define PROC_READ_DATA			7
#define PROC_SET_TEST_FLAG				8
#define PROC_NAME	"ftxxxx-debug"

#define WRITE_BUF_SIZE		512
#define READ_BUF_SIZE		512

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/
int fts_gesture_enabled = 0;	/* module switch */
bool fts_irq_enable = 0;

bool FTSDebugMode = 0;
#define FTS_TP_DBG(format, ...) \
{ \
    if(FTSDebugMode) \
    { \
        printk(KERN_INFO "[Focal]:" format , ## __VA_ARGS__); \
    } \
}


/*******************************************************************************
* Static variables
*******************************************************************************/
static unsigned char proc_operate_mode = PROC_UPGRADE;
static struct proc_dir_entry *fts_proc_entry;
/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
#if FTS_ESD_PROTECT
int apk_debug_flag = 0;
#endif
/*******************************************************************************
* Static function prototypes
*******************************************************************************/

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
/*interface of write proc*/
/************************************************************************
*   Name: fts_debug_write
*  Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
***********************************************************************/
static ssize_t fts_debug_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned char writebuf[WRITE_BUF_SIZE];
	int buflen = count;
	int writelen = 0;
	int ret = 0;
	#if FTS_ESD_PROTECT
	esd_switch(0);
	apk_debug_flag = 1;
	#endif
	if (copy_from_user(&writebuf, buff, buflen)) {
		dev_err(&fts_i2c_client->dev, "%s:copy from user error\n", __func__);
	#if FTS_ESD_PROTECT
	esd_switch(1);
	apk_debug_flag = 0;
	#endif
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			FTS_DBG("%s\n", upgrade_file_path);
			disable_irq(fts_i2c_client->irq);
		//	#if FTS_ESD_PROTECT
		//	apk_debug_flag = 1;
		//	#endif
			
			ret = fts_ctpm_fw_upgrade_with_app_file(fts_i2c_client, upgrade_file_path);
		//	#if FTS_ESD_PROTECT
		//	apk_debug_flag = 0;
		//	#endif
			enable_irq(fts_i2c_client->irq);
			if (ret < 0) {
				#if FTS_ESD_PROTECT
					esd_switch(1);
					apk_debug_flag = 0;
				#endif
				dev_err(&fts_i2c_client->dev, "%s:upgrade failed.\n", __func__);
				return ret;
			}
		}
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
	#if FTS_ESD_PROTECT
		esd_switch(1);
		apk_debug_flag = 0;
	#endif
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
	#if FTS_ESD_PROTECT
		esd_switch(1);
		apk_debug_flag = 0;
	#endif
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		FTS_DBG("%s: autoclb\n", __func__);
		fts_ctpm_auto_clb(fts_i2c_client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = count - 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
	#if FTS_ESD_PROTECT
		esd_switch(1);
		apk_debug_flag = 0;
	#endif
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	default:
		break;
	}
	
	#if FTS_ESD_PROTECT
		esd_switch(1);
		apk_debug_flag = 0;
	#endif

	return count;
}

/*interface of read proc*/
/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use 
* Output: page point to data
* Return: read char number
***********************************************************************/
static ssize_t fts_debug_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
	int ret = 0;
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	unsigned char buf[READ_BUF_SIZE];
	#if FTS_ESD_PROTECT
		esd_switch(0);
		apk_debug_flag = 1;
	#endif
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		//after calling fts_debug_write to upgrade
		regaddr = 0xA6;
		ret = fts_read_reg(fts_i2c_client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
#if FTS_ESD_PROTECT
		esd_switch(1);
		apk_debug_flag = 0;
#endif
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		} 
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
#if FTS_ESD_PROTECT
		esd_switch(1);
		apk_debug_flag = 0;
#endif
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		}
		
		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}
	
	if (copy_to_user(buff, buf, num_read_chars)) {
		dev_err(&fts_i2c_client->dev, "%s:copy to user error\n", __func__);
#if FTS_ESD_PROTECT
		esd_switch(1);
		apk_debug_flag = 0;
#endif
		return -EFAULT;
	}
	#if FTS_ESD_PROTECT
	esd_switch(1);
	apk_debug_flag = 0;
	#endif
	return num_read_chars;
}
static const struct file_operations fts_proc_fops = {
		.owner = THIS_MODULE,
		.read = fts_debug_read,
		.write = fts_debug_write,
		
};
#else
/*interface of write proc*/
/************************************************************************
*   Name: fts_debug_write
*  Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
***********************************************************************/
static int fts_debug_write(struct file *filp, 
	const char __user *buff, unsigned long len, void *data)
{
	unsigned char writebuf[WRITE_BUF_SIZE];
	int buflen = len;
	int writelen = 0;
	int ret = 0;
	
	#if FTS_ESD_PROTECT
	esd_switch(0);
	apk_debug_flag = 1;
	#endif
	if (copy_from_user(&writebuf, buff, buflen)) {
		dev_err(&fts_i2c_client->dev, "%s:copy from user error\n", __func__);
#if FTS_ESD_PROTECT
	esd_switch(1);
	apk_debug_flag = 0;
#endif
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			FTS_DBG("%s\n", upgrade_file_path);
			disable_irq(fts_i2c_client->irq);
		//	#if FTS_ESD_PROTECT
		//		apk_debug_flag = 1;
		//	#endif
			ret = fts_ctpm_fw_upgrade_with_app_file(fts_i2c_client, upgrade_file_path);
		//	#if FTS_ESD_PROTECT
		//		apk_debug_flag = 0;
		//	#endif
			enable_irq(fts_i2c_client->irq);
			if (ret < 0) {
				dev_err(&fts_i2c_client->dev, "%s:upgrade failed.\n", __func__);
			#if FTS_ESD_PROTECT
				esd_switch(1);
				apk_debug_flag = 0;
			#endif
				return ret;
			}
		}
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
#if FTS_ESD_PROTECT
		esd_switch(1);
		apk_debug_flag = 0;
#endif
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
#if FTS_ESD_PROTECT
		esd_switch(1);
		apk_debug_flag = 0;
#endif
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		FTS_DBG("%s: autoclb\n", __func__);
		fts_ctpm_auto_clb(fts_i2c_client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = len - 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
#if FTS_ESD_PROTECT
		esd_switch(1);
		apk_debug_flag = 0;
#endif
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	default:
		break;
	}
	
#if FTS_ESD_PROTECT
	esd_switch(1);
	apk_debug_flag = 0;
#endif

	return len;
}

/*interface of read proc*/
/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use 
* Output: page point to data
* Return: read char number
***********************************************************************/
static int fts_debug_read( char *page, char **start,
	off_t off, int count, int *eof, void *data )
{
	int ret = 0;
	unsigned char buf[READ_BUF_SIZE];
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	#if FTS_ESD_PROTECT
	esd_switch(0);
	apk_debug_flag = 1;
	#endif
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		//after calling fts_debug_write to upgrade
		regaddr = 0xA6;
		ret = fts_read_reg(fts_i2c_client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
#if FTS_ESD_PROTECT
	esd_switch(1);
	apk_debug_flag = 0;
#endif
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		} 
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
#if FTS_ESD_PROTECT
		esd_switch(1);
		apk_debug_flag = 0;
#endif
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		}
		
		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}
	
	memcpy(page, buf, num_read_chars);
#if FTS_ESD_PROTECT
	esd_switch(1);
	apk_debug_flag = 0;
#endif
	return num_read_chars;
}
#endif
/************************************************************************
* Name: fts_create_apk_debug_channel
* Brief:  create apk debug channel
* Input: i2c info
* Output: no
* Return: success =0
***********************************************************************/
int fts_create_apk_debug_channel(struct i2c_client * client)
{	
	#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
		fts_proc_entry = proc_create(PROC_NAME, 0777, NULL, &fts_proc_fops);		
	#else
		fts_proc_entry = create_proc_entry(PROC_NAME, 0777, NULL);
	#endif
	if (NULL == fts_proc_entry) 
	{
		dev_err(&client->dev, "Couldn't create proc entry!\n");
		
		return -ENOMEM;
	} 
	else 
	{
		dev_info(&client->dev, "Create proc entry success!\n");
		
		#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
			fts_proc_entry->write_proc = fts_debug_write;
			fts_proc_entry->read_proc = fts_debug_read;
		#endif
	}
	return 0;
}
/************************************************************************
* Name: fts_release_apk_debug_channel
* Brief:  release apk debug channel
* Input: no
* Output: no
* Return: no
***********************************************************************/
void fts_release_apk_debug_channel(void)
{
	
	if (fts_proc_entry)
		#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
			proc_remove(fts_proc_entry);
		#else
			remove_proc_entry(PROC_NAME, NULL);
		#endif
}

/************************************************************************
* Name: fts_tpfwver_show
* Brief:  show tp fw vwersion
* Input: device, device attribute, char buf
* Output: no
* Return: char number
***********************************************************************/
static ssize_t fts_tpfwver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;
	u8 fwver = 0;
	mutex_lock(&fts_input_dev->mutex);
	if (fts_read_reg(fts_i2c_client, FTS_REG_FW_VER, &fwver) < 0)
		return -1;
	
	
	if (fwver == 255)
		num_read_chars = snprintf(buf, 128,"get tp fw version fail!\n");
	else
	{
		num_read_chars = snprintf(buf, 128, "%02X\n", fwver);
	}
	
	mutex_unlock(&fts_input_dev->mutex);
	
	return num_read_chars;
}
/************************************************************************
* Name: fts_tpfwver_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tpfwver_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tpdriver_version_show
* Brief:  show tp fw vwersion
* Input: device, device attribute, char buf
* Output: no
* Return: char number
***********************************************************************/
static ssize_t fts_tpdriver_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;
	
	mutex_lock(&fts_input_dev->mutex);
	
	num_read_chars = snprintf(buf, 128,"%s \n", FTS_DRIVER_INFO);
	
	mutex_unlock(&fts_input_dev->mutex);
	
	return num_read_chars;
}
/************************************************************************
* Name: fts_tpdriver_version_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tpdriver_version_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tprwreg_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tprwreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tprwreg_store
* Brief:  read/write register
* Input: device, device attribute, char buf, char count
* Output: print register value
* Return: char count
***********************************************************************/
static ssize_t fts_tprwreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	long unsigned int wmreg=0;
	u8 regaddr=0xff,regvalue=0xff;
	u8 valbuf[5]={0};

	memset(valbuf, 0, sizeof(valbuf));
	mutex_lock(&fts_input_dev->mutex);	
	num_read_chars = count - 1;
	if (num_read_chars != 2) 
	{
		if (num_read_chars != 4) 
		{
			dev_err(dev, "please input 2 or 4 character\n");
			goto error_return;
		}
	}
	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, &wmreg);
	if (0 != retval) 
	{
		dev_err(dev, "%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n", __FUNCTION__, buf);
		goto error_return;
	}
	if (2 == num_read_chars) 
	{
		/*read register*/
		regaddr = wmreg;
		printk("[focal][test](0x%02x)\n", regaddr);
		if (fts_read_reg(client, regaddr, &regvalue) < 0)
			printk("[Focal] %s : Could not read the register(0x%02x)\n", __func__, regaddr);
		else
			printk("[Focal] %s : the register(0x%02x) is 0x%02x\n", __func__, regaddr, regvalue);
	} 
	else 
	{
		regaddr = wmreg>>8;
		regvalue = wmreg;
		if (fts_write_reg(client, regaddr, regvalue)<0)
			dev_err(dev, "[Focal] %s : Could not write the register(0x%02x)\n", __func__, regaddr);
		else
			dev_dbg(dev, "[Focal] %s : Write 0x%02x into register(0x%02x) successful\n", __func__, regvalue, regaddr);
	}
	error_return:
	mutex_unlock(&fts_input_dev->mutex);
	
	return count;
}
/************************************************************************
* Name: fts_fwupdate_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_fwupdate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

/************************************************************************
* Name: fts_fwupdate_store
* Brief:  upgrade from *.i
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_fwupdate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 uc_host_fm_ver;
	int i_ret;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	#if FTS_ESD_PROTECT
		esd_switch(0);
		apk_debug_flag = 1;
	#endif
	mutex_lock(&fts_input_dev->mutex);
	
	disable_irq(client->irq);
//	#if FTS_ESD_PROTECT
//		apk_debug_flag = 1;
//	#endif
	
	i_ret = fts_ctpm_fw_upgrade_with_i_file(client);
	if (i_ret == 0)
	{
		msleep(300);
		uc_host_fm_ver = fts_ctpm_get_i_file_ver();
		dev_dbg(dev, "%s [FTS] upgrade to new version 0x%x\n", __func__, uc_host_fm_ver);
	}
	else
	{
		dev_err(dev, "%s ERROR:[FTS] upgrade failed ret=%d.\n", __func__, i_ret);
	}
	
//	#if FTS_ESD_PROTECT
//		apk_debug_flag = 0;
//	#endif
	enable_irq(client->irq);
	mutex_unlock(&fts_input_dev->mutex);
	#if FTS_ESD_PROTECT
		esd_switch(1);
		apk_debug_flag = 0;
	#endif
	return count;
}
/************************************************************************
* Name: fts_fwupgradeapp_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_fwupgradeapp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

/************************************************************************
* Name: fts_fwupgradeapp_store
* Brief:  upgrade from app.bin
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_fwupgradeapp_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char fwname[128];
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count-1] = '\0';
	#if FTS_ESD_PROTECT
		esd_switch(0);
		apk_debug_flag = 1;
	#endif
	mutex_lock(&fts_input_dev->mutex);
	
	disable_irq(client->irq);
//	#if FTS_ESD_PROTECT
//				apk_debug_flag = 1;
//			#endif
	fts_ctpm_fw_upgrade_with_app_file(client, fwname);
//	#if FTS_ESD_PROTECT
//				apk_debug_flag = 0;
//			#endif
	enable_irq(client->irq);
	
	mutex_unlock(&fts_input_dev->mutex);
	#if FTS_ESD_PROTECT
		esd_switch(1);
		apk_debug_flag = 0;
	#endif
	return count;
}
/************************************************************************
* Name: fts_ftsgetprojectcode_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_getprojectcode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	
	return -EPERM;
}
/************************************************************************
* Name: fts_ftsgetprojectcode_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_getprojectcode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}


/************************************************************************
* Name: fts_ftsgetprojectcode_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM*
************************************************************************/
static ssize_t fts_touchinfo_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;
	char *ic_version = NULL , *module_vendor = NULL;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	u8 fwver = 0,vendor_id = 0;
	u8 reg_value;
	u8 reg_addr;
	int err;

	//Get Fw Version
	if (fts_read_reg(client, FTS_REG_FW_VER, &fwver) < 0)
		return -1;
	
	if (fwver == 255)
	{
		num_read_chars = snprintf(buf, 128,"get tp fw version fail!\n");
		return -EPERM;
	}
	else
	{
		//num_read_chars = snprintf(buf, 128, "%02X\n", fwver);
	
	}

	//Get module version
	if (fts_read_reg(client, FTS_REG_FW_VENDOR_ID, &vendor_id) < 0)
	{
	    num_read_chars = snprintf(buf, PAGE_SIZE,
					"get TP module vendor fail!\n");
		return -EPERM;
	}
	//printk("vendor_id = %d\n",vendor_id);
	
	if(vendor_id == 0x80) // Eachopto
	    module_vendor = "Eachopto";
	else if(vendor_id == 0x85) // junda
	    module_vendor = "Junda";
	else if(vendor_id == 0x53) // mutto
	    module_vendor = "Mutto";
	else
	    module_vendor = "Unknown";
	

	//Get chip info
	reg_addr = FTS_REG_ID;
	err = fts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0) {
		dev_err(&client->dev, "version read failed");
		return -EPERM;
	}

	if(reg_value == 0x54)
	{
		ic_version = "Ft5346i";
	}
	
	num_read_chars = snprintf(buf, 128, "%s_%s_V%02X\n", ic_version,module_vendor, fwver);

	return num_read_chars;
}
/************************************************************************
* Name: fts_ftsgetprojectcode_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_touchinfo_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

static ssize_t fts_gestureswitch_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int count = 0;
	
    	count +=snprintf(buf + count, PAGE_SIZE - count,"%s\n", 
             fts_gesture_enabled ? "gesture_enable" : "gesture_disable");
		
	return count;
}

static ssize_t fts_gestureswitch_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{

	 struct fts_ts_data *data = dev_get_drvdata(dev);

	 if (NULL == buf)
		 return -EINVAL;
	 
	if (data->suspended) 
		return -EINVAL;
	if (0 == size)
		return 0;
		
	fts_gesture_enabled = 0x00;
		
	if( buf[0] == '0')
	 {
		  device_init_wakeup(&fts_i2c_client->dev, 0);
		 fts_gesture_enabled = 0;
	 }
	 else if(buf[0] == '1')
	 {
		 
		 device_init_wakeup(&fts_i2c_client->dev, 1);
		 fts_gesture_enabled = 1;	
	 }
	 else
	 {
		pr_err("invalid  command! \n");
		return -1;
	 }
	 return size;
	 
}


/************************************************************************
* Name: fts_getregdata_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_getregdata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	 	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
		struct fts_ts_data *data = dev_get_drvdata(dev);
		u8 reg_val[10] = {0};
		u8 auc_i2c_write_buf[10];
		int count = 0;
		int i;
		
		if (data->suspended)
		{
			count +=snprintf(buf + count, PAGE_SIZE - count,"[FTS] touchpanel is suspend\n"); 
			return -EINVAL;
		}

		for(i=0; i < 10 ;i++)
		{
			auc_i2c_write_buf[0] = 0x36 + i;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 10);	
			
			printk("[FTS 0x%x]--reg_val[0]=%02x reg_val[1]=%02x reg_val[2]=%02x reg_val[3]=%02x reg_val[4]=%02x reg_val[5]=%02x reg_val[6]=%02x reg_val[7]=%02x reg_val[8]=%02x reg_val[9]=%02x\n", 
			auc_i2c_write_buf[0],reg_val[0],reg_val[1],reg_val[2],reg_val[3],reg_val[4],reg_val[5],reg_val[6],reg_val[7],reg_val[8],reg_val[9]);

			count +=snprintf(buf + count, PAGE_SIZE - count,"[FTS 0x%x]:reg_val[0]=%02x reg_val[1]=%02x reg_val[2]=%02x reg_val[3]=%02x reg_val[4]=%02x\n           reg_val[5]=%02x reg_val[6]=%02x reg_val[7]=%02x reg_val[8]=%02x reg_val[9]=%02x\n", 
			auc_i2c_write_buf[0],reg_val[0],reg_val[1],reg_val[2],reg_val[3],reg_val[4],reg_val[5],reg_val[6],reg_val[7],reg_val[8],reg_val[9]);
		}
	return count;
}
/************************************************************************
* Name: fts_getregdata_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_getregdata_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}


/************************************************************************
* Name: fts_getirqstate_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_getirqstate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	int count = 0;
	
	count +=snprintf(buf + count, PAGE_SIZE - count,"%s\n", 
             fts_irq_enable ? "IRQ_ENABLE" : "IRQ_DISABLE");

	return count;
		
}
/************************************************************************
* Name: fts_getirqstate_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_getirqstate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);

	 if (NULL == buf)
		 return -EINVAL;
	 
	if (data->suspended) 
		return -EINVAL;
	if (0 == count)
		return 0;
		
	if( buf[0] == '0')
	 {
	 	printk("[FTS] Disable IRQ. \n");
	 	disable_irq(data->client->irq);
		fts_irq_enable = 0;
	 }
	 else if(buf[0] == '1')
	 {
	 	 printk("[FTS] Enable IRQ. \n");
		 enable_irq(data->client->irq);
		 fts_irq_enable = 1;
	 }
	 else
	 {
		pr_err("invalid  command! \n");
		return -1;
	 }
	 return count;
}

/************************************************************************
* Name: fts_getirqstate_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_getDebugMode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	int count = 0;
	
	count +=snprintf(buf + count, PAGE_SIZE - count,"%s\n", 
             FTSDebugMode ? "DEBUG_ENABLE" : "DEBUG_DISABLE");

	return count;
		
}
/************************************************************************
* Name: fts_getirqstate_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_getDebugMode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);

	 if (NULL == buf)
		 return -EINVAL;
	 
	if (data->suspended) 
		return -EINVAL;
	if (0 == count)
		return 0;
		
	if( buf[0] == '0')
	 {
		FTSDebugMode = 0;
	 }
	 else if(buf[0] == '1')
	 {
		FTSDebugMode = 1;
	 }
	 else
	 {
		pr_err("invalid  command! \n");
		return -1;
	 }
	 return count;
}
#ifdef	FTS_TP_GLOVE
#define FTS_TP_GLOVE_SET    0xc1
#define FTS_TP_GLOVE_ENABLE 0x01

/*
	u8 glove_id; 0: close glove function, 1: open glove function
*/
static ssize_t tp_glove_id_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fts_ts_data *data = NULL;
	int ret;

	data = dev_get_drvdata(dev);

	mutex_lock(&data->glove_lock);
	ret = snprintf(buf, 50, "glove_id show:%d\n", data->glove_id);
	mutex_unlock(&data->glove_lock);

    return ret;
}

static ssize_t tp_glove_id_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct fts_ts_data *data = NULL;
	unsigned long val = 0;
	ssize_t ret = -EINVAL;
	char txbuf[2];
	txbuf[0] = FTS_TP_GLOVE_SET;

	
	data = dev_get_drvdata(dev);

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return -EINVAL;

	mutex_lock(&data->glove_lock);
	if (data->suspended) {
		data->glove_id = val;
		mutex_unlock(&data->glove_lock);
		return size;
	}
	


	if ( 0 == val )
	{
		data->glove_id = 0x00;
		txbuf[1] = 0x00;//disable tp glove
		fts_i2c_write(data->client, txbuf, sizeof(txbuf));
	} 
	else if ( 1 == val )
	{
		data->glove_id = 0x01;
		txbuf[1] = FTS_TP_GLOVE_ENABLE;// enable tp glove
		fts_i2c_write(data->client, txbuf, sizeof(txbuf));
	} else {
		pr_err("invalid  command! \n");
		mutex_unlock(&data->glove_lock);
		return -1;
	}
	printk("glove_id = %d \n", data->glove_id);
	mutex_unlock(&data->glove_lock);

	return size;
}

//static DEVICE_ATTR(ftstpgloveid, 0644, tp_glove_id_show, tp_glove_id_store);
static DEVICE_ATTR(ftstpgloveid, S_IRUGO|S_IWUSR, tp_glove_id_show, tp_glove_id_store);

#endif


/****************************************/
/* sysfs */
/*get the fw version
*example:cat ftstpfwver
*/
static DEVICE_ATTR(ftstpfwver, S_IRUGO|S_IWUSR, fts_tpfwver_show, fts_tpfwver_store);

static DEVICE_ATTR(ftstpdriverver, S_IRUGO|S_IWUSR, fts_tpdriver_version_show, fts_tpdriver_version_store);
/*upgrade from *.i
*example: echo 1 > ftsfwupdate
*/
static DEVICE_ATTR(ftsfwupdate, S_IRUGO|S_IWUSR, fts_fwupdate_show, fts_fwupdate_store);
/*read and write register
*read example: echo 88 > ftstprwreg ---read register 0x88
*write example:echo 8807 > ftstprwreg ---write 0x07 into register 0x88
*
*note:the number of input must be 2 or 4.if it not enough,please fill in the 0.
*/
static DEVICE_ATTR(ftstprwreg, S_IRUGO|S_IWUSR, fts_tprwreg_show, fts_tprwreg_store);
/*upgrade from app.bin
*example:echo "*_app.bin" > ftsfwupgradeapp
*/
static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO|S_IWUSR, fts_fwupgradeapp_show, fts_fwupgradeapp_store);
static DEVICE_ATTR(ftsgetprojectcode, S_IRUGO|S_IWUSR, fts_getprojectcode_show, fts_getprojectcode_store);

//show touch info
static DEVICE_ATTR(ftstouchinfo, S_IRUGO|S_IWUSR, fts_touchinfo_show, fts_touchinfo_store);
//show gesture switch
static DEVICE_ATTR(ftsgestureswitch, S_IRUGO|S_IWUSR, fts_gestureswitch_show, fts_gestureswitch_store);

//show reister data
static DEVICE_ATTR(ftstpregdata, S_IRUGO|S_IWUSR, fts_getregdata_show, fts_getregdata_store);

//show irq state
static DEVICE_ATTR(ftstpirqstate, S_IRUGO|S_IWUSR, fts_getirqstate_show, fts_getirqstate_store);


//show irq state
static DEVICE_ATTR(ftstpDebugMode, S_IRUGO|S_IWUSR, fts_getDebugMode_show, fts_getDebugMode_store);




/*add your attr in here*/
static struct attribute *fts_attributes[] = {
	&dev_attr_ftstpfwver.attr,
	&dev_attr_ftstpdriverver.attr,
	&dev_attr_ftsfwupdate.attr,
	&dev_attr_ftstprwreg.attr,
	&dev_attr_ftsfwupgradeapp.attr,
	&dev_attr_ftsgetprojectcode.attr,
	&dev_attr_ftstouchinfo.attr,
	&dev_attr_ftsgestureswitch.attr,
	&dev_attr_ftstpregdata.attr,
	&dev_attr_ftstpirqstate.attr,
	&dev_attr_ftstpDebugMode.attr,
#ifdef	FTS_TP_GLOVE
	&dev_attr_ftstpgloveid.attr,
#endif

	NULL
};

static struct attribute_group fts_attribute_group = {
	.attrs = fts_attributes
};

/************************************************************************
* Name: fts_create_sysfs
* Brief:  create sysfs for debug
* Input: i2c info
* Output: no
* Return: success =0
***********************************************************************/
int fts_create_sysfs(struct i2c_client * client)
{
	int err;
	
	err = sysfs_create_group(&client->dev.kobj, &fts_attribute_group);
	if (0 != err) 
	{
		dev_err(&client->dev, "%s() - ERROR: sysfs_create_group() failed.\n", __func__);
		sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
		return -EIO;
	} 
	else 
	{
		pr_info("fts:%s() - sysfs_create_group() succeeded.\n",__func__);
	}
	return err;
}
/************************************************************************
* Name: fts_remove_sysfs
* Brief:  remove sys
* Input: i2c info
* Output: no
* Return: no
***********************************************************************/
int fts_remove_sysfs(struct i2c_client * client)
{
	sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
	return 0;
}
