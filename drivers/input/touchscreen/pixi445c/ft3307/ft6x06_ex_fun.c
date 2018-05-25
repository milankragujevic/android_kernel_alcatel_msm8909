/*
 *drivers/input/touchscreen/ft5x06_ex_fun.c
 *
 *FocalTech ft6x06 expand function for debug.
 *
 *Copyright (c) 2010  Focal tech Ltd.
 *
 *This software is licensed under the terms of the GNU General Public
 *License version 2, as published by the Free Software Foundation, and
 *may be copied, distributed, and modified under those terms.
 *
 *This program is distributed in the hope that it will be useful,
 *but WITHOUT ANY WARRANTY; without even the implied warranty of
 *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *GNU General Public License for more details.
 *
 *Note:the error code of EIO is the general error in this file.
 */

#include "ft6x06_ex_fun.h"
#include "ft6x06_ts.h"
#include <linux/mount.h>
#include <linux/netdevice.h>
#include <linux/proc_fs.h>

#define ft6x06_i2c_Write ft6x06_i2c_write
#define ft6x06_i2c_Read ft6x06_i2c_read
#define BUFF_VOLUME 512
#define TCT_PIXI445TF
bool ft6336_fw_update_result = false;

struct Upgrade_Info {
	u16 delay_aa;		/*delay of write FT_UPGRADE_AA */
	u16 delay_55;		/*delay of write FT_UPGRADE_55 */
	u8 upgrade_id_1;	/*upgrade id 1 */
	u8 upgrade_id_2;	/*upgrade id 2 */
	u16 delay_readid;	/*delay of read id */
	u16 delay_earse_flash; /*delay of earse flash*/
};

// Modify update firmware
static unsigned char CTPM_FW[] = { 
	//#include "firmware/PIXI4_45_TF_3307_TDT0xe0_Ver0x18_20160126_app.h"
	#include "firmware/PIXI4_45_TF_3307_TDT0xe0_Ver0x19_20160217_app.h"
};


static struct mutex g_device_mutex;
extern bool FT6xx6DebugMode ;

#define FT6XX6_TP_DBG(format, ...) \
{ \
    if(FT6xx6DebugMode) \
    { \
        printk(KERN_INFO "FT6x06 debug " format , ## __VA_ARGS__); \
    } \
}

int fts_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			  u32 dw_lenth);


int ft6x06_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};
	buf[0] = regaddr;
	buf[1] = regvalue;

	return ft6x06_i2c_write(client, buf, sizeof(buf));
}


int ft6x06_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return ft6x06_i2c_read(client, &regaddr, 1, regvalue, 1);
}


int fts_ctpm_auto_clb_6x06(struct i2c_client *client)
{
	unsigned char uc_temp = 0x00;
	unsigned char i = 0;

	/*start auto CLB */
	msleep(200);

	ft6x06_write_reg(client, 0, FTS_FACTORYMODE_VALUE);
	/*make sure already enter factory mode */
	msleep(100);
	/*write command to start calibration */
	ft6x06_write_reg(client, 2, 0x4);
	msleep(300);
	for (i = 0; i < 100; i++) {
		ft6x06_read_reg(client, 0, &uc_temp);
		/*return to normal mode, calibration finish */
		if (0x0 == ((uc_temp & 0x70) >> 4))
			break;
	}

	msleep(200);
	/*calibration OK */
	msleep(300);
	ft6x06_write_reg(client, 0, FTS_FACTORYMODE_VALUE);	/*goto factory mode for store */
	msleep(100);	/*make sure already enter factory mode */
	ft6x06_write_reg(client, 2, 0x5);	/*store CLB result */
	msleep(300);
	ft6x06_write_reg(client, 0, FTS_WORKMODE_VALUE);	/*return to normal mode */
	msleep(300);

	/*store CLB result OK */
	return 0;
}

/*
upgrade with *.i file
*/
int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client *client)
{
	u8 *pbt_buf = NULL;
	int i_ret;
	int fw_len = sizeof(CTPM_FW);

	/*judge the fw that will be upgraded
	* if illegal, then stop upgrade and return.
	*/
	if (fw_len < 8 || fw_len > 32 * 1024) {
		dev_err(&client->dev, "%s:FW length error\n", __func__);
		return -EIO;
	}

	/*FW upgrade */
	pbt_buf = CTPM_FW;
	/*call the upgrade function */
	i_ret = fts_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
	if (i_ret != 0)
		dev_err(&client->dev, "%s:upgrade failed. err.\n",
					__func__);
	
	return i_ret;
}

u8 fts_ctpm_get_i_file_ver(void)
{
	u16 ui_sz;
	ui_sz = sizeof(CTPM_FW);
	if (ui_sz > 2)
		return CTPM_FW[0x10a];

	return 0x00;	/*default value */
}

/*update project setting
*only update these settings for COB project, or for some special case
*/
int fts_ctpm_update_project_setting_6x06(struct i2c_client *client)
{
	u8 uc_i2c_addr;	/*I2C slave address (7 bit address)*/
	u8 uc_io_voltage;	/*IO Voltage 0---3.3v;	1----1.8v*/
	u8 uc_panel_factory_id;	/*TP panel factory ID*/
	u8 buf[FTS_SETTING_BUF_LEN];
	u8 reg_val[2] = {0};
	u8 auc_i2c_write_buf[10] = {0};
	u8 packet_buf[FTS_SETTING_BUF_LEN + 6];
	u32 i = 0;
	int i_ret;

	uc_i2c_addr = client->addr;
	uc_io_voltage = 0x0;
	uc_panel_factory_id = 0x5a;


	/*Step 1:Reset  CTPM
	*write 0xaa to register 0xfc
	*/
	ft6x06_write_reg(client, 0xfc, 0xaa);
	msleep(50);

	/*write 0x55 to register 0xfc */
	ft6x06_write_reg(client, 0xfc, 0x55);
	msleep(30);

	/*********Step 2:Enter upgrade mode *****/
	auc_i2c_write_buf[0] = 0x55;
	auc_i2c_write_buf[1] = 0xaa;
	do {
		i++;
		i_ret = ft6x06_i2c_write(client, auc_i2c_write_buf, 2);
		msleep(5);
	} while (i_ret <= 0 && i < 5);


	/*********Step 3:check READ-ID***********************/
	auc_i2c_write_buf[0] = 0x90;
	auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;

	ft6x06_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
	msleep(5);
    printk("\n[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
			 reg_val[0], reg_val[1]);
	//if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
	if (reg_val[0] == 0x79 && reg_val[1] == 0x18)
		dev_dbg(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
			 reg_val[0], reg_val[1]);
	else
		return -EIO;

	auc_i2c_write_buf[0] = 0xcd;
	ft6x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	dev_dbg(&client->dev, "bootloader version = 0x%x\n", reg_val[0]);

	/*--------- read current project setting  ---------- */
	/*set read start address */
	buf[0] = 0x3;
	buf[1] = 0x0;
	buf[2] = 0x78;
	buf[3] = 0x0;

	ft6x06_i2c_read(client, buf, 4, buf, FTS_SETTING_BUF_LEN);
	dev_dbg(&client->dev, "[FTS] old setting: uc_i2c_addr = 0x%x,\
			uc_io_voltage = %d, uc_panel_factory_id = 0x%x\n",
			buf[0], buf[2], buf[4]);

	 /*--------- Step 4:erase project setting --------------*/
	auc_i2c_write_buf[0] = 0x63;
	ft6x06_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(100);

	/*----------  Set new settings ---------------*/
	buf[0] = uc_i2c_addr;
	buf[1] = ~uc_i2c_addr;
	buf[2] = uc_io_voltage;
	buf[3] = ~uc_io_voltage;
	buf[4] = uc_panel_factory_id;
	buf[5] = ~uc_panel_factory_id;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	packet_buf[2] = 0x78;
	packet_buf[3] = 0x0;
	packet_buf[4] = 0;
	packet_buf[5] = FTS_SETTING_BUF_LEN;

	for (i = 0; i < FTS_SETTING_BUF_LEN; i++)
		packet_buf[6 + i] = buf[i];

	ft6x06_i2c_write(client, packet_buf, FTS_SETTING_BUF_LEN + 6);
	msleep(100);

	/********* reset the new FW***********************/
	auc_i2c_write_buf[0] = 0x07;
	ft6x06_i2c_write(client, auc_i2c_write_buf, 1);

	msleep(200);
	return 0;
}

int fts_ctpm_auto_upgrade(struct i2c_client *client)
{
	u8 uc_host_fm_ver = FT6x06_REG_FW_VER;
	u8 uc_tp_fm_ver,vendor_id;
	int i,i_ret = -1;
#if 1
    u8 tp_project_id;
    char *tp_project_reg = NULL;
	char *tp_vendor_reg = NULL;
#endif
	// Modify by firmware update
	unsigned char version_list[] = {0x00,0xA,0xe0,0x0b,0x11,0x12,0x13,0x15,0x16,0x17,0x18,0x19};

	ft6x06_read_reg(client, FT6x06_REG_FW_VER, &uc_tp_fm_ver);
	uc_host_fm_ver = fts_ctpm_get_i_file_ver();
    printk(KERN_ERR "%s uc_tp_fm_ver = 0x%x uc_host_fm_ver = 0x%x \n",__func__,uc_tp_fm_ver,uc_host_fm_ver);
#if 1
    //1.read project reg
    if(uc_tp_fm_ver == 0)
    {
        printk(KERN_INFO "the uc_tp_fm_ver is NULL and the fw maybe has been destroied,try to recovery fw\n");
        goto update_firmware;
    }

	/* 2. Get tp module vendor id: 0x80(Eachopto)/0x85(junda)/0x53(mutto)
	 * FT6336 is use 0x80(Eachopto)
	 * FT6436 is use 0x85(junda)
	 * now we just use ft6336 update firmware
	 */
	ft6x06_read_reg(client, FT_REG_VENDOR_ID, &vendor_id);
	switch(vendor_id)
	{
		case 0x80:
			tp_vendor_reg = "Eachopto";
			break;
		case 0x85:
			tp_vendor_reg = "Junda";
			break;
		case 0x53:
			tp_vendor_reg = "Mutto";
			break;
		case 0xE0:
			tp_vendor_reg = "TDT";
		default:
			tp_vendor_reg = "Unknow";
			break;
	}
	printk(KERN_ERR "The TP vendor is %s,the default vendor is Eachopto \n",tp_vendor_reg);
	if(vendor_id != 0xE0)
	{
		printk(KERN_ERR "The TP module vendor is not match!\n");
		printk(KERN_ERR "Go to normal boot!\n");
        return -EIO; 
	}

	/* 3.Get project id
	 * if the project id is not match,cann't update firmware
	 */
    ft6x06_read_reg(client, FT6x06_REG_PROJECT_ID, &tp_project_id);
	printk("TP project ID : 0x%x", tp_project_id);
    switch(tp_project_id)
    {
        case 0x30:
            tp_project_reg = "PIXI343GEVDO";
            break;
        case 0x31:
            tp_project_reg = "PIXI344G";
            break;
        case 0x32:
            tp_project_reg = "PIXI3454GSPR";
            break;
        default:
			printk(KERN_INFO "The tp_project_reg is not defined\n");
			break;	
    }
    //2.when the project match the TP project reg , it try to update the firmware . 
   //other condition not try to update the firmware
	printk(KERN_ERR "The TP reg is %s,the project is PIXI3454GSPR \n",tp_project_reg);

#if 0
	if(tp_project_id != 0x32)
    {
        printk(KERN_ERR "Project is not match the TP!");
        return -EIO;  
    } 
#endif 

update_firmware:
#endif
    for (i = 0; i < sizeof(version_list)/sizeof(version_list[0]); i++)
    {
    	printk("[FTS] version_list[%d] = %x, uc_tp_fm_ver = %x\n",
			i, version_list[i],uc_tp_fm_ver);
        if (uc_tp_fm_ver == version_list[i])
        {
          if(uc_tp_fm_ver < uc_host_fm_ver)
          	{
		    msleep(100);
		    dev_dbg(&client->dev, "[FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",
			     uc_tp_fm_ver, uc_host_fm_ver);
			printk("[FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",
			     uc_tp_fm_ver, uc_host_fm_ver);
		    i_ret = fts_ctpm_fw_upgrade_with_i_file(client);
		    if (i_ret == 0)	
            {
			    msleep(300);
			    uc_host_fm_ver = fts_ctpm_get_i_file_ver();
			    dev_dbg(&client->dev, "[FTS] upgrade to new version 0x%x\n",
				uc_host_fm_ver);
		    	printk("\n%s :[FTS] upgrade to new version 0x%x\n",__func__,uc_host_fm_ver);
		    } 
            else 
            {
		     pr_err("[FTS] upgrade failed ret=%d.\n", i_ret);
		     return -EIO;
		    }
		    return 0;
          	}
		  else{
			printk("###the FW version in boot is older than phone's,don't need to updata#########\n");
			return 0;
		  }
	    }
    }
    if(i == sizeof(version_list)/sizeof(version_list[0]))
        printk(KERN_ERR " it don't need to upgrade to new version! \n");
	return 0;
}

void delay_qt_ms(unsigned long  w_ms)
{
	unsigned long i;
	unsigned long j;

	for (i = 0; i < w_ms; i++)
	{
		for (j = 0; j < 1000; j++)
		{
			 udelay(1);
		}
	}
}

int fts_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			  u32 dw_lenth)
{
	u8 reg_val[2] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u32 fw_length;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	//int i_ret;


	if(pbt_buf[0] != 0x02)
	{
		DBG("[FTS] FW first byte is not 0x02. so it is invalid \n");
		return -1;
	}

	if(dw_lenth > 0x11f)
	{
		fw_length = ((u32)pbt_buf[0x100]<<8) + pbt_buf[0x101];
		if(dw_lenth < fw_length)
		{
			DBG("[FTS] Fw length is invalid \n");
			return -1;
		}
	}
	else
	{
		DBG("[FTS] Fw length is invalid \n");
		return -1;
	}
	//DBG("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
	printk("\n%s",__func__);
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		/*write 0xaa to register 0xbc */
		
		ft6x06_write_reg(client, 0xbc, FT_UPGRADE_AA);
		msleep(FT6X06_UPGRADE_AA_DELAY);

		/*write 0x55 to register 0xbc */
		ft6x06_write_reg(client, 0xbc, FT_UPGRADE_55);

		msleep(FT6X06_UPGRADE_55_DELAY);

		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		ft6x06_i2c_write(client, auc_i2c_write_buf, 1);

		auc_i2c_write_buf[0] = FT_UPGRADE_AA;
		ft6x06_i2c_write(client, auc_i2c_write_buf, 1);
		msleep(FT6X06_UPGRADE_READID_DELAY);

		/*********Step 3:check READ-ID***********************/		
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;
		reg_val[0] = 0x00;
		reg_val[1] = 0x00;
		ft6x06_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
        printk("\n[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);

		if (reg_val[0] == FT6X06_UPGRADE_ID_1
			&& reg_val[1] == FT6X06_UPGRADE_ID_2) {
			//dev_dbg(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				//reg_val[0], reg_val[1]);
			DBG("[FTS] Step 3: GET CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev, "[FTS] Step 3: GET CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;

	auc_i2c_write_buf[0] = 0x90;
	auc_i2c_write_buf[1] = 0x00;
	auc_i2c_write_buf[2] = 0x00;
	auc_i2c_write_buf[3] = 0x00;
	auc_i2c_write_buf[4] = 0x00;
	ft6x06_i2c_Write(client, auc_i2c_write_buf, 5);
	
	//auc_i2c_write_buf[0] = 0xcd;
	//ft6x06_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);


	/*Step 4:erase app and panel paramenter area*/
	DBG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = 0x61;
	ft6x06_i2c_Write(client, auc_i2c_write_buf, 1);	/*erase app area */
	msleep(FT6X06_UPGRADE_EARSE_DELAY);

	for(i = 0;i < 200;i++)
	{
		auc_i2c_write_buf[0] = 0x6a;
		auc_i2c_write_buf[1] = 0x00;
		auc_i2c_write_buf[2] = 0x00;
		auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = 0x00;
		reg_val[1] = 0x00;
		ft6x06_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if(0xb0 == reg_val[0] && 0x02 == reg_val[1])
		{
			DBG("[FTS] erase app finished \n");
			break;
		}
		msleep(50);
	}

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	DBG("Step 5:write firmware(FW) to ctpm flash\n");

	dw_lenth = fw_length;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++) {
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;

		for (i = 0; i < FTS_PACKET_LENGTH; i++) {
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		
		ft6x06_i2c_Write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		
		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			auc_i2c_write_buf[1] = 0x00;
			auc_i2c_write_buf[2] = 0x00;
			auc_i2c_write_buf[3] = 0x00;
			reg_val[0] = 0x00;
			reg_val[1] = 0x00;
			ft6x06_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
			if(0xb0 == (reg_val[0] & 0xf0) && (0x03 + (j % 0x0ffd)) == (((reg_val[0] & 0x0f) << 8) |reg_val[1]))
			{
				//DBG("[FTS] write a block data finished \n");
				msleep(1);
				break;
			}
			msleep(1);
		}
		//msleep(FTS_PACKET_LENGTH / 6 + 1);
		//DBG("write bytes:0x%04x\n", (j+1) * FTS_PACKET_LENGTH);
		//delay_qt_ms(FTS_PACKET_LENGTH / 6 + 1);
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		ft6x06_i2c_Write(client, packet_buf, temp + 6);

		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			auc_i2c_write_buf[1] = 0x00;
			auc_i2c_write_buf[2] = 0x00;
			auc_i2c_write_buf[3] = 0x00;
			reg_val[0] = 0x00;
			reg_val[1] = 0x00;
			ft6x06_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
			if(0xb0 == (reg_val[0] & 0xf0) && (0x03 + (j % 0x0ffd)) == (((reg_val[0] & 0x0f) << 8) |reg_val[1]))
			{
				//DBG("[FTS] write a block data finished \n");
				msleep(1);
				break;
			}
			msleep(1);
		}
		//msleep(20);
	}


	/*********Step 6: read out checksum***********************/
	/*send the opration head */
	DBG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0xcc;
	ft6x06_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) {
		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
					reg_val[0],
					bt_ecc);
		return -EIO;
	}

	/*********Step 7: reset the new FW***********************/
	DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	ft6x06_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(300);	/*make sure CTP startup normally */

	return 0;
}

/*sysfs debug*/

/*
*get firmware size

@firmware_name:firmware name
*note:the firmware default path is sdcard.
	if you want to change the dir, please modify by yourself.
*/
static int ft6x06_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));

	sprintf(filepath, "/sdcard/%s", firmware_name);

	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}



/*
*read firmware buf for .bin file.

@firmware_name: fireware name
@firmware_buf: data buf of fireware

note:the firmware default path is sdcard.
	if you want to change the dir, please modify by yourself.
*/
static int ft6x06_ReadFirmware(char *firmware_name,
			       unsigned char *firmware_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "/sdcard/%s", firmware_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, firmware_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}



/*
upgrade with *.bin file
*/

int fts_ctpm_fw_upgrade_with_app_file(struct i2c_client *client,
				       char *firmware_name)
{
	u8 *pbt_buf = NULL;
	int i_ret;
	int fwsize = ft6x06_GetFirmwareSize(firmware_name);

	if (fwsize <= 0) {
		dev_err(&client->dev, "%s ERROR:Get firmware size failed\n",
					__func__);
		return -EIO;
	}

	if (fwsize < 8 || fwsize > 32 * 1024) {
		dev_dbg(&client->dev, "%s:FW length error\n", __func__);
		return -EIO;
	}
	
	/*=========FW upgrade========================*/
	pbt_buf = kmalloc(fwsize + 1, GFP_ATOMIC);
	if (pbt_buf == NULL) {
		dev_err(&client->dev, "TP FW : malloc failed\n");
		return -EIO;
	}
	if (ft6x06_ReadFirmware(firmware_name, pbt_buf)) {
		dev_err(&client->dev, "%s() - ERROR: request_firmware failed\n",
					__func__);
		kfree(pbt_buf);
		return -EIO;
	}
	
	/*call the upgrade function */
	i_ret = fts_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	if (i_ret != 0)
		dev_err(&client->dev, "%s() - ERROR:[FTS] upgrade failed..\n",
					__func__);
	//else
		//fts_ctpm_auto_clb(client);
	kfree(pbt_buf);

	return i_ret;
}

static ssize_t ft6x06_tpfwver_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t num_read_chars = 0;
	u8 chip_id = 0,vendor_id = 0,fwver = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	char *module_vendor,*IC_vendor;
	u8 tp_project_id;
	char *tp_project_reg;

	mutex_lock(&g_device_mutex);
	//add by junfeng.zhou for add the TP_INFO begin .
    //get TP IC vendor reg
    if (ft6x06_read_reg(client, FT_REG_ID, &chip_id) < 0)
    {
	   num_read_chars = snprintf(buf, PAGE_SIZE,
		"get TP IC vendor fail!\n");
		goto error;
	}
					
	//get TP module vendor reg
	if (ft6x06_read_reg(client, FT_REG_VENDOR_ID, &vendor_id) < 0)
	{
	    num_read_chars = snprintf(buf, PAGE_SIZE,
					"get TP module vendor fail!\n");
		goto error;
	}
					
	//get TP FW version reg				
	if (ft6x06_read_reg(client, FT6x06_REG_FW_VER, &fwver) < 0)
	{
		num_read_chars = snprintf(buf, PAGE_SIZE,
					"get tp fw version fail!\n");
		goto error;
	}
	//else
	//get TP project reg				
	if (ft6x06_read_reg(client, FT6x06_REG_PROJECT_ID, &tp_project_id) < 0)
	{
		num_read_chars = snprintf(buf, PAGE_SIZE,
					"get tp fw version fail!\n");
		goto error;
	}
	switch(tp_project_id)
    {
        case 0x30:
            tp_project_reg = "PIXI343GEVDO";
            break;
        case 0x31:
            tp_project_reg = "PIXI344G";
            break;
        case 0x32:
            tp_project_reg = "PIXI3454GSPR";
            break;
        default:
            tp_project_reg = "NULL";
			printk(KERN_INFO "The tp_project_reg is not defined");
			break;	
    }
	printk(KERN_INFO "chip_id = 0x%02X,vendor_id = 0x%02X,fwver = 0x%02X",chip_id,vendor_id ,fwver);
	if(chip_id == 0x36) //ft6x36
	    IC_vendor = "FT6336";
	else if(chip_id == 0x0E)
		IC_vendor = "FT3307";
	else 
		IC_vendor = "Unknow";
	if(vendor_id == 0x80) // Eachopto
	    module_vendor = "Eachopto";
	else if(vendor_id == 0x85) // junda
	    module_vendor = "Junda";
	else if(vendor_id == 0x53) // mutto
	    module_vendor = "Mutto";
	else if(vendor_id == 0xE0)
		module_vendor = "TDT"; //TCL 
	else
	    module_vendor = "Unknown";
	num_read_chars = snprintf(buf, PAGE_SIZE, "%s_%s_V0x%x\n",IC_vendor,module_vendor,fwver);
		
    //add by junfeng.zhou for add the TP_INFO end . 

error:
	mutex_unlock(&g_device_mutex);

	return num_read_chars;
}

static ssize_t ft6x06_tpfwver_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}



static ssize_t ft6x06_tprwreg_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/*place holder for future use*/
	return -EPERM;
}

static ssize_t ft6x06_tprwreg_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	long unsigned int wmreg = 0;
	u8 regaddr = 0xff, regvalue = 0xff;
	u8 valbuf[5] = {0};

	memset(valbuf, 0, sizeof(valbuf));
	mutex_lock(&g_device_mutex);
	num_read_chars = count - 1;

	if (num_read_chars != 2) {
		if (num_read_chars != 4) {
			pr_info("please input 2 or 4 character\n");
			goto error_return;
		}
	}

	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, &wmreg);

	if (0 != retval) {
		dev_err(&client->dev, "%s() - ERROR: Could not convert the "\
						"given input to a number." \
						"The given input was: \"%s\"\n",
						__func__, buf);
		goto error_return;
	}

	if (2 == num_read_chars) {
		/*read register*/
		regaddr = wmreg;
		if (ft6x06_read_reg(client, regaddr, &regvalue) < 0)
			dev_err(&client->dev, "Could not read the register(0x%02x)\n",
						regaddr);
		else
			pr_info("the register(0x%02x) is 0x%02x\n",
					regaddr, regvalue);
	} else {
		regaddr = wmreg >> 8;
		regvalue = wmreg;
		if (ft6x06_write_reg(client, regaddr, regvalue) < 0)
			dev_err(&client->dev, "Could not write the register(0x%02x)\n",
							regaddr);
		else
			dev_err(&client->dev, "Write 0x%02x into register(0x%02x) successful\n",
							regvalue, regaddr);
	}

error_return:
	mutex_unlock(&g_device_mutex);

	return count;
}

static ssize_t ft6x06_fwupdate_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/* place holder for future use */
	printk("*** %s()***\n", __func__);

    return sprintf(buf, "%d\n", ft6336_fw_update_result);
}

/*upgrade from *.i*/
static ssize_t ft6x06_fwupdate_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct ft6x06_ts_data *data = NULL;
	u8 uc_host_fm_ver;
	int i_ret;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	ft6336_fw_update_result = false;

	data = (struct ft6x06_ts_data *)i2c_get_clientdata(client);

	mutex_lock(&g_device_mutex);

	disable_irq(client->irq);
	i_ret = fts_ctpm_fw_upgrade_with_i_file(client);
	if (i_ret == 0) {
		msleep(300);
		uc_host_fm_ver = fts_ctpm_get_i_file_ver();
		ft6336_fw_update_result = true;
		pr_info("%s [FTS] upgrade to new version 0x%x\n", __func__,
					 uc_host_fm_ver);
	} else
		dev_err(&client->dev, "%s ERROR:[FTS] upgrade failed.\n",
					__func__);

	enable_irq(client->irq);
	mutex_unlock(&g_device_mutex);

	return count;
}

static ssize_t ft6x06_fwupgradeapp_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/*place holder for future use*/
	return -EPERM;
}


/*upgrade from app.bin*/
static ssize_t ft6x06_fwupgradeapp_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char fwname[128];
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count - 1] = '\0';

	mutex_lock(&g_device_mutex);
	disable_irq(client->irq);

	fts_ctpm_fw_upgrade_with_app_file(client, fwname);

	enable_irq(client->irq);
	mutex_unlock(&g_device_mutex);

	return count;
}


//Debug
ssize_t FT6xx6_Debug_Show(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{

    printk("*** %s()***\n", __func__);

    return sprintf(pBuf, "%d\n", FT6xx6DebugMode);
}

ssize_t FT6xx6_Debug_Store(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
	if( pBuf[0] == '0')
	{
	  FT6xx6DebugMode  = false;
	}
	else if(pBuf[0]=='1')
	{
		FT6xx6DebugMode  = true;
	}
	else
	{
	   pr_err("invalid	command! \n");
	   return -1;
	}

	printk("*** %s() FT6xx6DebugMode = %d ***\n", __func__,FT6xx6DebugMode);

    return nSize;
}


static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR, FT6xx6_Debug_Show,
			FT6xx6_Debug_Store);


/*sysfs */
/*get the fw version
*example:cat ftstpfwver
*/
static DEVICE_ATTR(ftstpfwver, S_IRUGO | S_IWUSR, ft6x06_tpfwver_show,
			ft6x06_tpfwver_store);

/*upgrade from *.i
*example: echo 1 > ftsfwupdate
*/
static DEVICE_ATTR(ftsfwupdate, S_IRUGO | S_IWUSR, ft6x06_fwupdate_show,
			ft6x06_fwupdate_store);

/*read and write register
*read example: echo 88 > ftstprwreg ---read register 0x88
*write example:echo 8807 > ftstprwreg ---write 0x07 into register 0x88
*
*note:the number of input must be 2 or 4.if it not enough,please fill in the 0.
*/
static DEVICE_ATTR(ftstprwreg, S_IRUGO | S_IWUSR, ft6x06_tprwreg_show,
			ft6x06_tprwreg_store);


/*upgrade from app.bin
*example:echo "*_app.bin" > ftsfwupgradeapp
*/
static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO | S_IWUSR, ft6x06_fwupgradeapp_show,
			ft6x06_fwupgradeapp_store);


/*add your attr in here*/
static struct attribute *ft6x06_attributes[] = {
	&dev_attr_debug.attr,
	&dev_attr_ftstpfwver.attr,
	&dev_attr_ftsfwupdate.attr,
	&dev_attr_ftstprwreg.attr,
	&dev_attr_ftsfwupgradeapp.attr,
	NULL
};

static struct attribute_group ft6x06_attribute_group = {
	.attrs = ft6x06_attributes
};

/*create sysfs for debug*/
int ft6x06_create_sysfs(struct i2c_client *client)
{
	int err;
	err = sysfs_create_group(&client->dev.kobj, &ft6x06_attribute_group);
	if (0 != err) {
		dev_err(&client->dev,
					 "%s() - ERROR: sysfs_create_group() failed.\n",
					 __func__);
		sysfs_remove_group(&client->dev.kobj, &ft6x06_attribute_group);
		return -EIO;
	} else {
		mutex_init(&g_device_mutex);
		pr_info("ft6x06:%s() - sysfs_create_group() succeeded.\n",
				__func__);
	}
	return err;
}

void ft6x06_release_sysfs(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &ft6x06_attribute_group);
	mutex_destroy(&g_device_mutex);
}

/*create apk debug channel*/
#define PROC_UPGRADE			0
#define PROC_READ_REGISTER		1
#define PROC_WRITE_REGISTER		2
#define PROC_RAWDATA			3
#define PROC_AUTOCLB			4
#define PROC_UPGRADE_INFO		5
#define PROC_WRITE_DATA			6
#define PROC_READ_DATA			7


#define PROC_NAME	"ft5x0x-debug"
static unsigned char proc_operate_mode = PROC_UPGRADE;
static struct proc_dir_entry *ft6x06_proc_entry;
static int g_upgrade_successful = 0;
struct i2c_client *g_client = NULL;

/*interface of write proc*/
//static int ft6x06_debug_write(struct file *filp, 
//	const char __user *buff, unsigned long len, void *data)
static int32_t ft6x06_debug_write(struct file *filp, const char __user *userbuf,
		size_t count, loff_t *ppos)
{
	struct i2c_client *client/* = (struct i2c_client *)(ft6x06_proc_entry->data)*/;
	//struct i2c_client *client = PDE_DATA(ft6x06_proc_entry);
	unsigned char writebuf[FTS_PACKET_LENGTH];
	//int buflen = len;
	int buflen = count;
	int writelen = 0;
	int ret = 0;
    if(g_client != NULL)
	    client = g_client;
    else
    {
        printk(KERN_ERR "i2c_client is NULL");
        return 0;
    }
	//if (copy_from_user(&writebuf, buff, buflen)) {
	if (copy_from_user(&writebuf, userbuf, buflen)) {
		dev_err(&client->dev, "%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];
	FT6XX6_TP_DBG("*** %s *** proc_operate_mode = %d\n", __func__,proc_operate_mode);
   // printk("%s proc_operate_mode = %d\n",__func__,proc_operate_mode);
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			DBG("%s: PROC_UPGRADE \n", __func__);
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			DBG("%s\n", upgrade_file_path);
			disable_irq(client->irq);

			ret = fts_ctpm_fw_upgrade_with_app_file(client, upgrade_file_path);

			enable_irq(client->irq);
			if (ret < 0) {
				dev_err(&client->dev, "%s:upgrade failed.\n", __func__);
				return ret;
			}
			else
			{
				g_upgrade_successful = 1;
			}
		}
		break;
	case PROC_READ_REGISTER:
		FT6XX6_TP_DBG("%s: PROC_READ_REGISTER \n", __func__);
		writelen = 1;
		ret = ft6x06_i2c_Write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		FT6XX6_TP_DBG("%s: PROC_READ_DATA \n", __func__);
		writelen = 2;
		ret = ft6x06_i2c_Write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_RAWDATA:
		break;
	case PROC_AUTOCLB:
		FT6XX6_TP_DBG("%s: autoclb\n", __func__);
		fts_ctpm_auto_clb_6x06(client);
		break;
	case PROC_UPGRADE_INFO:
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		FT6XX6_TP_DBG("%s: PROC_READ_DATA/PROC_WRITE_DATA \n", __func__);
		writelen = count - 1;
		ret = ft6x06_i2c_Write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	default:
		break;
	}
	return count;
}

/*interface of read proc*/
//static int ft6x06_debug_read( char *page, char **start,
//	off_t off, int count, int *eof, void *data )
static int32_t ft6x06_debug_read(struct file *file, char __user *user_buf,
         size_t count, loff_t *ppos)
{
	struct i2c_client *client/* = (struct i2c_client *)(ft6x06_proc_entry->data)*/;
	//struct i2c_client *client = PDE_DATA(ft6x06_proc_entry);
	int ret = 0;
	//unsigned char buf[PAGE_SIZE];
	//unsigned char buf[1024-8];
	unsigned char buf[BUFF_VOLUME];
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	//int desc = 0;
	if(g_client != NULL)
	    client = g_client;
    else
    {
        printk(KERN_ERR "i2c_client is NULL");
        return 0;
    }
	//printk("%s proc_operate_mode = %d\n",__func__,proc_operate_mode);
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		/*after calling ft5x0x_debug_write to upgrade*/
		regaddr = 0xA6;
		ret = ft6x06_read_reg(client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = ft6x06_i2c_Read(client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:read iic error\n", __func__);
			return ret;
		} 
	//	printk("%s buf = %d \n",__func__,*buf);
		num_read_chars = 1;
		break;
	case PROC_RAWDATA:
		break;
	case PROC_UPGRADE_INFO:
		if(1 == g_upgrade_successful)
			buf[0] = 1;
		else
			buf[0] = 0;
		//dev_dbg("%s:value=0x%02x\n", __func__, raw_buf[0]);
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = ft6x06_i2c_Read(client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:read iic error\n", __func__);
			return ret;
		}
		
		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}
	//printk("%s num_read_chars = %d \n",__func__,num_read_chars);
	//memcpy(page, buf, num_read_chars);
	ret = simple_read_from_buffer(user_buf, count, ppos,
		buf, num_read_chars);
	return ret;
}

static const struct file_operations ft6x06_proc_t_fops = {
	.write = ft6x06_debug_write,
	.read = ft6x06_debug_read,
	.open = simple_open,
	.owner = THIS_MODULE,
};

int ft6x06_create_apk_debug_channel(struct i2c_client * client)
{
	/*ft6x06_proc_entry = create_proc_entry(PROC_NAME, 0777, NULL);
	if (NULL == ft6x06_proc_entry) {
		dev_err(&client->dev, "Couldn't create proc entry!\n");
		return -ENOMEM;
	} else {
		dev_info(&client->dev, "Create proc entry success!\n");
		ft6x06_proc_entry->data = client;
		ft6x06_proc_entry->write_proc = ft6x06_debug_write;
		ft6x06_proc_entry->read_proc = ft6x06_debug_read;
	}*/
	ft6x06_proc_entry = proc_create_data(PROC_NAME,
			/*S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP*/0777,
			ft6x06_proc_entry,
			&ft6x06_proc_t_fops,(void *)client);
	if (ft6x06_proc_entry == NULL) {
		dev_err(&client->dev, "Couldn't create proc entry!");
		return 0;
	}
	//ft6x06_proc_entry->data = (void *)client;
	g_client = client;
	return 0;
}

void ft6x06_release_apk_debug_channel(void)
{
	if (ft6x06_proc_entry)
		remove_proc_entry(PROC_NAME, NULL);
}

