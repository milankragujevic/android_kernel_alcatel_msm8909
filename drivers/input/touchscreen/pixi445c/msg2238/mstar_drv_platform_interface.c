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
 * @file    mstar_drv_platform_interface.c
 *
 * @brief   This file defines the interface of touch screen
 *
 *
 */

/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include "mstar_drv_platform_interface.h"
#include "mstar_drv_main.h"
#include "mstar_drv_ic_fw_porting_layer.h"
#include "mstar_drv_platform_porting_layer.h"
#include "mstar_drv_utility_adaption.h"

#ifdef CONFIG_ENABLE_HOTKNOT
#include "mstar_drv_hotknot.h"
#endif //CONFIG_ENABLE_HOTKNOT
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
#include <linux/sensors.h>
#endif//CONFIG_ENABLE_PROXIMITY_DETECTION


/*=============================================================*/
// EXTERN VARIABLE DECLARATION
/*=============================================================*/

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern u32 g_GestureWakeupMode[2];
extern u8 g_GestureWakeupFlag;
extern struct i2c_client *g_I2cClient;

#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
extern u8 g_GestureDebugFlag;
extern u8 g_GestureDebugMode;
#endif //CONFIG_ENABLE_GESTURE_DEBUG_MODE

#endif //CONFIG_ENABLE_GESTURE_WAKEUP

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
extern u8 g_EnableTpProximity;
extern struct mutex g_Mutex;
extern u32 SLAVE_I2C_ID_DWI2C;
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION

#ifdef CONFIG_ENABLE_GLOVE_MODE
extern u8 g_IsEnableGloveMode;
#endif //CONFIG_ENABLE_GLOVE_MODE

extern u8 g_IsUpdateFirmware;

/*=============================================================*/
// GLOBAL VARIABLE DEFINITION
/*=============================================================*/

extern struct input_dev *g_InputDevice;
extern u8 g_IsUpdateFirmware;

#ifdef CONFIG_ENABLE_HOTKNOT
extern u8 g_HotKnotState;
extern u32 SLAVE_I2C_ID_DWI2C;
#endif //CONFIG_ENABLE_HOTKNOT

#ifdef CONFIG_ENABLE_CHARGER_DETECTION
extern u8 g_ForceUpdate;
#endif //CONFIG_ENABLE_CHARGER_DETECTION

extern u8 IS_FIRMWARE_DATA_LOG_ENABLED;

/*=============================================================*/
// LOCAL VARIABLE DEFINITION
/*=============================================================*/

#ifdef CONFIG_ENABLE_HOTKNOT
static u8 _gAMStartCmd[4] = {HOTKNOT_SEND, ADAPTIVEMOD_BEGIN, 0, 0};
#endif //CONFIG_ENABLE_HOTKNOT
//add by mike.li for suspend & resume.[2015.07.07]
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
struct notifier_block TPfb_notifier;
#endif

/*=============================================================*/
// GLOBAL FUNCTION DEFINITION
/*=============================================================*/

//#ifdef CONFIG_ENABLE_NOTIFIER_FB
#if 0
int MsDrvInterfaceTouchDeviceFbNotifierCallback(struct notifier_block *pSelf, unsigned long nEvent, void *pData)
{
    struct fb_event *pEventData = pData;
    int *pBlank;

    if (pEventData && pEventData->data && nEvent == FB_EVENT_BLANK)
    {
        pBlank = pEventData->data;

        if (*pBlank == FB_BLANK_UNBLANK)
        {
            DBG("*** %s() TP Resume ***\n", __func__);

            if (g_IsUpdateFirmware != 0) // Check whether update frimware is finished
            {
                DBG("Not allow to power on/off touch ic while update firmware.\n");
                return 0;
            }

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
            if (g_EnableTpProximity == 1)
            {
                DBG("g_EnableTpProximity = %d\n", g_EnableTpProximity);
                return 0;
            }
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION
            
#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#ifdef CONFIG_ENABLE_HOTKNOT
            if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT
            {
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
                if (g_GestureDebugMode == 1)
                {
                    DrvIcFwLyrCloseGestureDebugMode();
                }
#endif //CONFIG_ENABLE_GESTURE_DEBUG_MODE

                if (g_GestureWakeupFlag == 1)
                {
                    DrvIcFwLyrCloseGestureWakeup();
                }
                else
                {
                    DrvPlatformLyrEnableFingerTouchReport(); 
                }
            }
#ifdef CONFIG_ENABLE_HOTKNOT
            else    // Enable touch in hotknot transfer mode
            {
                DrvPlatformLyrEnableFingerTouchReport();     
            }
#endif //CONFIG_ENABLE_HOTKNOT
#endif //CONFIG_ENABLE_GESTURE_WAKEUP
    
#ifdef CONFIG_ENABLE_HOTKNOT
            if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT        
            {
                DrvPlatformLyrTouchDevicePowerOn(); 
            }   
    
#ifdef CONFIG_ENABLE_CHARGER_DETECTION 
            {
                u8 szChargerStatus[20] = {0};
     
                DrvCommonReadFile("/sys/class/power_supply/battery/status", szChargerStatus, 20);
            
                DBG("*** Battery Status : %s ***\n", szChargerStatus);
            
                g_ForceUpdate = 1; // Set flag to force update charger status
                
                if (strstr(szChargerStatus, "Charging") != NULL || strstr(szChargerStatus, "Full") != NULL || strstr(szChargerStatus, "Fully charged") != NULL)
                {
                    DrvFwCtrlChargerDetection(1); // charger plug-in
                }
                else // Not charging
                {
                    DrvFwCtrlChargerDetection(0); // charger plug-out
                }

                g_ForceUpdate = 0; // Clear flag after force update charger status
            }           
#endif //CONFIG_ENABLE_CHARGER_DETECTION

#ifdef CONFIG_ENABLE_GLOVE_MODE
            if (g_IsEnableGloveMode == 1)
            {
                DrvIcFwCtrlOpenGloveMode();
            }
#endif //CONFIG_ENABLE_GLOVE_MODE

            if (IS_FIRMWARE_DATA_LOG_ENABLED)    
            {
                DrvIcFwLyrRestoreFirmwareModeToLogDataMode(); // Mark this function call for avoiding device driver may spend longer time to resume from suspend state.
            } //IS_FIRMWARE_DATA_LOG_ENABLED

#ifndef CONFIG_ENABLE_GESTURE_WAKEUP
            DrvPlatformLyrEnableFingerTouchReport(); 
#endif //CONFIG_ENABLE_GESTURE_WAKEUP
        }
        else if (*pBlank == FB_BLANK_POWERDOWN)
        {
            DBG("*** %s() TP Suspend ***\n", __func__);
            
            if (g_IsUpdateFirmware != 0) // Check whether update frimware is finished
            {
                DBG("Not allow to power on/off touch ic while update firmware.\n");
                return 0;
            }

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
            if (g_EnableTpProximity == 1)
            {
                DBG("g_EnableTpProximity = %d\n", g_EnableTpProximity);
                return 0;
            }
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#ifdef CONFIG_ENABLE_HOTKNOT
            if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT
            {
                if (g_GestureWakeupMode[0] != 0x00000000 || g_GestureWakeupMode[1] != 0x00000000)
                {
                    DrvIcFwLyrOpenGestureWakeup(&g_GestureWakeupMode[0]);
                    return 0;
                }
            }
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

#ifdef CONFIG_ENABLE_HOTKNOT
            if (g_HotKnotState == HOTKNOT_BEFORE_TRANS_STATE || g_HotKnotState == HOTKNOT_TRANS_STATE || g_HotKnotState == HOTKNOT_AFTER_TRANS_STATE)
            {
                IicWriteData(SLAVE_I2C_ID_DWI2C, &_gAMStartCmd[0], 4); 
            }
#endif //CONFIG_ENABLE_HOTKNOT 


            DrvPlatformLyrFingerTouchReleased(0, 0, 0); // Send touch end for clearing point touch
            input_sync(g_InputDevice);

            DrvPlatformLyrDisableFingerTouchReport();

#ifdef CONFIG_ENABLE_HOTKNOT
            if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT        
            {
                DrvPlatformLyrTouchDevicePowerOff(); 
            }    
        }
    }

    return 0;
}

#else
struct i2c_client	*msg22xx_g_client;

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
static struct msg2238_ts_data msg2238_data;
struct msg22xx_virtualpsensor msg22xx_vps; 
struct sensors_classdev msg22xx_virtual_sensors_proximity_cdev = {
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
 };
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION
static int MsDrvInterfaceTouchDeviceSuspend(struct device *dev)
{
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
	 int nRetVal = 0;
     DBG("*** %s() ***\n", __func__);
		//TP selef open and close
		if(g_EnableTpProximity)
		{
			nRetVal = enable_irq_wake(msg2238_data.msg2138_irq);  
			if (nRetVal) {  
	        	printk("[msg2238] enable_irq_wake failed.\n");   
	   	 	}  
		
			if (device_may_wakeup(&msg22xx_g_client->dev))
			{
				enable_irq_wake(msg2238_data.msg2138_irq);
			}
			//msg22xx_tpd_proximity_suspend= 1;
			//msg2238_data->in_suspend = true;
			printk("== virtual proximity %s() ==\n",__func__);
			return 0;
		}
#endif//CONFIG_ENABLE_PROXIMITY_DETECTION
#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
    if (g_IsUpdateFirmware != 0) // Check whether update frimware is finished
    {
        DBG("Not allow to power on/off touch ic while update firmware.\n");
        return 0;
    }
#endif

	/* modify by furong, 2014/12/10, PR-865304, add for sometimes TP can't work after suspend/resume. */
	#if 0
	if (atomic_read(&tp_suspended)) {
		DBG("****Already in suspend state*****\n");
		return;
	}
	atomic_set(&tp_suspended, 1);
	#endif

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
    if (g_EnableTpProximity == 1)
    {
        DBG("g_EnableTpProximity = %d\n", g_EnableTpProximity);
        return 0;
    }
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#ifdef CONFIG_ENABLE_HOTKNOT
    if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT
    {
        if (g_GestureWakeupMode[0] != 0x00000000 || g_GestureWakeupMode[1] != 0x00000000)
        {
            DrvIcFwLyrOpenGestureWakeup(&g_GestureWakeupMode[0]);
            return;
        }
    }
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

#ifdef CONFIG_ENABLE_HOTKNOT
    if (g_HotKnotState == HOTKNOT_BEFORE_TRANS_STATE || g_HotKnotState == HOTKNOT_TRANS_STATE || g_HotKnotState == HOTKNOT_AFTER_TRANS_STATE)
    {
        IicWriteData(SLAVE_I2C_ID_DWI2C, &_gAMStartCmd[0], 4); 
    }
#endif //CONFIG_ENABLE_HOTKNOT  

	DrvPlatformLyrDisableFingerTouchReport();
    DrvPlatformLyrFingerTouchReleased(0, 0, 0); // Send touch end for clearing point touch
    input_sync(g_InputDevice);

    //DrvPlatformLyrDisableFingerTouchReport();

#ifdef CONFIG_ENABLE_HOTKNOT
    if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT        
    {
        DrvPlatformLyrTouchDevicePowerOff(); 
    DrvPlatformLyrTouchDeviceVoltageControl(false);//zxzadd

    }    
	return 0;

}

static int MsDrvInterfaceTouchDeviceResume(struct device *dev)
{
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
		int nRetVal = 0;
		DBG("*** %s() ***\n", __func__);

		//TP selef open and close
		if(g_EnableTpProximity)
		{
			nRetVal = disable_irq_wake(msg2238_data.msg2138_irq);	
			if (nRetVal) {	
				printk("[msg2238] disable_irq_wake failed.\n");   
			} 
			
			if(device_may_wakeup(&msg22xx_g_client->dev))
			{
	
				disable_irq_wake(msg2238_data.msg2138_irq);
			}	
			printk("== virtual proximity %s() ==\n",__func__);
			//msg2238_data->in_suspend = false;
			return 0;
		}
#endif

/* modify by furong, 2014/12/10, PR-865304, add for sometimes TP can't work after suspend/resume.
  because sometimes suspend twice but resume once, this leads to irq disabled twice but enabled only one time,
  irq disabled times not equal to enabled times will cause TP irq not correctly enable and TP can't work. */
	#if 0
	if (!atomic_read(&tp_suspended)) {
		DBG("****Already in awake state*****\n");
		return;
	}
	atomic_set(&tp_suspended, 0);
	#endif


#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
    if (g_EnableTpProximity == 1)
    {
        DBG("g_EnableTpProximity = %d\n", g_EnableTpProximity);
        return 0;
    }
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#ifdef CONFIG_ENABLE_HOTKNOT
    if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT
    {
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
        if (g_GestureDebugMode == 1)
        {
            DrvIcFwLyrCloseGestureDebugMode();
        }
#endif //CONFIG_ENABLE_GESTURE_DEBUG_MODE

	if (g_GestureWakeupFlag != 0x0000)
        {
            DrvIcFwLyrCloseGestureWakeup();
        }
        else
        {
            DrvPlatformLyrEnableFingerTouchReport(); 
        }
    }
#ifdef CONFIG_ENABLE_HOTKNOT
    else    // Enable touch in hotknot transfer mode
    {
        DrvPlatformLyrEnableFingerTouchReport();     
    }
#endif //CONFIG_ENABLE_HOTKNOT
#endif //CONFIG_ENABLE_GESTURE_WAKEUP
    
#ifdef CONFIG_ENABLE_HOTKNOT
    if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT        
    {
    DrvPlatformLyrTouchDeviceVoltageControl(true);//zxzadd
        DrvPlatformLyrTouchDevicePowerOn(); 
	mdelay(50);
    }   
    
#ifdef CONFIG_ENABLE_CHARGER_DETECTION 
    {
        u8 szChargerStatus[20] = {0};
 
        DrvCommonReadFile("/sys/class/power_supply/battery/status", szChargerStatus, 20);
        
        DBG("*** Battery Status : %s ***\n", szChargerStatus);
        
        g_ForceUpdate = 1; // Set flag to force update charger status

        if (strstr(szChargerStatus, "Charging") != NULL || strstr(szChargerStatus, "Full") != NULL || strstr(szChargerStatus, "Fully charged") != NULL)
        {
            DrvFwCtrlChargerDetection(1); // charger plug-in
        }
        else // Not charging
        {
            DrvFwCtrlChargerDetection(0); // charger plug-out
        }

        g_ForceUpdate = 0; // Clear flag after force update charger status
    }           
#endif //CONFIG_ENABLE_CHARGER_DETECTION

#ifdef CONFIG_ENABLE_GLOVE_MODE
    if (g_IsEnableGloveMode == 1)
    {
        DrvIcFwCtrlOpenGloveMode();
    }
#endif //CONFIG_ENABLE_GLOVE_MODE

    if (IS_FIRMWARE_DATA_LOG_ENABLED)    
    {
        DrvIcFwLyrRestoreFirmwareModeToLogDataMode(); // Mark this function call for avoiding device driver may spend longer time to resume from suspend state.
    } //IS_FIRMWARE_DATA_LOG_ENABLED

#ifndef CONFIG_ENABLE_GESTURE_WAKEUP
    DrvPlatformLyrEnableFingerTouchReport(); 
#endif //CONFIG_ENABLE_GESTURE_WAKEUP
    DrvPlatformLyrEnableFingerTouchReport(); 

#if 0
	//Add by Luoxingxing to release all points when resume.START
	for (i = 0; i < MAX_TOUCH_NUM; i ++)
	{
		 input_report_abs(g_InputDevice, ABS_MT_TRACKING_ID, -1);
   		 input_mt_sync(g_InputDevice);
	}
	input_sync(g_InputDevice);
#endif	
	return 0;
	//END
	
}

#endif //CONFIG_ENABLE_NOTIFIER_FB

#if 1
//add by mike.li for suspend & resume.[2015.07.07]
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			MsDrvInterfaceTouchDeviceResume(&msg22xx_g_client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			MsDrvInterfaceTouchDeviceSuspend(&msg22xx_g_client->dev);
	}
	return 0;
}
#endif
#endif
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION

void Set_psensor_register(int status)
{
	unsigned char tx_data[4];
	
	tx_data[0] = 0x52;
	tx_data[1] = 0x00;
	tx_data[2] = 0x4a;

	//0xa0:enable 0xa1: disable
	if(1==status)
		tx_data[3] = 0xa0;
	else
		tx_data[3] = 0xa1;
	
	mutex_lock(&g_Mutex);
	IicWriteData(SLAVE_I2C_ID_DWI2C, tx_data, 4);
	mutex_unlock(&g_Mutex);
//	printk("** %s ** , ebable = %d, tx_data[3] =%x\n" ,__func__ ,status,tx_data[3]);	
	//msleep(150);
}
#endif
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
int mag22x_vps_set_enable(struct sensors_classdev *sensors_cdev,unsigned int enable)
 {
	 u8 set_state; 
	 //if want to enable P-sensor should resume TP first
	 if(enable)
		 MsDrvInterfaceTouchDeviceResume(&msg22xx_g_client->dev);//whitout param
	 set_state=enable;
	 //for(i=0;i<3;i++)
	 {
		Set_psensor_register(enable);
		 msg22xx_vps.vps_enabled = set_state;
		 g_EnableTpProximity = msg22xx_vps.vps_enabled;
		 PRINTF_ERR("** ===== %s , tpd_proximity_msg2238_enable = %d ==== **\n",
				 __func__, g_EnableTpProximity);
	 }
	 if(1==enable)
	 {
		 if(0==msg22xx_vps.active_ps_first)
		 {
			 input_report_abs(msg22xx_vps.virtualdevice, ABS_DISTANCE, 1);
			 input_sync(msg22xx_vps.virtualdevice);
			 msg22xx_vps.active_ps_first = 1;
		 }
	 }
	 return 0;
 }
 
  int msg22xx_virtual_psensor_input_register(struct i2c_client *pClient)
 {
	 s32 nRetVal = 0;
	 PRINTF_ERR("*** %s() *** \n", __func__);
//msg22xx_vps = kzalloc(sizeof(struct msg22xx_virtualpsensor), GFP_KERNEL);
 //vps->client = pClient;
 //i2c_set_clientdata(pClient, vps);
 //add a input dev allocation start by ke.li
	 msg22xx_vps.virtualdevice= input_allocate_device();
	 if (msg22xx_vps.virtualdevice == NULL)
	 {
		 PRINTF_ERR("*** P-sensot input device allocation failed ***\n");
		 return -ENOMEM;
	 }
 
	 msg22xx_vps.virtualdevice->name = "proximity";
	 msg22xx_vps.virtualdevice->id.bustype = BUS_I2C;//i2c?
 
	 /* set the supported event type for input device */
	 set_bit(EV_ABS, msg22xx_vps.virtualdevice->evbit);
	 set_bit(ABS_DISTANCE, msg22xx_vps.virtualdevice->absbit);
	 input_set_abs_params(msg22xx_vps.virtualdevice, ABS_DISTANCE, 0, 1, 0, 0);
 
	 nRetVal = input_register_device(msg22xx_vps.virtualdevice);
	 if (nRetVal < 0)
	 {
		 PRINTF_ERR("*** Unable to register virtual P-sensor input device ***\n");
		 return nRetVal;
	 }
 
	 msg22xx_vps.vps_cdev = msg22xx_virtual_sensors_proximity_cdev; 
	 msg22xx_vps.vps_cdev.sensors_enable = mag22x_vps_set_enable;
	 msg22xx_vps.vps_cdev.sensors_poll_delay = NULL;
  //add a input dev allocation end by ke.li and register a sensors driver
	 nRetVal = sensors_classdev_register(&pClient->dev, &msg22xx_vps.vps_cdev);
	 if (nRetVal) {
		 PRINTF_ERR("%s: Unable to register to sensors class: %d\n",__func__, nRetVal);
		 return nRetVal;
	 }
	 return 0;
 }
#endif//CONFIG_ENABLE_PROXIMITY_DETECTION

extern int msg2638_pinctrl_init(struct i2c_client *pClient);
/* probe function is used for matching and initializing input device */
s32 /*__devinit*/ MsDrvInterfaceTouchDeviceProbe(struct i2c_client *pClient, const struct i2c_device_id *pDeviceId)
{
    s32 nRetVal = 0;

    DBG("*** %s() ***\n", __func__);
    DrvPlatformLyrTouchDeviceVoltageInit(pClient,true);//zxzadd
    DrvPlatformLyrTouchDeviceVoltageControl(true);//zxzadd

    nRetVal = msg2638_pinctrl_init(pClient);
	if (nRetVal < 0) {
		DBG("MIKE: msg2638_pinctrl_init err : %d\n", nRetVal);
		return nRetVal;
	}

    nRetVal = DrvPlatformLyrTouchDeviceRequestGPIO();
	if (nRetVal < 0) {
		DBG("MIKE: DrvPlatformLyrTouchDeviceRequestGPIO err : %d\n", nRetVal);
		return nRetVal;
	}

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
    DrvPlatformLyrTouchDeviceRegulatorPowerOn();
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

    DrvPlatformLyrTouchDevicePowerOn();
//add by ke.li add register Start for tp-psensor enable
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
	msg22xx_g_client = pClient;
	nRetVal= msg22xx_virtual_psensor_input_register(pClient);
	if(nRetVal< 0)
	{
		   printk("***msg22xx sensors input register  %d: failed\n", nRetVal);
		   return nRetVal;
	}
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION
//add by ke.li add register End for tp-psensor enable
    nRetVal = DrvPlatformLyrInputDeviceInitialize(pClient);
	if (nRetVal < 0) {
		DBG("MIKE: DrvPlatformLyrInputDeviceInitialize err : %d\n", nRetVal);
		return nRetVal;
	}
#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
    DmaAlloc();
#endif //CONFIG_ENABLE_DMA_IIC
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM

    nRetVal = DrvMainTouchDeviceInitialize();
    if (nRetVal == -ENODEV)
    {
        DrvPlatformLyrTouchDeviceRemove(pClient);
        return nRetVal;
    }

    DrvPlatformLyrTouchDeviceRegisterFingerTouchInterruptHandler();

    //DrvPlatformLyrTouchDeviceRegisterEarlySuspend();

//add by mike.li for suspend & resume.[2015.07.07]
#if defined(CONFIG_FB)
	TPfb_notifier.notifier_call = fb_notifier_callback;

	nRetVal = fb_register_client(&TPfb_notifier);

	if (nRetVal)
		dev_err(&pClient->dev, "Unable to register fb_notifier: %d\n",
			nRetVal);
#endif
	//add by mike.li for gesture.
	device_init_wakeup(&pClient->dev, 1);

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
    DrvIcFwLyrCheckFirmwareUpdateBySwId();
#endif //CONFIG_UPDATE_FIRMWARE_BY_SW_ID

    DBG("*** MStar touch driver registered ***\n");

	tp_dbg = 0;
    return nRetVal;
}

/* remove function is triggered when the input device is removed from input sub-system */
s32 /*__devexit*/ MsDrvInterfaceTouchDeviceRemove(struct i2c_client *pClient)
{
    DBG("*** %s() ***\n", __func__);

    return DrvPlatformLyrTouchDeviceRemove(pClient);
}

void MsDrvInterfaceTouchDeviceSetIicDataRate(struct i2c_client *pClient, u32 nIicDataRate)
{
    DBG("*** %s() ***\n", __func__);

    DrvPlatformLyrSetIicDataRate(pClient, nIicDataRate);
}    
