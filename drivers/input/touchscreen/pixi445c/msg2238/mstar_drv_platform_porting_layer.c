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
 * @file    mstar_drv_platform_porting_layer.c
 *
 * @brief   This file defines the interface of touch screen
 *
 *
 */
 
/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include "mstar_drv_platform_porting_layer.h"
#include "mstar_drv_ic_fw_porting_layer.h"
#include "mstar_drv_platform_interface.h"
#include "mstar_drv_utility_adaption.h"
#include "mstar_drv_main.h"

#ifdef CONFIG_ENABLE_HOTKNOT
#include "mstar_drv_hotknot_queue.h"
#endif //CONFIG_ENABLE_HOTKNOT

#ifdef CONFIG_ENABLE_JNI_INTERFACE
#include "mstar_drv_jni_interface.h"
#endif //CONFIG_ENABLE_JNI_INTERFACE

#include <linux/regulator/consumer.h>
/*=============================================================*/
// EXTREN VARIABLE DECLARATION
/*=============================================================*/

extern struct kset *g_TouchKSet;
extern struct kobject *g_TouchKObj;

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
extern struct kset *g_GestureKSet;
extern struct kobject *g_GestureKObj;
#endif //CONFIG_ENABLE_GESTURE_DEBUG_MODE
#endif //CONFIG_ENABLE_GESTURE_DEBUG_MODE

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
extern u8 g_FaceClosingTp;
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
extern struct tpd_device *tpd;
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
extern struct regulator *g_ReguVdd;
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON
#endif

#ifdef CONFIG_ENABLE_HOTKNOT
extern struct miscdevice hotknot_miscdevice;
extern u8 g_HotKnotState;
#endif //CONFIG_ENABLE_HOTKNOT

extern u8 IS_FIRMWARE_DATA_LOG_ENABLED;
extern struct msg22xx_virtualpsensor msg22xx_vps; 
extern void Drv_jrd_sysfs_remove(void);

/*=============================================================*/
// LOCAL VARIABLE DEFINITION
/*=============================================================*/

struct mutex g_Mutex;
spinlock_t _gIrqLock;

static struct work_struct _gFingerTouchWork;
static int _gInterruptFlag = 0;

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
#ifdef CONFIG_ENABLE_NOTIFIER_FB
//static struct notifier_block _gFbNotifier;
#else
static struct early_suspend _gEarlySuspend;
#endif //CONFIG_ENABLE_NOTIFIER_FB
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM || CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifndef CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM
static DECLARE_WAIT_QUEUE_HEAD(_gWaiter);
static struct task_struct *_gThread = NULL;
static int _gTpdFlag = 0;
#endif //CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM

/*=============================================================*/
// GLOBAL VARIABLE DEFINITION
/*=============================================================*/

#ifdef CONFIG_TP_HAVE_KEY
const int g_TpVirtualKey[] = {TOUCH_KEY_MENU, TOUCH_KEY_HOME, TOUCH_KEY_BACK};

#ifdef CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE
#define BUTTON_W (100)
#define BUTTON_H (100)

const int g_TpVirtualKeyDimLocal[MAX_KEY_NUM][4] = {{BUTTON_W/2*1,TOUCH_SCREEN_Y_MAX+BUTTON_H/2,BUTTON_W,BUTTON_H},{BUTTON_W/2*3,TOUCH_SCREEN_Y_MAX+BUTTON_H/2,BUTTON_W,BUTTON_H},{BUTTON_W/2*5,TOUCH_SCREEN_Y_MAX+BUTTON_H/2,BUTTON_W,BUTTON_H},{BUTTON_W/2*7,TOUCH_SCREEN_Y_MAX+BUTTON_H/2,BUTTON_W,BUTTON_H}};
#endif //CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE
#endif //CONFIG_TP_HAVE_KEY

struct input_dev *g_InputDevice = NULL;
static int _gIrq = -1;

/*=============================================================*/
// LOCAL FUNCTION DEFINITION
/*=============================================================*/

/* read data through I2C then report data to input sub-system when interrupt occurred */
static void _DrvPlatformLyrFingerTouchDoWork(struct work_struct *pWork)
{
    unsigned long nIrqFlag;

    DBG("*** %s() ***\n", __func__);

    DrvIcFwLyrHandleFingerTouch(NULL, 0);

    DBG("*** %s() _gInterruptFlag = %d ***\n", __func__, _gInterruptFlag);  // add for debug

//    DBG("*** %s() spin_lock_irqsave() ***\n", __func__);  // add for debug
    spin_lock_irqsave(&_gIrqLock, nIrqFlag);

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)

    if (_gInterruptFlag == 0) 
    {
        enable_irq(_gIrq);
//        DBG("*** %s() enable_irq(_gIrq) ***\n", __func__);  // add for debug

        _gInterruptFlag = 1;
    } 
        
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)

    if (_gInterruptFlag == 0) 
    {
        mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
//        DBG("*** %s() mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM) ***\n", __func__);  // add for debug

        _gInterruptFlag = 1;
    }

#endif

    spin_unlock_irqrestore(&_gIrqLock, nIrqFlag);
//    DBG("*** %s() spin_unlock_irqrestore() ***\n", __func__);  // add for debug
}

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
/* The interrupt service routine will be triggered when interrupt occurred */
static irqreturn_t _DrvPlatformLyrFingerTouchInterruptHandler(s32 nIrq, void *pDeviceId)
{
    unsigned long nIrqFlag;

    DBG("*** %s() ***\n", __func__);

    DBG("*** %s() _gInterruptFlag = %d ***\n", __func__, _gInterruptFlag);  // add for debug

//    DBG("*** %s() spin_lock_irqsave() ***\n", __func__);  // add for debug
    spin_lock_irqsave(&_gIrqLock, nIrqFlag);

    if (_gInterruptFlag == 1) 
    {
        disable_irq_nosync(_gIrq);
//        DBG("*** %s() disable_irq_nosync(_gIrq) ***\n", __func__);  // add for debug

        _gInterruptFlag = 0;

        schedule_work(&_gFingerTouchWork);
    }

    spin_unlock_irqrestore(&_gIrqLock, nIrqFlag);
//    DBG("*** %s() spin_unlock_irqrestore() ***\n", __func__);  // add for debug
    
    return IRQ_HANDLED;
}

/*=============================================================*/
// GLOBAL FUNCTION DEFINITION
/*=============================================================*/
//zxzadd
int DrvPlatformLyrTouchDeviceVoltageInit(struct i2c_client *client,bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	TPvdd = regulator_get(&client->dev, "vdd");
	if (IS_ERR(TPvdd)) {
		rc = PTR_ERR(TPvdd);
		dev_err(&client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(TPvdd) > 0) {
		rc = regulator_set_voltage(TPvdd, MSG_VTG_MIN_UV,
					   MSG_VTG_MAX_UV);
		if (rc) {
			dev_err(&client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	TPvcc_i2c = regulator_get(&client->dev, "vcc_i2c");
	if (IS_ERR(TPvcc_i2c)) {
		rc = PTR_ERR(TPvcc_i2c);
		dev_err(&client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(TPvcc_i2c) > 0) {
		rc = regulator_set_voltage(TPvcc_i2c, MSG_I2C_VTG_MIN_UV,
					   MSG_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}
	return 0;

reg_vcc_i2c_put:
	regulator_put(TPvcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(TPvdd) > 0)
		regulator_set_voltage(TPvdd, 0, MSG_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(TPvdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(TPvdd) > 0)
		regulator_set_voltage(TPvdd, 0, MSG_VTG_MAX_UV);

	regulator_put(TPvdd);

	if (regulator_count_voltages(TPvcc_i2c) > 0)
		regulator_set_voltage(TPvcc_i2c, 0, MSG_I2C_VTG_MAX_UV);

	regulator_put(TPvcc_i2c);
	return 0;
}


int DrvPlatformLyrTouchDeviceVoltageControl(bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(TPvdd);
	if (rc) {
		pr_err("Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(TPvcc_i2c);
	if (rc) {
		pr_err("Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(TPvdd);
	}

	return rc;

power_off:
	rc = regulator_disable(TPvdd);
	if (rc) {
		pr_err("Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(TPvcc_i2c);
	if (rc) {
		pr_err("Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(TPvdd);
		if (rc) {
			pr_err("Regulator vdd enable failed rc=%d\n", rc);
		}
	}
	
	return rc;
}

#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
static void _DrvPlatformLyrFingerTouchInterruptHandler(void)
{
    unsigned long nIrqFlag;

    DBG("*** %s() ***\n", __func__);  

    DBG("*** %s() _gInterruptFlag = %d ***\n", __func__, _gInterruptFlag);  // add for debug

//    DBG("*** %s() spin_lock_irqsave() ***\n", __func__);  // add for debug
    spin_lock_irqsave(&_gIrqLock, nIrqFlag);

#ifdef CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM

    if (_gInterruptFlag == 1)
    {
        mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
//        DBG("*** %s() mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM) ***\n", __func__);  // add for debug

        _gInterruptFlag = 0;

        schedule_work(&_gFingerTouchWork);
    }

#else    

    if (_gInterruptFlag == 1) 
    {    
        mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
//        DBG("*** %s() mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM) ***\n", __func__);  // add for debug

        _gInterruptFlag = 0;

        _gTpdFlag = 1;
        wake_up_interruptible(&_gWaiter);
    }        
#endif //CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM

    spin_unlock_irqrestore(&_gIrqLock, nIrqFlag);
//    DBG("*** %s() spin_unlock_irqrestore() ***\n", __func__);  // add for debug
}

#ifndef CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM
static int _DrvPlatformLyrFingerTouchHandler(void *pUnUsed)
{
    unsigned long nIrqFlag;
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    sched_setscheduler(current, SCHED_RR, &param);

    DBG("*** %s() ***\n", __func__);  
	
    do
    {
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(_gWaiter, _gTpdFlag != 0);
        _gTpdFlag = 0;
        
        set_current_state(TASK_RUNNING);

        DrvIcFwLyrHandleFingerTouch(NULL, 0);

        DBG("*** %s() _gInterruptFlag = %d ***\n", __func__, _gInterruptFlag);  // add for debug

//        DBG("*** %s() spin_lock_irqsave() ***\n", __func__);  // add for debug
        spin_lock_irqsave(&_gIrqLock, nIrqFlag);

        if (_gInterruptFlag == 0)        
        {
            mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
//            DBG("*** %s() mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM) ***\n", __func__);  // add for debug

            _gInterruptFlag = 1;
        } 

        spin_unlock_irqrestore(&_gIrqLock, nIrqFlag);
//        DBG("*** %s() spin_unlock_irqrestore() ***\n", __func__);  // add for debug
		
    } while (!kthread_should_stop());
	
    return 0;
}
#endif //CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM
#endif

/*=============================================================*/
// GLOBAL FUNCTION DEFINITION
/*=============================================================*/

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
void DrvPlatformLyrTouchDeviceRegulatorPowerOn(void)
{
#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
    s32 nRetVal = 0;

    DBG("*** %s() ***\n", __func__);
    
    nRetVal = regulator_set_voltage(g_ReguVdd, 2800000, 2800000); // For specific SPRD BB chip(ex. SC7715) or QCOM BB chip(ex. MSM8610), need to enable this function call for correctly power on Touch IC.

    if (nRetVal)
    {
        DBG("Could not set to 2800mv.\n");
    }
    regulator_enable(g_ReguVdd);

    mdelay(20);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
    //hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_2800, "TP"); // For specific MTK BB chip(ex. MT6582), need to enable this function call for correctly power on Touch IC.
    hwPowerOn(PMIC_APP_CAP_TOUCH_VDD, VOL_2800, "TP"); // For specific MTK BB chip(ex. MT6735), need to enable this function call for correctly power on Touch IC.
#endif
}
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

void DrvPlatformLyrTouchDevicePowerOn(void)
{
    DBG("*** %s() ***\n", __func__);
    
    gpio_direction_output(global_reset_gpio, 1);
//    gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 1); 
//    mdelay(100);
    gpio_set_value(global_reset_gpio, 0);
    mdelay(50);
    gpio_set_value(global_reset_gpio, 1);
    mdelay(50);//(300);

}

void DrvPlatformLyrTouchDevicePowerOff(void)
{
    DBG("*** %s() ***\n", __func__);
    
    DrvIcFwLyrOptimizeCurrentConsumption();

//    gpio_direction_output(MS_TS_MSG_IC_GPIO_RST, 0);
    gpio_set_value(global_reset_gpio, 0);

}

void DrvPlatformLyrTouchDeviceResetHw(void)
{
    DBG("*** %s() ***\n", __func__);
        gpio_direction_output(global_reset_gpio, 1);
//    gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 1); 
    gpio_set_value(global_reset_gpio, 0);
    mdelay(50); //(100);
    gpio_set_value(global_reset_gpio, 1);
    mdelay(50); //(100);

}

void DrvPlatformLyrDisableFingerTouchReport(void)
{
    unsigned long nIrqFlag;

    DBG("*** %s() ***\n", __func__);

    DBG("*** %s() _gInterruptFlag = %d ***\n", __func__, _gInterruptFlag);  // add for debug

//    DBG("*** %s() spin_lock_irqsave() ***\n", __func__);  // add for debug
    spin_lock_irqsave(&_gIrqLock, nIrqFlag);

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)

#ifdef CONFIG_ENABLE_HOTKNOT
    if (g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT
    {
        if (_gInterruptFlag == 1)  
        {
            disable_irq(_gIrq);
//            DBG("*** %s() disable_irq(_gIrq) ***\n", __func__);  // add for debug

            _gInterruptFlag = 0;
        }
    }
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)

#ifdef CONFIG_ENABLE_HOTKNOT
    if (g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT           
    {
        if (_gInterruptFlag == 1) 
        {
            mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
//            DBG("*** %s() mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM) ***\n", __func__);  // add for debug

            _gInterruptFlag = 0;
        }
    }
#endif

    spin_unlock_irqrestore(&_gIrqLock, nIrqFlag);
//    DBG("*** %s() spin_unlock_irqrestore() ***\n", __func__);  // add for debug
}

void DrvPlatformLyrEnableFingerTouchReport(void)
{
    unsigned long nIrqFlag;

    DBG("*** %s() ***\n", __func__);

    DBG("*** %s() _gInterruptFlag = %d ***\n", __func__, _gInterruptFlag);  // add for debug

//    DBG("*** %s() spin_lock_irqsave() ***\n", __func__);  // add for debug
    spin_lock_irqsave(&_gIrqLock, nIrqFlag);

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)

    if (_gInterruptFlag == 0) 
    {
        enable_irq(_gIrq);
//        DBG("*** %s() enable_irq(_gIrq) ***\n", __func__);  // add for debug

        _gInterruptFlag = 1;        
    }

#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)

    if (_gInterruptFlag == 0) 
    {
        mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
//        DBG("*** %s() mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM) ***\n", __func__);  // add for debug

        _gInterruptFlag = 1;        
    }

#endif

    spin_unlock_irqrestore(&_gIrqLock, nIrqFlag);
//    DBG("*** %s() spin_unlock_irqrestore() ***\n", __func__);  // add for debug
}

void DrvPlatformLyrFingerTouchPressed(s32 nX, s32 nY, s32 nPressure, s32 nId)
{
    DBG("*** %s() ***\n", __func__);
    DBG("point touch pressed\n");

#ifdef CONFIG_ENABLE_TYPE_B_PROTOCOL // TYPE B PROTOCOL
//	input_report_key(g_InputDevice, BTN_TOUCH, 1);
    input_mt_slot(g_InputDevice, nId);
    input_mt_report_slot_state(g_InputDevice, MT_TOOL_FINGER, true);
//    input_report_abs(g_InputDevice, ABS_MT_TRACKING_ID, nId);
//    input_report_abs(g_InputDevice, ABS_MT_TOUCH_MAJOR, 1);
//    input_report_abs(g_InputDevice, ABS_MT_WIDTH_MAJOR, 1);
    input_report_abs(g_InputDevice, ABS_MT_POSITION_X, nX);
    input_report_abs(g_InputDevice, ABS_MT_POSITION_Y, nY);
    input_mt_report_pointer_emulation(g_InputDevice,false);

//    DBG("nId=%d, nX=%d, nY=%d\n", nId, nX, nY); // TODO : add for debug
#else // TYPE A PROTOCOL
    input_report_key(g_InputDevice, BTN_TOUCH, 1);
#if defined(CONFIG_ENABLE_TOUCH_DRIVER_FOR_MUTUAL_IC)
    input_report_abs(g_InputDevice, ABS_MT_TRACKING_ID, nId);
#endif //CONFIG_ENABLE_TOUCH_DRIVER_FOR_MUTUAL_IC
    input_report_abs(g_InputDevice, ABS_MT_TOUCH_MAJOR, 1);
    input_report_abs(g_InputDevice, ABS_MT_WIDTH_MAJOR, 1);
    input_report_abs(g_InputDevice, ABS_MT_POSITION_X, nX);
    input_report_abs(g_InputDevice, ABS_MT_POSITION_Y, nY);

    input_mt_sync(g_InputDevice);
#endif //CONFIG_ENABLE_TYPE_B_PROTOCOL


#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_TP_HAVE_KEY    
    if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
    {   
        tpd_button(nX, nY, 1);  
    }
#endif //CONFIG_TP_HAVE_KEY

    TPD_EM_PRINT(nX, nY, nX, nY, nId, 1);
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
}

void DrvPlatformLyrFingerTouchReleased(s32 nX, s32 nY, s32 nId)
{
    DBG("*** %s() ***\n", __func__);
    DBG("point touch released\n");

#ifdef CONFIG_ENABLE_TYPE_B_PROTOCOL // TYPE B PROTOCOL
//    input_report_key(g_InputDevice, BTN_TOUCH, 0); 
    input_mt_slot(g_InputDevice, nId);
//    input_report_abs(g_InputDevice, ABS_MT_TRACKING_ID, -1);
    input_mt_report_slot_state(g_InputDevice, MT_TOOL_FINGER, false);
    input_mt_report_pointer_emulation(g_InputDevice,false);

//    DBG("nId=%d\n", nId); // TODO : add for debug
#else // TYPE A PROTOCOL
    input_report_key(g_InputDevice, BTN_TOUCH, 0);
    input_mt_sync(g_InputDevice);
#endif //CONFIG_ENABLE_TYPE_B_PROTOCOL


#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_TP_HAVE_KEY 
    if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
    {   
       tpd_button(nX, nY, 0); 
//       tpd_button(0, 0, 0); 
    }            
#endif //CONFIG_TP_HAVE_KEY    

    TPD_EM_PRINT(nX, nY, nX, nY, 0, 0);
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
}

s32 DrvPlatformLyrInputDeviceInitialize(struct i2c_client *pClient)
{
    s32 nRetVal = 0;

    DBG("*** %s() ***\n", __func__);

    mutex_init(&g_Mutex);
    spin_lock_init(&_gIrqLock);

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
    /* allocate an input device */
    g_InputDevice = input_allocate_device();
    if (g_InputDevice == NULL)
    {
        DBG("*** input device allocation failed ***\n");
        return -ENOMEM;
    }

    g_InputDevice->name = pClient->name;
    g_InputDevice->phys = "I2C";
    g_InputDevice->dev.parent = &pClient->dev;
    g_InputDevice->id.bustype = BUS_I2C;
    
    /* set the supported event type for input device */
    set_bit(EV_ABS, g_InputDevice->evbit);
    set_bit(EV_SYN, g_InputDevice->evbit);
    set_bit(EV_KEY, g_InputDevice->evbit);
    set_bit(BTN_TOUCH, g_InputDevice->keybit);
    set_bit(INPUT_PROP_DIRECT, g_InputDevice->propbit);

#ifdef CONFIG_TP_HAVE_KEY
    // Method 1.
    { 
        u32 i;
        for (i = 0; i < MAX_KEY_NUM; i ++)
        {
            input_set_capability(g_InputDevice, EV_KEY, g_TpVirtualKey[i]);
        }
    }
#endif
/*  
#ifdef CONFIG_TP_HAVE_KEY
    // Method 2.
    set_bit(TOUCH_KEY_MENU, g_InputDevice->keybit); //Menu
    set_bit(TOUCH_KEY_HOME, g_InputDevice->keybit); //Home
    set_bit(TOUCH_KEY_BACK, g_InputDevice->keybit); //Back
    set_bit(TOUCH_KEY_SEARCH, g_InputDevice->keybit); //Search
#endif
*/

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    input_set_capability(g_InputDevice, EV_KEY, KEY_POWER);
    input_set_capability(g_InputDevice, EV_KEY, KEY_UP);
    input_set_capability(g_InputDevice, EV_KEY, KEY_DOWN);
    input_set_capability(g_InputDevice, EV_KEY, KEY_LEFT);
    input_set_capability(g_InputDevice, EV_KEY, KEY_RIGHT);
    input_set_capability(g_InputDevice, EV_KEY, KEY_W);
    input_set_capability(g_InputDevice, EV_KEY, KEY_Z);
    input_set_capability(g_InputDevice, EV_KEY, KEY_V);
    input_set_capability(g_InputDevice, EV_KEY, KEY_O);
    input_set_capability(g_InputDevice, EV_KEY, KEY_M);
    input_set_capability(g_InputDevice, EV_KEY, KEY_C);
    input_set_capability(g_InputDevice, EV_KEY, KEY_E);
    input_set_capability(g_InputDevice, EV_KEY, KEY_S);
#endif //CONFIG_ENABLE_GESTURE_WAKEUP


#if defined(CONFIG_ENABLE_TOUCH_DRIVER_FOR_MUTUAL_IC)
    input_set_abs_params(g_InputDevice, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
#endif //CONFIG_ENABLE_TOUCH_DRIVER_FOR_MUTUAL_IC
    input_set_abs_params(g_InputDevice, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(g_InputDevice, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
    input_set_abs_params(g_InputDevice, ABS_MT_POSITION_X, TOUCH_SCREEN_X_MIN, TOUCH_SCREEN_X_MAX, 0, 0);
    input_set_abs_params(g_InputDevice, ABS_MT_POSITION_Y, TOUCH_SCREEN_Y_MIN, TOUCH_SCREEN_Y_MAX, 0, 0);

#ifdef CONFIG_ENABLE_TYPE_B_PROTOCOL
//    set_bit(BTN_TOOL_FINGER, g_InputDevice->keybit);
//    input_set_abs_params(g_InputDevice, ABS_MT_SLOT, 0, MAX_TOUCH_NUM - 1, 0, 0);
    input_mt_init_slots(g_InputDevice, MAX_TOUCH_NUM,INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
#endif //CONFIG_ENABLE_TYPE_B_PROTOCOL

    /* register the input device to input sub-system */
    nRetVal = input_register_device(g_InputDevice);
    if (nRetVal < 0)
    {
        DBG("*** Unable to register touch input device ***\n");
    }
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
    g_InputDevice = tpd->dev;
/*
    g_InputDevice->phys = "I2C";
    g_InputDevice->dev.parent = &pClient->dev;
    g_InputDevice->id.bustype = BUS_I2C;
    
    // set the supported event type for input device 
    set_bit(EV_ABS, g_InputDevice->evbit);
    set_bit(EV_SYN, g_InputDevice->evbit);
    set_bit(EV_KEY, g_InputDevice->evbit);
    set_bit(BTN_TOUCH, g_InputDevice->keybit);
    set_bit(INPUT_PROP_DIRECT, g_InputDevice->propbit);
*/

#ifdef CONFIG_TP_HAVE_KEY
    {
        u32 i;
        for (i = 0; i < MAX_KEY_NUM; i ++)
        {
            input_set_capability(g_InputDevice, EV_KEY, g_TpVirtualKey[i]);
        }
    }
#endif

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    input_set_capability(g_InputDevice, EV_KEY, KEY_POWER);
    input_set_capability(g_InputDevice, EV_KEY, KEY_UP);
    input_set_capability(g_InputDevice, EV_KEY, KEY_DOWN);
    input_set_capability(g_InputDevice, EV_KEY, KEY_LEFT);
    input_set_capability(g_InputDevice, EV_KEY, KEY_RIGHT);
    input_set_capability(g_InputDevice, EV_KEY, KEY_W);
    input_set_capability(g_InputDevice, EV_KEY, KEY_Z);
    input_set_capability(g_InputDevice, EV_KEY, KEY_V);
    input_set_capability(g_InputDevice, EV_KEY, KEY_O);
    input_set_capability(g_InputDevice, EV_KEY, KEY_M);
    input_set_capability(g_InputDevice, EV_KEY, KEY_C);
    input_set_capability(g_InputDevice, EV_KEY, KEY_E);
    input_set_capability(g_InputDevice, EV_KEY, KEY_S);
#endif //CONFIG_ENABLE_GESTURE_WAKEUP


#if defined(CONFIG_ENABLE_TOUCH_DRIVER_FOR_MUTUAL_IC)
    input_set_abs_params(g_InputDevice, ABS_MT_TRACKING_ID, 0, (MAX_TOUCH_NUM-1), 0, 0);
#endif //CONFIG_ENABLE_TOUCH_DRIVER_FOR_MUTUAL_IC
/*
    input_set_abs_params(g_InputDevice, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(g_InputDevice, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
    input_set_abs_params(g_InputDevice, ABS_MT_POSITION_X, TOUCH_SCREEN_X_MIN, TOUCH_SCREEN_X_MAX, 0, 0);
    input_set_abs_params(g_InputDevice, ABS_MT_POSITION_Y, TOUCH_SCREEN_Y_MIN, TOUCH_SCREEN_Y_MAX, 0, 0);
*/

#ifdef CONFIG_ENABLE_TYPE_B_PROTOCOL
//    set_bit(BTN_TOOL_FINGER, g_InputDevice->keybit);
//    input_set_abs_params(g_InputDevice, ABS_MT_SLOT, 0, MAX_TOUCH_NUM - 1, 0, 0);
    input_mt_init_slots(g_InputDevice, MAX_TOUCH_NUM, INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
#endif //CONFIG_ENABLE_TYPE_B_PROTOCOL

#endif

    return nRetVal;    
}

s32 DrvPlatformLyrTouchDeviceRequestGPIO(void)
{
    s32 nRetVal = 0;

    DBG("*** %s() ***\n", __func__);
    
    nRetVal = gpio_request(global_reset_gpio, "C_TP_RST");     
    if (nRetVal < 0)
    {	
        DBG("*** Failed to request GPIO %d, error %d ***\n", MS_TS_MSG_IC_GPIO_RST, nRetVal);
		goto err_request_tprst;
    }
	gpio_direction_output(global_reset_gpio, 1);

    nRetVal = gpio_request(global_irq_gpio, "C_TP_INT");    
    if (nRetVal < 0)
    {
        DBG("*** Failed to request GPIO %d, error %d ***\n", MS_TS_MSG_IC_GPIO_INT, nRetVal);
		goto err_request_tpirq;
    }
	gpio_direction_input(global_irq_gpio);
    return nRetVal;    

err_request_tpirq:
	gpio_free(global_irq_gpio);
err_request_tprst:
	gpio_free(global_reset_gpio);
	return -1;
}

struct msg2638_sys_data_s{
    struct pinctrl *pinctrl;
    struct pinctrl_state *pinctrl_state_active;
    struct pinctrl_state *pinctrl_state_suspend;
};

struct msg2638_sys_data_s *msg2638_sys_data;
int msg2638_pinctrl_init(struct i2c_client *pClient)
{
    int ret;

	msg2638_sys_data = kzalloc(sizeof(struct msg2638_sys_data_s), GFP_KERNEL);
    if (msg2638_sys_data == NULL) {
        kfree(msg2638_sys_data);
        return -ENOMEM;
    }

    msg2638_sys_data->pinctrl = devm_pinctrl_get(&(pClient->dev));
    if (IS_ERR_OR_NULL(msg2638_sys_data->pinctrl)) {
        ret = PTR_ERR(msg2638_sys_data->pinctrl);
        goto err_pinctrl_get;
    }

    msg2638_sys_data->pinctrl_state_active = pinctrl_lookup_state(msg2638_sys_data->pinctrl, "pmx_ts_int_active");
    if (IS_ERR_OR_NULL(msg2638_sys_data->pinctrl_state_active)) {
        ret = PTR_ERR(msg2638_sys_data->pinctrl_state_active);
        goto err_pinctrl_lookup;
    }

    msg2638_sys_data->pinctrl_state_suspend = pinctrl_lookup_state(msg2638_sys_data->pinctrl, "pmx_ts_int_suspend");
    if (IS_ERR_OR_NULL(msg2638_sys_data->pinctrl_state_suspend)) {
        ret = PTR_ERR(msg2638_sys_data->pinctrl_state_suspend);
        goto err_pinctrl_lookup;
    }

    ret = pinctrl_select_state(msg2638_sys_data->pinctrl, msg2638_sys_data->pinctrl_state_active);
    if (ret) {
        return ret;
    }
    return 0;

err_pinctrl_lookup:
    devm_pinctrl_put(msg2638_sys_data->pinctrl);

err_pinctrl_get:
    msg2638_sys_data->pinctrl = NULL;
    return ret;
}



s32 DrvPlatformLyrTouchDeviceRegisterFingerTouchInterruptHandler(void)
{
    s32 nRetVal = 0;

    DBG("*** %s() ***\n", __func__);

    if (DrvIcFwLyrIsRegisterFingerTouchInterruptHandler())
    {    	
#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
        /* initialize the finger touch work queue */ 
        INIT_WORK(&_gFingerTouchWork, _DrvPlatformLyrFingerTouchDoWork);

        _gIrq = gpio_to_irq(global_irq_gpio);

        /* request an irq and register the isr */
        nRetVal = request_threaded_irq(_gIrq/*MS_TS_MSG_IC_GPIO_INT*/, NULL, _DrvPlatformLyrFingerTouchInterruptHandler,
                      IRQF_TRIGGER_FALLING | IRQF_ONESHOT/* | IRQF_NO_SUSPEND *//* IRQF_TRIGGER_FALLING */,
                      "msg2xxx", NULL); 

//        nRetVal = request_irq(_gIrq/*MS_TS_MSG_IC_GPIO_INT*/, _DrvPlatformLyrFingerTouchInterruptHandler,
//                      IRQF_TRIGGER_RISING /* | IRQF_NO_SUSPEND *//* IRQF_TRIGGER_FALLING */,
//                      "msg2xxx", NULL); 
        _gInterruptFlag = 1;
        
        if (nRetVal != 0)
        {
            DBG("*** Unable to claim irq %d; error %d ***\n", MS_TS_MSG_IC_GPIO_INT, nRetVal);
        }

        DBG("MIKE: ************* irq %d; error %d ****************\n", global_irq_gpio, nRetVal);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
        mt_set_gpio_mode(MS_TS_MSG_IC_GPIO_INT, GPIO_CTP_EINT_PIN_M_EINT);
        mt_set_gpio_dir(MS_TS_MSG_IC_GPIO_INT, GPIO_DIR_IN);
        mt_set_gpio_pull_enable(MS_TS_MSG_IC_GPIO_INT, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(MS_TS_MSG_IC_GPIO_INT, GPIO_PULL_UP);

        mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
        mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE/* EINTF_TRIGGER_RISING */, _DrvPlatformLyrFingerTouchInterruptHandler, 1);

        mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

        _gInterruptFlag = 1;

#ifdef CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM
        /* initialize the finger touch work queue */ 
        INIT_WORK(&_gFingerTouchWork, _DrvPlatformLyrFingerTouchDoWork);
#else
        _gThread = kthread_run(_DrvPlatformLyrFingerTouchHandler, 0, TPD_DEVICE);
        if (IS_ERR(_gThread))
        { 
            nRetVal = PTR_ERR(_gThread);
            DBG("Failed to create kernel thread: %d\n", nRetVal);
        }
#endif //CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM
#endif
    }
    
    return nRetVal;    
}	
void DrvPlatformLyrTouchDeviceRegisterEarlySuspend(void)
{
    DBG("*** %s() ***\n", __func__);

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
#ifdef CONFIG_ENABLE_NOTIFIER_FB
    //_gFbNotifier.notifier_call = MsDrvInterfaceTouchDeviceFbNotifierCallback;
    //fb_register_client(&_gFbNotifier);
#else
    _gEarlySuspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
    _gEarlySuspend.suspend = MsDrvInterfaceTouchDeviceSuspend;
    _gEarlySuspend.resume = MsDrvInterfaceTouchDeviceResume;
    register_early_suspend(&_gEarlySuspend);
#endif //CONFIG_ENABLE_NOTIFIER_FB   
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM || CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM
}
/* remove function is triggered when the input device is removed from input sub-system */
s32 DrvPlatformLyrTouchDeviceRemove(struct i2c_client *pClient)
{
    DBG("*** %s() ***\n", __func__);
#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
//    free_irq(MS_TS_MSG_IC_GPIO_INT, g_InputDevice);
    free_irq(_gIrq, g_InputDevice);
    gpio_free(global_irq_gpio);
    gpio_free(global_reset_gpio);
	
   DrvPlatformLyrTouchDeviceVoltageControl(false);//zxzadd
   DrvPlatformLyrTouchDeviceVoltageInit(pClient,false);//zxzadd
   Drv_jrd_sysfs_remove();
   
    	input_unregister_device(g_InputDevice);
	#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
	sensors_classdev_unregister(&msg22xx_vps.vps_cdev);
	input_unregister_device(msg22xx_vps.virtualdevice);
	#endif
#endif    
    if (IS_FIRMWARE_DATA_LOG_ENABLED)
    {    	
        if (g_TouchKSet)
        {
            kset_unregister(g_TouchKSet);
            g_TouchKSet = NULL;
        }
    
        if (g_TouchKObj)
        {
            kobject_put(g_TouchKObj);
            g_TouchKObj = NULL;
        }
    } //IS_FIRMWARE_DATA_LOG_ENABLED

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
    if (g_GestureKSet)
    {
        kset_unregister(g_GestureKSet);
        g_GestureKSet = NULL;
    }
    
    if (g_GestureKObj)
    {
        kobject_put(g_GestureKObj);
        g_GestureKObj = NULL;
    }
#endif //CONFIG_ENABLE_GESTURE_DEBUG_MODE
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

    DrvMainRemoveProcfsDirEntry();

#ifdef CONFIG_ENABLE_HOTKNOT
    DeleteQueue();
    DeleteHotKnotMem();
    DBG("Deregister hotknot misc device.\n");
    misc_deregister( &hotknot_miscdevice );   
#endif //CONFIG_ENABLE_HOTKNOT

#ifdef CONFIG_ENABLE_JNI_INTERFACE
    DeleteMsgToolMem();
#endif //CONFIG_ENABLE_JNI_INTERFACE

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
    DmaFree();
#endif //CONFIG_ENABLE_DMA_IIC
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM

    return 0;
}

void DrvPlatformLyrSetIicDataRate(struct i2c_client *pClient, u32 nIicDataRate)
{
    DBG("*** %s() nIicDataRate = %d ***\n", __func__, nIicDataRate);

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM)
    // TODO : Please FAE colleague to confirm with customer device driver engineer for how to set i2c data rate on SPRD platform
    sprd_i2c_ctl_chg_clk(pClient->adapter->nr, nIicDataRate); 
    mdelay(100);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
    // TODO : Please FAE colleague to confirm with customer device driver engineer for how to set i2c data rate on QCOM platform
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
    pClient->timing = nIicDataRate/1000;
#endif
}

//------------------------------------------------------------------------------//

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION

int DrvPlatformLyrGetTpPsData(void)
{
    DBG("*** %s() g_FaceClosingTp = %d ***\n", __func__, g_FaceClosingTp);
	
    return g_FaceClosingTp;
}

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
void DrvPlatformLyrTpPsEnable(int nEnable)
{
    DBG("*** %s() nEnable = %d ***\n", __func__, nEnable);

    if (nEnable)
    {
        DrvIcFwLyrEnableProximity();
    }
    else
    {
        DrvIcFwLyrDisableProximity();
    }
}
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
int DrvPlatformLyrTpPsOperate(void* pSelf, u32 nCommand, void* pBuffIn, int nSizeIn,
				   void* pBuffOut, int nSizeOut, int* pActualOut)
{
    int nErr = 0;
    int nValue;
    hwm_sensor_data *pSensorData;

    switch (nCommand)
    {
        case SENSOR_DELAY:
            if ((pBuffIn == NULL) || (nSizeIn < sizeof(int)))
            {
                nErr = -EINVAL;
            }
            // Do nothing
            break;

        case SENSOR_ENABLE:
            if ((pBuffIn == NULL) || (nSizeIn < sizeof(int)))
            {
                nErr = -EINVAL;
            }
            else
            {
                nValue = *(int *)pBuffIn;
                if (nValue)
                {
                    if (DrvIcFwLyrEnableProximity() < 0)
                    {
                        DBG("Enable ps fail: %d\n", nErr);
                        return -1;
                    }
                }
                else
                {
                    if (DrvIcFwLyrDisableProximity() < 0)
                    {
                        DBG("Disable ps fail: %d\n", nErr);
                        return -1;
                    }
                }
            }
            break;

        case SENSOR_GET_DATA:
            if ((pBuffOut == NULL) || (nSizeOut < sizeof(hwm_sensor_data)))
            {
                DBG("Get sensor data parameter error!\n");
                nErr = -EINVAL;
            }
            else
            {
                pSensorData = (hwm_sensor_data *)pBuffOut;

                pSensorData->values[0] = DrvPlatformLyrGetTpPsData();
                pSensorData->value_divide = 1;
                pSensorData->status = SENSOR_STATUS_ACCURACY_MEDIUM;
            }
            break;

       default:
           DBG("Un-recognized parameter %d!\n", nCommand);
           nErr = -1;
           break;
    }

    return nErr;
}
#endif

#endif //CONFIG_ENABLE_PROXIMITY_DETECTION

//------------------------------------------------------------------------------//

