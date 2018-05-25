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
 * @file    mstar_drv_platform_interface.h
 *
 * @brief   This file defines the interface of touch screen
 *
 *
 */

#ifndef __MSTAR_DRV_PLATFORM_INTERFACE_H__
#define __MSTAR_DRV_PLATFORM_INTERFACE_H__

/*--------------------------------------------------------------------------*/
/* INCLUDE FILE                                                             */
/*--------------------------------------------------------------------------*/

#include "mstar_drv_common.h"
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
#include <linux/sensors.h>
#endif//CONFIG_ENABLE_PROXIMITY_DETECTION

/*--------------------------------------------------------------------------*/
/* GLOBAL FUNCTION DECLARATION                                              */
/*--------------------------------------------------------------------------*/
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
#define VPS_NAME "virtual-proximity"
struct msg2238_ts_data {
        struct i2c_client *client;
        struct input_dev *input_dev;
        struct regulator *vdd;
        struct regulator *vcc_i2c;
        struct msg22xx_ts_platform_data *pdata ;

        struct dentry *dir;
        u16 addr;

	    bool in_suspend;  
		int msg2138_irq;
		struct mutex irq_lock;
//		struct wake_lock ps_wakelock;
};

struct msg22xx_virtualpsensor {
	 char const *name;
     struct i2c_client *client;
	 struct input_dev *virtualdevice;
	 bool vps_enabled;
	 struct sensors_classdev vps_cdev;
	 bool virtual_proximity_data;
	 int active_ps_first;
 };
#endif
extern s32 /*__devinit*/ MsDrvInterfaceTouchDeviceProbe(struct i2c_client *pClient, const struct i2c_device_id *pDeviceId);
extern s32 /*__devexit*/ MsDrvInterfaceTouchDeviceRemove(struct i2c_client *pClient);
#ifdef CONFIG_ENABLE_NOTIFIER_FB
extern int MsDrvInterfaceTouchDeviceFbNotifierCallback(struct notifier_block *pSelf, unsigned long nEvent, void *pData);
#else
extern void MsDrvInterfaceTouchDeviceResume(struct early_suspend *pSuspend);        
extern void MsDrvInterfaceTouchDeviceSuspend(struct early_suspend *pSuspend);
#endif //CONFIG_ENABLE_NOTIFIER_FB
extern void MsDrvInterfaceTouchDeviceSetIicDataRate(struct i2c_client *pClient, u32 nIicDataRate);
        
#endif  /* __MSTAR_DRV_PLATFORM_INTERFACE_H__ */
