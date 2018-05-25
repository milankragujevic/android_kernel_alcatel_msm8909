
/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
*
* File Name	: lis2de.h
* Authors	: AMS - Motion Mems Division - Application Team
*		: Matteo Dameno (matteo.dameno@st.com)
*		: Denis Ciocca (denis.ciocca@st.com)
*		: Both authors are willing to be considered the contact
*		: and update points for the driver.
* Version	: V.1.0.1
* Date		: 2012/Oct/12
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
*******************************************************************************/
/*******************************************************************************
Version History.

 Revision 1.0.1: 2012/Oct/12
*******************************************************************************/

#ifndef	__LIS2DE_H__
#define	__LIS2DE_H__


#define	LIS2DE_ACC_DEV_NAME		"lis2de12_acc"

#define	LIS2DE_ACC_MIN_POLL_PERIOD_MS	1
//#define LIS2DE_ACC_ENABLE_IRQ1	 1
//#define LIS2DE_ACC_ENABLE_IRQ2	 1
//#define LIS2DE_ACC_ENABLE_FILTER	1

#define LIS2DE_ACC_SENSITIVITY_2G		156	/** mg/digit*10 */
#define LIS2DE_ACC_SENSITIVITY_4G		312	/** mg/digit*10 */
#define LIS2DE_ACC_SENSITIVITY_8G		625	/** mg/digit*10 */
#define LIS2DE_ACC_SENSITIVITY_16G		1875	/** mg/digit*10 */

#define FILTER_ELEMENT_DTH 20	// Add by shengwang.luo for data filter on 2015/03/23.

#ifdef __KERNEL__

#define LIS2DE_SAD0L				(0x00)
#define LIS2DE_SAD0H				(0x01)
#define LIS2DE_ACC_I2C_SADROOT			(0x0C)

/* I2C address if acc SA0 pin to GND */
#define LIS2DE_ACC_I2C_SAD_L		((LIS2DE_ACC_I2C_SADROOT<<1)| \
						LIS2DE_SAD0L)

/* I2C address if acc SA0 pin to Vdd */
#define LIS2DE_ACC_I2C_SAD_H		((LIS2DE_ACC_I2C_SADROOT<<1)| \
						LIS2DE_SAD0H)

/* to set gpios numb connected to interrupt pins, 
* the unused ones have to be set to -EINVAL
*/
#ifdef LIS2DE_ACC_ENABLE_IRQ1
#define LIS2DE_ACC_DEFAULT_INT1_GPIO		(96+911)	//add(modify) by shengwang.luo for enable irq1, 20141226
#else
#define LIS2DE_ACC_DEFAULT_INT1_GPIO		(-EINVAL)
#endif
#ifdef LIS2DE_ACC_ENABLE_IRQ2
#define LIS2DE_ACC_DEFAULT_INT2_GPIO		(97+911)	//add(modify) by shengwang.luo for enable irq2, 20141226
#else
#define LIS2DE_ACC_DEFAULT_INT2_GPIO		(-EINVAL)
#endif

/* Accelerometer Sensor Full Scale */
#define	LIS2DE_ACC_FS_MASK		(0x30)
#define LIS2DE_ACC_G_2G			(0x00)
#define LIS2DE_ACC_G_4G			(0x10)
#define LIS2DE_ACC_G_8G			(0x20)
#define LIS2DE_ACC_G_16G		(0x30)

struct lis2de_acc_platform_data {
	unsigned int poll_interval;
	unsigned int min_interval;

	u8 fs_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	/* set gpio_int[1,2] either to the choosen gpio pin number or to -EINVAL
	 * if leaved unconnected
	 */
	int gpio_int1;
	int gpio_int2;
};

#endif	/* __KERNEL__ */

#endif	/* __LIS2DE_H__ */



