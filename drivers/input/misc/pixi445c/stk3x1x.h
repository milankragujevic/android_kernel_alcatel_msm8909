/*
 *
 * $Id: stk3x1x.h
 *
 * Copyright (C) 2012~2013 Lex Hsieh     <lex_hsieh@sensortek.com.tw> 
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 */
#ifndef __STK3X1X_H__
#define __STK3X1X_H__

/* platform data */
struct stk3x1x_platform_data
{
	uint8_t state_reg;
	uint8_t psctrl_reg;
	uint8_t alsctrl_reg;
	uint8_t ledctrl_reg;
	uint8_t	wait_reg;	
	uint16_t ps_thd_h;
	uint16_t ps_thd_l;
	int int_pin;
	uint32_t transmittance;
	uint32_t int_flags;
	bool use_fir;
	uint16_t mmi_thd_h_limit;
	uint16_t mmi_thd_l_limit;
	uint16_t low_ps_ct_h_offset;
	uint16_t low_ps_ct_l_offset;
	uint16_t hi_ps_ct_h_offset;
	uint16_t hi_ps_ct_l_offset;
};
	

#endif // __STK3X1X_H__
