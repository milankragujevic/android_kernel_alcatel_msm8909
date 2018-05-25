/*
* Created on Nov 27th, 2013, by rongxiao.deng@tcl.com 
* Last modified: Nov 27th, 2013
*
* This software program is licensed subject to the GNU General Public License
* All Rights Reserved
*/

#ifndef __EMODE_GPIO_H
#define __EMODE_GPIO_H

typedef enum {
        GET_MODE_STA = 0,
        SET_MODE_0,  // 00
        SET_MODE_1,  // 01
        SET_MODE_2,  // 10
        SET_MODE_3,  // 11

        GET_DIR_STA,
        SET_DIR_IN,  // 0
        SET_DIR_OUT, // 1

        GET_PULLEN_STA,
        SET_PULLEN_DISABLE,  // 0
        SET_PULLEN_ENABLE,   // 1
 
        GET_PULL_STA,
        SET_PULL_DOWN,  // 0
        SET_PULL_UP,    // 1
 
        GET_INV_STA,
        SET_INV_ENABLE,  // 1
        SET_INV_DISABLE, // 0
 
        GET_DATA_IN,
        GET_DATA_OUT,
        SET_DATA_LOW,  // 0
        SET_DATA_HIGH, // 1
}GPIO_OP;


enum GPIO_OUT
{
    GPIO_OUT_ZERO = 0,
    GPIO_OUT_ONE  = 1,
};


#endif



