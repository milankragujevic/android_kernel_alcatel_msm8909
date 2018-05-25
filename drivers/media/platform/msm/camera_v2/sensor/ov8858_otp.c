#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"
#include "ov8858_otp.h"

//#define OV8858_DEBUG
#define OV8858_TYPICAL_RG_SUNRISE 327 // 0x147
#define OV8858_TYPICAL_BG_SUNRISE 288 // 0x120
#define OV8858_TYPICAL_RG_QTECH 308 // 0x134
#define OV8858_TYPICAL_BG_QTECH 302 // 0x12E
/*[BUGFIX]-Mod-BEGIN by TCTSZ.(gaoxiang.zou@tcl.com), otp 12/17/2015*/

#define OV8858_TYPICAL_RG_SUNNY 147 // 0x134
#define OV8858_TYPICAL_BG_SUNNY 120

/*[BUGFIX]-Mod-END by TCTSZ.(gaoxiang.zou@tcl.com), OTP*/

#define OV8858_REG_MODULE_FLAG 0x7010
#define OV8858_REG_MODULE_GROUP1_START 0x7011
#define OV8858_REG_MODULE_GROUP1_END 0x7017
#define OV8858_REG_MODULE_GROUP2_START 0x7018
#define OV8858_REG_MODULE_GROUP2_END 0x701E
#define OV8858_REG_AWB_FLAG 0x701F
#define OV8858_REG_AWB_GROUP1_START 0x7020
#define OV8858_REG_AWB_GROUP1_END 0x7022
#define OV8858_REG_AWB_GROUP2_START 0x7023
#define OV8858_REG_AWB_GROUP2_END 0x7025
#define OV8858_REG_LENC_FLAG 0x702D
#define OV8858_REG_LENC_GROUP1_START 0x702E
#define OV8858_REG_LENC_GROUP1_END 0x711E
#define OV8858_REG_LENC_GROUP2_START 0x711F
#define OV8858_REG_LENC_GROUP2_END 0x720F
#define OV8858_NUM_REG_LENC 240

typedef struct{
    uint16_t mid;
    uint16_t lenc[OV8858_NUM_REG_LENC];
    uint16_t lenc_cnt;
    uint32_t r_gain, g_gain, b_gain;
}ov8858_otp_data_t;
static ov8858_otp_data_t ov8858_otp_data = {0};
static struct msm_sensor_ctrl_t *ov8858_sctrl = NULL;
static int32_t ov8858_otp_i2c_write(uint32_t addr, uint16_t data);
static uint16_t ov8858_otp_i2c_read(uint32_t addr);
static uint32_t ov8858_otp_update_awb(void);
static uint32_t ov8858_otp_update_lenc(void);

int32_t ov8858_otp_config(struct msm_sensor_ctrl_t *s_ctrl)
{
	mutex_lock(s_ctrl->msm_sensor_mutex);
    ov8858_sctrl = s_ctrl;
    // awb
    ov8858_otp_update_awb();
    // lenc
    ov8858_otp_update_lenc();
	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return 0;
}

uint16_t ov8858_otp_get_mid(struct msm_sensor_ctrl_t *s_ctrl)
{
    uint32_t i = 0;
	uint16_t temp = 0, flag = 0, mid = 0;

	mutex_lock(s_ctrl->msm_sensor_mutex);
    if(0 != ov8858_otp_data.mid){
        pr_err("%s, use buff, mid:%d\n", __FUNCTION__, ov8858_otp_data.mid);
        mid = ov8858_otp_data.mid;
    }else{
        ov8858_sctrl = s_ctrl;
        // start stream
        ov8858_otp_i2c_write(0x0100, 0x01);
        // set 0x5002[3] to '0'
        temp = ov8858_otp_i2c_read(0x5002);
        ov8858_otp_i2c_write(0x5002, (temp & (~0x08)));
        //enable partial OTP read mode
        ov8858_otp_i2c_write(0x3d84, 0xC0);
        // partial mode OTP write start address
        ov8858_otp_i2c_write(0x3d88, ((OV8858_REG_MODULE_FLAG >> 8) & 0xff));
        ov8858_otp_i2c_write(0x3d89, (OV8858_REG_MODULE_FLAG & 0xff));
        // partial mode OTP write end address
        ov8858_otp_i2c_write(0x3d8A, ((OV8858_REG_MODULE_GROUP2_END >> 8) & 0xff));
        ov8858_otp_i2c_write(0x3d8B, (OV8858_REG_MODULE_GROUP2_END & 0xff));
        // read otp into buffer
        ov8858_otp_i2c_write(0x3d81, 0x01);
        mdelay(10);
        flag = ov8858_otp_i2c_read(OV8858_REG_MODULE_FLAG);
        if(0x40 == (flag & 0xC0)){ // group 1
            mid = ov8858_otp_i2c_read(OV8858_REG_MODULE_GROUP1_START);
        }else if(0x10 == (flag & 0x30)){ // group 2
            mid = ov8858_otp_i2c_read(OV8858_REG_MODULE_GROUP2_START);
        }else{
            pr_err("%s, module info flag error:%d\n", __FUNCTION__, flag);
        }
        if((0x40 == (flag & 0xC0)) || (0x10 == (flag & 0x30))){
            ov8858_otp_data.mid = mid;
            pr_err("%s, mid:0x%x\n", __FUNCTION__, mid);
        }
        // clear otp buffer
        for (i = OV8858_REG_MODULE_FLAG; i <= OV8858_REG_MODULE_GROUP2_END; i++){
        	ov8858_otp_i2c_write(i, 0x00);
        }
        // set 0x5002[3] to '1'
        temp = ov8858_otp_i2c_read(0x5002);
        ov8858_otp_i2c_write(0x5002, (temp | 0x08));
        // stop stream
        ov8858_otp_i2c_write(0x0100, 0x00);
    }
	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return mid;
}

static int32_t ov8858_otp_i2c_write(uint32_t addr, uint16_t data)
{
	int32_t rc = 0;

    if(NULL != ov8858_sctrl){
    	rc = ov8858_sctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(ov8858_sctrl->sensor_i2c_client, 
    		    addr, data, MSM_CAMERA_I2C_BYTE_DATA);
    }
    
	return rc;
}

static uint16_t ov8858_otp_i2c_read(uint32_t addr)
{
	uint16_t data = 0;

    if(NULL != ov8858_sctrl){
    	ov8858_sctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(ov8858_sctrl->sensor_i2c_client, 
    		addr, &data, MSM_CAMERA_I2C_BYTE_DATA);
    }
    
	return data;
}

static uint32_t ov8858_otp_update_awb()
{
    uint16_t temp = 0, flag = 0, rg_gain = 0, bg_gain = 0, rg_msb = 0, bg_msb = 0, lsb = 0, 
        typical_rg=0, typical_bg=0;
	uint32_t i = 0, r_g_gain=0, b_g_gain=0, g_g_gain=0, base_gain=0, r_gain=0, b_gain=0, g_gain=0;

    //start stream
    ov8858_otp_i2c_write(0x0100, 0x01);
    if(0 != ov8858_otp_data.r_gain ||
        0 != ov8858_otp_data.g_gain ||
        0 != ov8858_otp_data.b_gain){
        pr_err("%s, use buff, gain(rgb):(%d,%d,%d)\n", __FUNCTION__, ov8858_otp_data.r_gain, 
            ov8858_otp_data.g_gain, ov8858_otp_data.b_gain);
        if(0x400 < ov8858_otp_data.r_gain){
            ov8858_otp_i2c_write(0x5032, (ov8858_otp_data.r_gain >> 8));
            ov8858_otp_i2c_write(0x5033, (ov8858_otp_data.r_gain & 0xff));
        }
        if(0x400 < ov8858_otp_data.g_gain){
            ov8858_otp_i2c_write(0x5034, (ov8858_otp_data.g_gain >> 8));
            ov8858_otp_i2c_write(0x5035, (ov8858_otp_data.g_gain & 0xff));
        }
        if(0x400 < ov8858_otp_data.b_gain){
            ov8858_otp_i2c_write(0x5036, (ov8858_otp_data.b_gain >> 8));
            ov8858_otp_i2c_write(0x5037, (ov8858_otp_data.b_gain & 0xff));
        }
    }else{
        // set 0x5002[3] to '0'
        temp = ov8858_otp_i2c_read(0x5002);
        ov8858_otp_i2c_write(0x5002, (temp & (~0x08)));
        //enable partial OTP read mode
        ov8858_otp_i2c_write(0x3d84, 0xC0);
        // partial mode OTP write start address
        ov8858_otp_i2c_write(0x3d88, ((OV8858_REG_AWB_FLAG >> 8) & 0xff));
        ov8858_otp_i2c_write(0x3d89, (OV8858_REG_AWB_FLAG & 0xff));
        // partial mode OTP write end address
        ov8858_otp_i2c_write(0x3d8A, ((OV8858_REG_AWB_GROUP2_END >> 8) & 0xff));
        ov8858_otp_i2c_write(0x3d8B, (OV8858_REG_AWB_GROUP2_END & 0xff));
        // read otp into buffer
        ov8858_otp_i2c_write(0x3d81, 0x01);
        mdelay(10);
        flag = ov8858_otp_i2c_read(OV8858_REG_AWB_FLAG);
    	if(0x40 == (flag & 0xC0)){ // group 1
            rg_msb = ov8858_otp_i2c_read(OV8858_REG_AWB_GROUP1_START);
            bg_msb = ov8858_otp_i2c_read(OV8858_REG_AWB_GROUP1_START + 1);
    		lsb = ov8858_otp_i2c_read(OV8858_REG_AWB_GROUP1_END);
        }else if (0x10 == (flag & 0x30)){ // group 2
            rg_msb = ov8858_otp_i2c_read(OV8858_REG_AWB_GROUP2_START);
            bg_msb = ov8858_otp_i2c_read(OV8858_REG_AWB_GROUP2_START + 1);
    		lsb = ov8858_otp_i2c_read(OV8858_REG_AWB_GROUP2_END);
        }else{
            pr_err("%s, awb flag error:%d\n", __FUNCTION__, flag);
        }
        if((0x40 == (flag & 0xC0)) || (0x10 == (flag & 0x30))){
            rg_gain = (rg_msb << 2) | ((lsb >> 6) & 0x03);
            bg_gain = (bg_msb << 2) | ((lsb >> 4) & 0x03);
            if((0 != rg_gain) && (0 != bg_gain)){
                if(OV8858_MID_SUNRISE == ov8858_otp_data.mid){
                    typical_rg = OV8858_TYPICAL_RG_SUNRISE;
                    typical_bg = OV8858_TYPICAL_BG_SUNRISE;
                }else if(OV8858_MID_QTECH == ov8858_otp_data.mid){
                    typical_rg = OV8858_TYPICAL_RG_QTECH;
                    typical_bg = OV8858_TYPICAL_BG_QTECH;
                }
				/*[BUGFIX]-Mod-BEGIN by TCTSZ.(gaoxiang.zou@tcl.com), PR1193020 12/21/2015*/
				#if defined(JRD_PROJECT_PIXI464G) || defined(JRD_PROJECT_PIXI464GCRICKET)
				else if(OV8858_MID_SUNNY == ov8858_otp_data.mid){
                    typical_rg = OV8858_TYPICAL_RG_SUNNY;
                    typical_bg = OV8858_TYPICAL_BG_SUNNY;
					pr_err("%s, ov8858_sunny_zgx module:%d\n", __FUNCTION__, ov8858_otp_data.mid);
                }
				#endif
				/*[BUGFIX]-Mod-END by TCTSZ.(gaoxiang.zou@tcl.com), PR1193020*/
				else{
                    pr_err("%s, ov8858 module unknown:%d\n", __FUNCTION__, ov8858_otp_data.mid);
                }
                if((OV8858_MID_SUNRISE == ov8858_otp_data.mid) ||
                    (OV8858_MID_QTECH == ov8858_otp_data.mid)){
                    r_g_gain = (typical_rg * 1000) / rg_gain;
                    b_g_gain = (typical_bg * 1000) / bg_gain;
                    g_g_gain = 1000;
                    if (r_g_gain < 1000 || b_g_gain < 1000) 
                    {
                        if (r_g_gain < b_g_gain) {
                            base_gain = r_g_gain;
                        } else {
                            base_gain = b_g_gain;
                        }
                    } else {
                        base_gain = g_g_gain;
                    }
                    
                    r_gain = 0x400 * r_g_gain / (base_gain);
                    g_gain = 0x400 * g_g_gain / (base_gain);
                    b_gain = 0x400 * b_g_gain / (base_gain);
                    if (0x400 < r_gain){
                        ov8858_otp_i2c_write(0x5032, (r_gain >> 8));
                        ov8858_otp_i2c_write(0x5033, (r_gain & 0xff));
                    }
                    if (0x400 < g_gain){
                        ov8858_otp_i2c_write(0x5034, (g_gain >> 8));
                        ov8858_otp_i2c_write(0x5035, (g_gain & 0xff));
                    }
                    if (0x400 < b_gain){
                        ov8858_otp_i2c_write(0x5036, (b_gain >> 8));
                        ov8858_otp_i2c_write(0x5037, (b_gain & 0xff));
                    }
                    ov8858_otp_data.r_gain = r_gain;
                    ov8858_otp_data.g_gain = g_gain;
                    ov8858_otp_data.b_gain = b_gain;
                    pr_err("%s, gain(rgb):(%d, %d, %d), typical(rg, bg):(%d, %d)\n",
                        __FUNCTION__, r_gain, g_gain, b_gain, typical_rg, typical_bg);
                }
            }else{
                pr_err("%s, wrong value 0, gain(rg, bg):(%d, %d)\n", __FUNCTION__, rg_gain, bg_gain);
            }
        }
        // clear otp buffer
        for (i = OV8858_REG_AWB_FLAG; i <= OV8858_REG_AWB_GROUP2_END; i++){
        	ov8858_otp_i2c_write(i, 0x00);
        }
        // set 0x5002[3] to '1'
        temp = ov8858_otp_i2c_read(0x5002);
        ov8858_otp_i2c_write(0x5002, (temp | 0x08));
    }
    //stop stream
    ov8858_otp_i2c_write(0x0100, 0x00);

    return 0;
}

static uint32_t ov8858_otp_update_lenc()
{
    uint16_t temp = 0, flag = 0, lenc[OV8858_NUM_REG_LENC] = {0};
    uint32_t start_addr = 0, end_addr = 0, i = 0, j = 0, sum_lenc = 0, checksum = 0;

    // start stream
    ov8858_otp_i2c_write(0x0100, 0x01);
    if(0 < ov8858_otp_data.lenc_cnt){
        pr_err("%s, use buff, num:%d\n", __FUNCTION__, ov8858_otp_data.lenc_cnt);
        temp = ov8858_otp_i2c_read(0x5000);
        ov8858_otp_i2c_write(0x5000, (temp | 0x80));
        for(i = 0; i < ov8858_otp_data.lenc_cnt; i++){
            ov8858_otp_i2c_write((0x5800 + i), ov8858_otp_data.lenc[i]);
#ifdef OV8858_DEBUG
            pr_err("%s, buff write, %d, 0x%x:0x%x\n", __FUNCTION__,
                i, (0x5800 + i), ov8858_otp_data.lenc[i]);
#endif
        }
    }else{
        // set 0x5002[3] to '0'
        temp = ov8858_otp_i2c_read(0x5002);
        ov8858_otp_i2c_write(0x5002, (temp & (~0x08)));
        //enable partial OTP read mode
        ov8858_otp_i2c_write(0x3d84, 0xC0);
        // partial mode OTP write start address
        ov8858_otp_i2c_write(0x3d88, ((OV8858_REG_LENC_FLAG >> 8) & 0xff));
        ov8858_otp_i2c_write(0x3d89, (OV8858_REG_LENC_FLAG & 0xff));
        // partial mode OTP write end address
        ov8858_otp_i2c_write(0x3d8A, ((OV8858_REG_LENC_FLAG >> 8) & 0xff));
        ov8858_otp_i2c_write(0x3d8B, (OV8858_REG_LENC_FLAG & 0xff));
        // read otp into buffer
        ov8858_otp_i2c_write(0x3d81, 0x01);
        mdelay(10);
        flag = ov8858_otp_i2c_read(OV8858_REG_LENC_FLAG);
    	if(0x10 == (flag & 0x30)){ // group 2
            start_addr = OV8858_REG_LENC_GROUP2_START;
            end_addr = OV8858_REG_LENC_GROUP2_END;
        }else if(0x40 == (flag & 0xC0)){ // group 1
    		start_addr = OV8858_REG_LENC_GROUP1_START;
            end_addr = OV8858_REG_LENC_GROUP1_END;
        }else{
            pr_err("%s, lenc flag error:%d\n", __FUNCTION__, flag);
        }
        
        if((0x40 == (flag & 0xC0)) || (0x10 == (flag & 0x30))){
            // partial mode OTP write start address
            ov8858_otp_i2c_write(0x3d88, ((start_addr >> 8) & 0xff));
            ov8858_otp_i2c_write(0x3d89, (start_addr & 0xff));
            // partial mode OTP write end address
            ov8858_otp_i2c_write(0x3d8A, ((end_addr >> 8) & 0xff));
            ov8858_otp_i2c_write(0x3d8B, (end_addr & 0xff));
            // read otp into buffer
            ov8858_otp_i2c_write(0x3d81, 0x01);
            mdelay(10);
            for(i = 0; i <= (end_addr - 1 - start_addr); i++){
                if(OV8858_NUM_REG_LENC <= i){
                    pr_err("%s, lenc number is over limited:%d\n", __FUNCTION__, i);
                    break;
                }
                lenc[i] = ov8858_otp_i2c_read(start_addr + i);
                sum_lenc += lenc[i];
#ifdef OV8858_DEBUG
                pr_err("%s, read, %d, 0x%x:0x%x\n", __FUNCTION__,
                    i, (start_addr + i), lenc[i]);
#endif
            }
            sum_lenc = (sum_lenc % 255 + 1);
            checksum = ov8858_otp_i2c_read(end_addr);
            if(checksum == sum_lenc){
                temp = ov8858_otp_i2c_read(0x5000);
                ov8858_otp_i2c_write(0x5000, (temp | 0x80));
                for(j = 0; j < i; j++){
                    ov8858_otp_i2c_write((0x5800 + j), lenc[j]);
                    ov8858_otp_data.lenc[j] = lenc[j];
#ifdef OV8858_DEBUG
                    pr_err("%s, write, %d, 0x%x:0x%x\n", __FUNCTION__,
                        j, (0x5800 + j), lenc[j]);
#endif
                }
                ov8858_otp_data.lenc_cnt = i;
                pr_err("%s, success, num:%d, check sum:%d\n", __FUNCTION__, i, checksum);
            }else{
                pr_err("%s, checksum error, sum lenc:%d, check sum:%d\n",
                    __FUNCTION__, sum_lenc, checksum);
            }
            // clear otp buffer
            for (i = start_addr; i <= end_addr; i++){
                ov8858_otp_i2c_write(i, 0x00);
            }
        }
        // clear otp buffer
    	ov8858_otp_i2c_write(OV8858_REG_LENC_FLAG, 0x00);
        // set 0x5002[3] to '1'
        temp = ov8858_otp_i2c_read(0x5002);
        ov8858_otp_i2c_write(0x5002, (temp | 0x08));
    }
    //stop stream
    ov8858_otp_i2c_write(0x0100, 0x00);

    return 0;
}
