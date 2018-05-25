#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"
#include "ov8865_otp.h"

//#define OV8865_DEBUG
#define OV8865_TYPICAL_RG_SUNRISE 327 // 0x147
#define OV8865_TYPICAL_BG_SUNRISE 288 // 0x120
#define OV8865_TYPICAL_RG_QTECH 308 // 0x134
#define OV8865_TYPICAL_BG_QTECH 302 // 0x12E

#define OV8865_REG_MODULE_FLAG 0x7010
#define OV8865_REG_MODULE_GROUP1_START 0x7011
#define OV8865_REG_MODULE_GROUP1_END 0x7015
#define OV8865_REG_MODULE_GROUP2_START 0x7016
#define OV8865_REG_MODULE_GROUP2_END 0x701A
#define OV8865_REG_MODULE_GROUP3_START 0x701B
#define OV8865_REG_MODULE_GROUP3_END 0x701F
#define OV8865_REG_AWB_FLAG 0x7020
#define OV8865_REG_AWB_GROUP1_START 0x7021
#define OV8865_REG_AWB_GROUP1_END 0x7025
#define OV8865_REG_AWB_GROUP2_START 0x7026
#define OV8865_REG_AWB_GROUP2_END 0x702A
#define OV8865_REG_AWB_GROUP3_START 0x702B
#define OV8865_REG_AWB_GROUP3_END 0x702F
#define OV8865_REG_VCM_FLAG 0x7030
#define OV8865_REG_VCM_GROUP1_START 0x7031
#define OV8865_REG_VCM_GROUP1_END 0x7033
#define OV8865_REG_VCM_GROUP2_START 0x7034
#define OV8865_REG_VCM_GROUP2_END 0x7036
#define OV8865_REG_VCM_GROUP3_START 0x7037
#define OV8865_REG_VCM_GROUP3_END 0x7039
#define OV8865_REG_LENC_FLAG 0x703A
#define OV8865_REG_LENC_GROUP1_START 0x703B
#define OV8865_REG_LENC_GROUP1_END 0x7078
#define OV8865_REG_LENC_GROUP2_START 0x7079
#define OV8865_REG_LENC_GROUP2_END 0x70B6
#define OV8865_REG_LENC_GROUP3_START 0x70B7
#define OV8865_REG_LENC_GROUP3_END 0x70F4
#define OV8865_NUM_REG_LENC 240

typedef struct{
    uint16_t mid;
    uint16_t lenc[OV8865_NUM_REG_LENC];
    uint16_t lenc_cnt;
    uint32_t r_gain, g_gain, b_gain;
}ov8865_otp_data_t;
static ov8865_otp_data_t ov8865_otp_data = {0};
static struct msm_sensor_ctrl_t *ov8865_sctrl = NULL;
static int32_t ov8865_otp_i2c_write(uint32_t addr, uint16_t data);
static uint16_t ov8865_otp_i2c_read(uint32_t addr);
static uint32_t ov8865_otp_update_awb(void);
static uint32_t ov8865_otp_update_lenc(void);

int32_t ov8865_otp_config(struct msm_sensor_ctrl_t *s_ctrl)
{
	mutex_lock(s_ctrl->msm_sensor_mutex);
    ov8865_sctrl = s_ctrl;
    // awb
    ov8865_otp_update_awb();
    // lenc
    ov8865_otp_update_lenc();
	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return 0;
}

uint16_t ov8865_otp_get_mid(struct msm_sensor_ctrl_t *s_ctrl)
{
    uint32_t i = 0;
//	uint16_t temp = 0;
	uint16_t flag = 0, mid = 0;

    if(0 != ov8865_otp_data.mid){
        pr_err("%s, use buff, mid:%d\n", __FUNCTION__, ov8865_otp_data.mid);
        return ov8865_otp_data.mid;
    }

    mutex_lock(s_ctrl->msm_sensor_mutex);
	
    ov8865_sctrl = s_ctrl;
    // start stream
    ov8865_otp_i2c_write(0x0100, 0x01);
    // set 0x5002[3] to '0'
    //temp = ov8865_otp_i2c_read(0x5002);
    //ov8865_otp_i2c_write(0x5002, (temp & (~0x08)));
    //enable partial OTP read mode
    ov8865_otp_i2c_write(0x3d84, 0xC0);
    // partial mode OTP write start address
    ov8865_otp_i2c_write(0x3d88, ((OV8865_REG_MODULE_FLAG >> 8) & 0xff));
    ov8865_otp_i2c_write(0x3d89, (OV8865_REG_MODULE_FLAG & 0xff));
    // partial mode OTP write end address
    ov8865_otp_i2c_write(0x3d8A, ((OV8865_REG_MODULE_GROUP3_END >> 8) & 0xff));
    ov8865_otp_i2c_write(0x3d8B, (OV8865_REG_MODULE_GROUP3_END & 0xff));
    // read otp into buffer
    ov8865_otp_i2c_write(0x3d81, 0x01);
    mdelay(5);
    flag = ov8865_otp_i2c_read(OV8865_REG_MODULE_FLAG);
    
    if(0x40 == (flag & 0xC0)){ // group 1
        mid = ov8865_otp_i2c_read(OV8865_REG_MODULE_GROUP1_START);
    }else if(0x10 == (flag & 0x30)){ // group 2
        mid = ov8865_otp_i2c_read(OV8865_REG_MODULE_GROUP2_START);
    }else if(0x04 == (flag & 0x0C)){ // group 3
        mid = ov8865_otp_i2c_read(OV8865_REG_MODULE_GROUP3_START);
    }else{
        pr_err("%s, module info flag error:%d\n", __FUNCTION__, flag);
    }
    
    if((0x40 == (flag & 0xC0)) || (0x10 == (flag & 0x30)) || (0x04 == (flag & 0x0C))){
        ov8865_otp_data.mid = mid;
        pr_err("%s, mid:0x%x\n", __FUNCTION__, mid);
    }
    
    // clear otp buffer
    for (i = OV8865_REG_MODULE_FLAG; i <= OV8865_REG_MODULE_GROUP3_END; i++){
    	ov8865_otp_i2c_write(i, 0x00);
    }
    // set 0x5002[3] to '1'
    //temp = ov8865_otp_i2c_read(0x5002);
    //ov8865_otp_i2c_write(0x5002, (temp | 0x08));
    // stop stream
    ov8865_otp_i2c_write(0x0100, 0x00);
    
	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return mid;
}

static int32_t ov8865_otp_i2c_write(uint32_t addr, uint16_t data)
{
	int32_t rc = 0;

    if(NULL != ov8865_sctrl){
    	rc = ov8865_sctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(ov8865_sctrl->sensor_i2c_client, 
    		    addr, data, MSM_CAMERA_I2C_BYTE_DATA);
    }
    
	return rc;
}

static uint16_t ov8865_otp_i2c_read(uint32_t addr)
{
	uint16_t data = 0;

    if(NULL != ov8865_sctrl){
    	ov8865_sctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(ov8865_sctrl->sensor_i2c_client, 
    		addr, &data, MSM_CAMERA_I2C_BYTE_DATA);
    }
    
	return data;
}

static uint32_t ov8865_otp_update_awb()
{
//    uint16_t temp = 0;
    uint16_t flag = 0, rg_gain = 0, bg_gain = 0, light_rg = 0, light_bg = 0, lsb = 0, typical_rg = 0, typical_bg = 0;
	uint32_t i = 0, r_g_gain = 0, b_g_gain = 0, g_g_gain = 0, base_gain = 0, r_gain = 0, b_gain = 0, g_gain = 0;

    if(0 != ov8865_otp_data.r_gain || 0 != ov8865_otp_data.g_gain || 0 != ov8865_otp_data.b_gain){
        pr_err("%s, use buff, gain(rgb):(%d,%d,%d)\n", __FUNCTION__, ov8865_otp_data.r_gain, 
            ov8865_otp_data.g_gain, ov8865_otp_data.b_gain);
            
        //start stream
        ov8865_otp_i2c_write(0x0100, 0x01);
        if(0x400 < ov8865_otp_data.r_gain){
            ov8865_otp_i2c_write(0x5018, (ov8865_otp_data.r_gain >> 6));
            ov8865_otp_i2c_write(0x5019, (ov8865_otp_data.r_gain & 0x3f));
        }
        if(0x400 < ov8865_otp_data.g_gain){
            ov8865_otp_i2c_write(0x501A, (ov8865_otp_data.g_gain >> 6));
            ov8865_otp_i2c_write(0x501B, (ov8865_otp_data.g_gain & 0x3f));
        }
        if(0x400 < ov8865_otp_data.b_gain){
            ov8865_otp_i2c_write(0x501C, (ov8865_otp_data.b_gain >> 6));
            ov8865_otp_i2c_write(0x501D, (ov8865_otp_data.b_gain & 0x3f));
        }
        //stop stream
        ov8865_otp_i2c_write(0x0100, 0x00);
        return 0;
    }
    
#if 0
    if(OV8865_MID_SUNRISE == ov8865_otp_data.mid){
        typical_rg = OV8865_TYPICAL_RG_SUNRISE;
        typical_bg = OV8865_TYPICAL_BG_SUNRISE;
    }else if(OV8865_MID_QTECH == ov8865_otp_data.mid){
        typical_rg = OV8865_TYPICAL_RG_QTECH;
        typical_bg = OV8865_TYPICAL_BG_QTECH;
    }else{
        pr_err("%s, ov8865 module unknown:%d\n", __FUNCTION__, ov8865_otp_data.mid);
    }
#endif
    if(0 < typical_rg && 0 < typical_bg){
        //start stream
        ov8865_otp_i2c_write(0x0100, 0x01);
        // set 0x5002[3] to '0'
        //temp = ov8865_otp_i2c_read(0x5002);
        //ov8865_otp_i2c_write(0x5002, (temp & (~0x08)));
        //enable partial OTP read mode
        ov8865_otp_i2c_write(0x3d84, 0xC0);
        // partial mode OTP write start address
        ov8865_otp_i2c_write(0x3d88, ((OV8865_REG_AWB_FLAG >> 8) & 0xff));
        ov8865_otp_i2c_write(0x3d89, (OV8865_REG_AWB_FLAG & 0xff));
        // partial mode OTP write end address
        ov8865_otp_i2c_write(0x3d8A, ((OV8865_REG_AWB_GROUP3_END >> 8) & 0xff));
        ov8865_otp_i2c_write(0x3d8B, (OV8865_REG_AWB_GROUP3_END & 0xff));
        // read otp into buffer
        ov8865_otp_i2c_write(0x3d81, 0x01);
        mdelay(5);
        flag = ov8865_otp_i2c_read(OV8865_REG_AWB_FLAG);
        
    	if(0x40 == (flag & 0xC0)){ // group 1
            rg_gain = ov8865_otp_i2c_read(OV8865_REG_AWB_GROUP1_START);
            bg_gain = ov8865_otp_i2c_read(OV8865_REG_AWB_GROUP1_START + 1);
            light_rg = ov8865_otp_i2c_read(OV8865_REG_AWB_GROUP1_START + 2);
            light_bg = ov8865_otp_i2c_read(OV8865_REG_AWB_GROUP1_START + 3);
    		lsb = ov8865_otp_i2c_read(OV8865_REG_AWB_GROUP1_START + 4);
        }else if (0x10 == (flag & 0x30)){ // group 2
            rg_gain = ov8865_otp_i2c_read(OV8865_REG_AWB_GROUP2_START);
            bg_gain = ov8865_otp_i2c_read(OV8865_REG_AWB_GROUP2_START + 1);
            light_rg = ov8865_otp_i2c_read(OV8865_REG_AWB_GROUP2_START + 2);
            light_bg = ov8865_otp_i2c_read(OV8865_REG_AWB_GROUP2_START + 3);
    		lsb = ov8865_otp_i2c_read(OV8865_REG_AWB_GROUP2_START + 4);
        }else if (0x04 == (flag & 0x0C)){ // group 3
            rg_gain = ov8865_otp_i2c_read(OV8865_REG_AWB_GROUP3_START);
            bg_gain = ov8865_otp_i2c_read(OV8865_REG_AWB_GROUP3_START + 1);
            light_rg = ov8865_otp_i2c_read(OV8865_REG_AWB_GROUP3_START + 2);
            light_bg = ov8865_otp_i2c_read(OV8865_REG_AWB_GROUP3_START + 3);
    		lsb = ov8865_otp_i2c_read(OV8865_REG_AWB_GROUP3_START + 4);
        }else{
            pr_err("%s, awb flag error:%d\n", __FUNCTION__, flag);
        }
        
        if((0x40 == (flag & 0xC0)) || (0x10 == (flag & 0x30)) || (0x04 == (flag & 0x0C))){
            rg_gain = (rg_gain << 2) | ((lsb >> 6) & 0x03);
            bg_gain = (bg_gain << 2) | ((lsb >> 4) & 0x03);
            light_rg = (light_rg << 2) | ((lsb >> 2) & 0x03);
            light_bg = (light_bg << 2) | (lsb & 0x03);
            
            // light source information in OTP
            if(0 < light_rg){
                rg_gain = rg_gain * (light_rg + 512) / 1024;
            }
            if(0 < light_bg){
                bg_gain = bg_gain * (light_bg + 512) / 1024;
            }
            
            if((0 != rg_gain) && (0 != bg_gain)){
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
                    ov8865_otp_i2c_write(0x5018, (r_gain >> 6));
                    ov8865_otp_i2c_write(0x5019, (r_gain & 0x3f));
                }
                if (0x400 < g_gain){
                    ov8865_otp_i2c_write(0x501A, (g_gain >> 6));
                    ov8865_otp_i2c_write(0x501B, (g_gain & 0x3f));
                }
                if (0x400 < b_gain){
                    ov8865_otp_i2c_write(0x501C, (b_gain >> 6));
                    ov8865_otp_i2c_write(0x501D, (b_gain & 0x3f));
                }
                
                ov8865_otp_data.r_gain = r_gain;
                ov8865_otp_data.g_gain = g_gain;
                ov8865_otp_data.b_gain = b_gain;
                pr_err("%s, gain(rgb):(%d, %d, %d), typical(rg, bg):(%d, %d)\n",
                    __FUNCTION__, r_gain, g_gain, b_gain, typical_rg, typical_bg);
            }else{
                pr_err("%s, error, gain(rg, bg):(%d, %d)\n", __FUNCTION__, rg_gain, bg_gain);
            }
        }
        // clear otp buffer
        for (i = OV8865_REG_AWB_FLAG; i <= OV8865_REG_AWB_GROUP3_END; i++){
        	ov8865_otp_i2c_write(i, 0x00);
        }
        // set 0x5002[3] to '1'
        //temp = ov8865_otp_i2c_read(0x5002);
        //ov8865_otp_i2c_write(0x5002, (temp | 0x08));
        //stop stream
        ov8865_otp_i2c_write(0x0100, 0x00);
    }else{
        pr_err("%s, error, typical(rg, bg):(%d, %d)\n", __FUNCTION__, typical_rg, typical_bg);
    }

    return 0;
}

static uint32_t ov8865_otp_update_lenc()
{
    uint16_t temp = 0;
    uint16_t flag = 0, lenc[OV8865_NUM_REG_LENC] = {0};
    uint32_t start_addr = 0, end_addr = 0, i = 0, j = 0;

    if(0 < ov8865_otp_data.lenc_cnt){
        pr_err("%s, use buff, num:%d\n", __FUNCTION__, ov8865_otp_data.lenc_cnt);
        
        // start stream
        ov8865_otp_i2c_write(0x0100, 0x01);
        temp = ov8865_otp_i2c_read(0x5000);
        ov8865_otp_i2c_write(0x5000, (temp | 0x80));
        
        for(i = 0; i < ov8865_otp_data.lenc_cnt; i++){
            ov8865_otp_i2c_write((0x5800 + i), ov8865_otp_data.lenc[i]);
#ifdef OV8865_DEBUG
            pr_err("%s, buff write, %d, 0x%x:0x%x\n", __FUNCTION__,
                i, (0x5800 + i), ov8865_otp_data.lenc[i]);
#endif
        }
        //stop stream
        ov8865_otp_i2c_write(0x0100, 0x00);
        
        return 0;
    }
    
    // start stream
    ov8865_otp_i2c_write(0x0100, 0x01);
    // set 0x5002[3] to '0'
    //temp = ov8865_otp_i2c_read(0x5002);
    //ov8865_otp_i2c_write(0x5002, (temp & (~0x08)));
    //enable partial OTP read mode
    ov8865_otp_i2c_write(0x3d84, 0xC0);
    // partial mode OTP write start address
    ov8865_otp_i2c_write(0x3d88, ((OV8865_REG_LENC_FLAG >> 8) & 0xff));
    ov8865_otp_i2c_write(0x3d89, (OV8865_REG_LENC_FLAG & 0xff));
    // partial mode OTP write end address
    ov8865_otp_i2c_write(0x3d8A, ((OV8865_REG_LENC_FLAG >> 8) & 0xff));
    ov8865_otp_i2c_write(0x3d8B, (OV8865_REG_LENC_FLAG & 0xff));
    // read otp into buffer
    ov8865_otp_i2c_write(0x3d81, 0x01);
    mdelay(5);
    flag = ov8865_otp_i2c_read(OV8865_REG_LENC_FLAG);
    
	if(0x40 == (flag & 0xC0)){ // group 1
		start_addr = OV8865_REG_LENC_GROUP1_START;
        end_addr = OV8865_REG_LENC_GROUP1_END;
    }else if(0x10 == (flag & 0x30)){ // group 2
        start_addr = OV8865_REG_LENC_GROUP2_START;
        end_addr = OV8865_REG_LENC_GROUP2_END;
    }else if(0x04 == (flag & 0x0C)){ // group 3
        start_addr = OV8865_REG_LENC_GROUP3_START;
        end_addr = OV8865_REG_LENC_GROUP3_END;
    }else{
        pr_err("%s, lenc flag error:%d\n", __FUNCTION__, flag);
    }
    
    if((0x40 == (flag & 0xC0)) || (0x10 == (flag & 0x30)) || (0x04 == (flag & 0x0C))){
        // partial mode OTP write start address
        ov8865_otp_i2c_write(0x3d88, ((start_addr >> 8) & 0xff));
        ov8865_otp_i2c_write(0x3d89, (start_addr & 0xff));
        // partial mode OTP write end address
        ov8865_otp_i2c_write(0x3d8A, ((end_addr >> 8) & 0xff));
        ov8865_otp_i2c_write(0x3d8B, (end_addr & 0xff));
        // read otp into buffer
        ov8865_otp_i2c_write(0x3d81, 0x01);
        mdelay(10);
        
        for(i = 0; i < (end_addr - start_addr + 1); i++){
            if(OV8865_NUM_REG_LENC <= i){
                pr_err("%s, lenc number is over limited:%d\n", __FUNCTION__, i);
                break;
            }
            
            lenc[i] = ov8865_otp_i2c_read(start_addr + i);
#ifdef OV8865_DEBUG
            pr_err("%s, read, %d, 0x%x:0x%x\n", __FUNCTION__,
                i, (start_addr + i), lenc[i]);
#endif
        }
        
        temp = ov8865_otp_i2c_read(0x5000);
        ov8865_otp_i2c_write(0x5000, (temp | 0x80));
        for(j = 0; j < i; j++){
            ov8865_otp_i2c_write((0x5800 + j), lenc[j]);
            ov8865_otp_data.lenc[j] = lenc[j];
#ifdef OV8865_DEBUG
            pr_err("%s, write, %d, 0x%x:0x%x\n", __FUNCTION__,
                j, (0x5800 + j), lenc[j]);
#endif
        }
        ov8865_otp_data.lenc_cnt = i;
        pr_err("%s, success, num:%d\n", __FUNCTION__, i);
        // clear otp buffer
        for (i = start_addr; i <= end_addr; i++){
            ov8865_otp_i2c_write(i, 0x00);
        }
    }
    // clear otp buffer
	ov8865_otp_i2c_write(OV8865_REG_LENC_FLAG, 0x00);
    // set 0x5002[3] to '1'
    //temp = ov8865_otp_i2c_read(0x5002);
    //ov8865_otp_i2c_write(0x5002, (temp | 0x08));
    //stop stream
    ov8865_otp_i2c_write(0x0100, 0x00);

    return 0;
}
