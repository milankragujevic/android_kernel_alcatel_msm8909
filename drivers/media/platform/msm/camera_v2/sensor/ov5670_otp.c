#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"
#include "ov5670_otp.h"

#define OV5670_REG_AWB_FLAG 0x7026
#define OV5670_REG_AWB_GROUP1_START 0x7027
#define OV5670_REG_AWB_GROUP1_END 0x702F
#define OV5670_REG_AWB_GROUP2_START 0x7030
#define OV5670_REG_AWB_GROUP2_END 0x7038
#define OV5670_REG_AWB_GROUP3_START 0x7039
#define OV5670_REG_AWB_GROUP3_END 0x7041
#define OV5670_TYPICAL_RG 611
#define OV5670_TYPICAL_BG 611

//SZ xuejian.zhong@tcl.com add 2016.03.07
#if defined(JRD_PROJECT_PIXI464G) || defined(JRD_PROJECT_PIXI464GCRICKET)
#define OV5670_TYPICAL_RG_QTECH 573
#define OV5670_TYPICAL_BG_QTECH 586

#define OV5670_TYPICAL_RG_EWELLY  611
#define OV5670_TYPICAL_BG_EWELLY  611

#define OTP_MID_INFO_STARTADDR 0x7010
#define OTP_MID_INFO_ENDADDR   0x701f 
#endif
//end


typedef struct{
	uint16_t r_gain, g_gain, b_gain;
}ov5670_otp_data_t;
static ov5670_otp_data_t ov5670_otp_data = {0};
static struct msm_sensor_ctrl_t *ov5670_sctrl = NULL;
static int32_t ov5670_otp_i2c_write(uint32_t addr, uint16_t data);
static uint16_t ov5670_otp_i2c_read(uint32_t addr);
static uint32_t ov5670_otp_update_awb(void);
//SZ xuejian.zhong@tcl.com add 2016.03.07
#if defined(JRD_PROJECT_PIXI464G) || defined(JRD_PROJECT_PIXI464GCRICKET)
static uint16_t ov5670_mid = 0;
#endif
//end
int32_t ov5670_otp_config(struct msm_sensor_ctrl_t *s_ctrl)
{
	mutex_lock(s_ctrl->msm_sensor_mutex);
	ov5670_sctrl = s_ctrl;
    // awb
    ov5670_otp_update_awb();
	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return 0;
}

static int32_t ov5670_otp_i2c_write(uint32_t addr, uint16_t data)
{
	int32_t rc = 0;

	if(NULL != ov5670_sctrl){
		rc = ov5670_sctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(ov5670_sctrl->sensor_i2c_client, 
			    addr, data, MSM_CAMERA_I2C_BYTE_DATA);
	}
	
	return rc;
}

static uint16_t ov5670_otp_i2c_read(uint32_t addr)
{
	uint16_t data = 0;

	if(NULL != ov5670_sctrl){
		ov5670_sctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(ov5670_sctrl->sensor_i2c_client, 
			addr, &data, MSM_CAMERA_I2C_BYTE_DATA);
	}
	
	return data;
}

static uint32_t ov5670_otp_update_awb()
{
    uint16_t temp = 0, flag = 0, rg_gain = 0, bg_gain = 0, rg_msb = 0, bg_msb = 0, lsb = 0,
        typical_rg=0, typical_bg=0;
    uint32_t i = 0, r_g_gain=0, b_g_gain=0, g_g_gain=0, base_gain=0, r_gain=0, b_gain=0, g_gain=0;

	//start stream
	ov5670_otp_i2c_write(0x0100, 0x01);
	if(0 != ov5670_otp_data.r_gain ||
		0 != ov5670_otp_data.g_gain ||
		0 != ov5670_otp_data.b_gain){
		pr_err("%s, use buff, gain(rgb):(%d,%d,%d)\n", __FUNCTION__, ov5670_otp_data.r_gain,
			ov5670_otp_data.g_gain, ov5670_otp_data.b_gain);
        if (0x400 < ov5670_otp_data.r_gain){
            ov5670_otp_i2c_write(0x5032, (ov5670_otp_data.r_gain >> 8));
            ov5670_otp_i2c_write(0x5033, (ov5670_otp_data.r_gain & 0xff));
        }
        if (0x400 < ov5670_otp_data.g_gain){
            ov5670_otp_i2c_write(0x5034, (ov5670_otp_data.g_gain >> 8));
            ov5670_otp_i2c_write(0x5035, (ov5670_otp_data.g_gain & 0xff));
        }
        if (0x400 < ov5670_otp_data.b_gain){
            ov5670_otp_i2c_write(0x5036, (ov5670_otp_data.b_gain >> 8));
            ov5670_otp_i2c_write(0x5037, (ov5670_otp_data.b_gain & 0xff));
        }
	}else{
	    // set 0x5002[1] to 0
	    temp = ov5670_otp_i2c_read(0x5002);
	    ov5670_otp_i2c_write(0x5002, (temp & (~0x02)));
	    //enable partial OTP read mode
	    ov5670_otp_i2c_write(0x3d84, 0xC0);
	    // partial mode OTP write start address
	    ov5670_otp_i2c_write(0x3d88, ((OV5670_REG_AWB_FLAG >> 8) & 0xff));
	    ov5670_otp_i2c_write(0x3d89, (OV5670_REG_AWB_FLAG & 0xff));
	    // partial mode OTP write end address
	    ov5670_otp_i2c_write(0x3d8A, ((OV5670_REG_AWB_GROUP3_END >> 8) & 0xff));
	    ov5670_otp_i2c_write(0x3d8B, (OV5670_REG_AWB_GROUP3_END & 0xff));
	    // read otp into buffer
	    ov5670_otp_i2c_write(0x3d81, 0x01);
	    mdelay(10);
	    flag = ov5670_otp_i2c_read(OV5670_REG_AWB_FLAG);
	    if(0x40 == (flag & 0xC0)){ // group 1
	        rg_msb = ov5670_otp_i2c_read(OV5670_REG_AWB_GROUP1_START + 1);
	        bg_msb = ov5670_otp_i2c_read(OV5670_REG_AWB_GROUP1_START + 2);
	        lsb = ov5670_otp_i2c_read(OV5670_REG_AWB_GROUP1_START + 4);
	    }else if(0x10 == (flag & 0x30)){ // group 2
	        rg_msb = ov5670_otp_i2c_read(OV5670_REG_AWB_GROUP2_START + 1);
	        bg_msb = ov5670_otp_i2c_read(OV5670_REG_AWB_GROUP2_START + 2);
	        lsb = ov5670_otp_i2c_read(OV5670_REG_AWB_GROUP2_START + 4);
	    }else if(0x04 == (flag & 0x0C)){ // group 3
	        rg_msb = ov5670_otp_i2c_read(OV5670_REG_AWB_GROUP3_START + 1);
	        bg_msb = ov5670_otp_i2c_read(OV5670_REG_AWB_GROUP3_START + 2);
	        lsb = ov5670_otp_i2c_read(OV5670_REG_AWB_GROUP3_START + 4);
	    }else{
	        pr_err("%s, awb flag error:%d\n", __FUNCTION__, flag);
	    }
	    if((0x40 == (flag & 0xC0)) || (0x10 == (flag & 0x30)) || (0x04 == (flag & 0x0C))){
	        rg_gain = (rg_msb << 2) | ((lsb >> 6) & 0x03);
	        bg_gain = (bg_msb << 2) | ((lsb >> 4) & 0x03);
	        if((0 != rg_gain) && (0 != bg_gain)){
			//SZ xuejian.zhong@tcl.com add 2016.03.07
#if defined(JRD_PROJECT_PIXI464G) || defined(JRD_PROJECT_PIXI464GCRICKET)
              typical_rg = ov5670_mid == OV5670_MID_QTECH ? OV5670_TYPICAL_RG_QTECH : OV5670_TYPICAL_RG_EWELLY;
              typical_bg = ov5670_mid == OV5670_MID_QTECH ? OV5670_TYPICAL_BG_QTECH : OV5670_TYPICAL_BG_EWELLY;

#else
                typical_rg = OV5670_TYPICAL_RG;
                typical_bg = OV5670_TYPICAL_BG;
#endif
//end
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
	                ov5670_otp_i2c_write(0x5032, (r_gain >> 8));
	                ov5670_otp_i2c_write(0x5033, (r_gain & 0xff));
	            }
	            if (0x400 < g_gain){
	                ov5670_otp_i2c_write(0x5034, (g_gain >> 8));
	                ov5670_otp_i2c_write(0x5035, (g_gain & 0xff));
	            }
	            if (0x400 < b_gain){
	                ov5670_otp_i2c_write(0x5036, (b_gain >> 8));
	                ov5670_otp_i2c_write(0x5037, (b_gain & 0xff));
	            }
				ov5670_otp_data.r_gain = r_gain;
				ov5670_otp_data.g_gain = g_gain;
				ov5670_otp_data.b_gain = b_gain;
	            pr_err("%s, gain(rgb):(%d, %d, %d), typical(rg, bg):(%d, %d)\n",
	                __FUNCTION__, r_gain, g_gain, b_gain, typical_rg, typical_bg);
	        }else{
	            pr_err("%s, wrong value 0, gain(rg, bg):(%d, %d)\n", __FUNCTION__, rg_gain, bg_gain);
	        }
	    }
	    // clear otp buffer
	    for (i = OV5670_REG_AWB_FLAG; i <= OV5670_REG_AWB_GROUP3_END; i++){
	    	ov5670_otp_i2c_write(i, 0x00);
	    }
	    //set 0x5002[1] to 1
	    temp = ov5670_otp_i2c_read(0x5002);
	    ov5670_otp_i2c_write(0x5002, (temp | 0x02));
	}
	//stop stream
	ov5670_otp_i2c_write(0x0100, 0x00);

    return 0;
}
			//SZ xuejian.zhong@tcl.com add 2016.03.07
#if defined(JRD_PROJECT_PIXI464G) || defined(JRD_PROJECT_PIXI464GCRICKET)
uint16_t ov5670_otp_get_mid(struct msm_sensor_ctrl_t *s_ctrl)
{
	  uint16_t  i, mid_address ,temp = 0, flag = 0;
	  
	  mutex_lock(s_ctrl->msm_sensor_mutex);
    if(0 != ov5670_mid){
        pr_err("%s, use buff, MID:%x\n", __FUNCTION__, ov5670_mid);
        mutex_unlock(s_ctrl->msm_sensor_mutex);
        return ov5670_mid;
    }else{
        ov5670_sctrl = s_ctrl;
        // start stream
        ov5670_otp_i2c_write(0x0100, 0x01);
        // set 0x5002[3] to '0'
        temp = ov5670_otp_i2c_read(0x5002);
        ov5670_otp_i2c_write(0x5002, (temp & (~0x08)));
        //enable partial OTP read mode
        ov5670_otp_i2c_write(0x3d84, 0xC0);
        // partial mode OTP write start address
        ov5670_otp_i2c_write(0x3d88, ((OTP_MID_INFO_STARTADDR >> 8) & 0xff));
        ov5670_otp_i2c_write(0x3d89, (OTP_MID_INFO_STARTADDR & 0xff));
        // partial mode OTP write end address
        ov5670_otp_i2c_write(0x3d8A, ((OTP_MID_INFO_ENDADDR >> 8) & 0xff));
        ov5670_otp_i2c_write(0x3d8B, (OTP_MID_INFO_ENDADDR & 0xff));
        // read otp into buffer
        ov5670_otp_i2c_write(0x3d81, 0x01);
        mdelay(10);
        flag = ov5670_otp_i2c_read(0x7010);
        printk("%s :flag %x\n",__func__ ,flag);
        if(0x40 == (flag & 0xC0)){ // group 1
        	mid_address =  0x7011 ;
        }else if(0x10 == (flag & 0x30)){ // group 2
            mid_address =  0x7018 ;
        }else if(0x04 == (flag & 0x0c)){ // group 3
        	mid_address =  0x701f ;
        }else{
            pr_err("%s, module info flag error:%d\n", __FUNCTION__, flag);
        }       
        
        ov5670_mid = ov5670_otp_i2c_read(mid_address);

        // clear otp buffer
        for (i = OTP_MID_INFO_STARTADDR; i <= OTP_MID_INFO_ENDADDR; i++){
        	ov5670_otp_i2c_write(i, 0x00);
        }
            
        // set 0x5002[3] to '1'
        temp = ov5670_otp_i2c_read(0x5002);
        ov5670_otp_i2c_write(0x5002, 0x08 | (temp | 0x08));
        // stop stream
        ov5670_otp_i2c_write(0x0100, 0x00);
    }
    
	  mutex_unlock(s_ctrl->msm_sensor_mutex);
    printk("alphaxiang %s MID :%x\n",__func__ ,ov5670_mid);
	  return ov5670_mid;
}
#endif
//end