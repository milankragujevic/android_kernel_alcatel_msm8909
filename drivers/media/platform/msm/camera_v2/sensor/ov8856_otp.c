#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"
#include "ov8856_otp.h"

#define OV8856_DEBUG
#define OV8856_TYPICAL_RG_SUNNY 308 // 0x134
#define OV8856_TYPICAL_BG_SUNNY 324
#define OV8856_REG_MODULE_FLAG 0x7010
#define OV8856_REG_MODULE_GROUP1_START 0x7011
#define OV8856_REG_MODULE_GROUP1_END 0x7018
#define OV8856_REG_MODULE_GROUP2_START 0x7019
#define OV8856_REG_MODULE_GROUP2_END 0x7020
#define OV8858_REG_LENC_FLAG 0x7028
#define OV8858_REG_LENC_GROUP1_START 0x7029
#define OV8858_REG_LENC_GROUP1_END 0x7118
#define OV8858_REG_LENC_GROUP2_START 0x711A
#define OV8858_REG_LENC_GROUP2_END 0x7209

#define OV8856_NUM_REG_LENC 240

typedef struct{
    uint16_t mid;
    uint16_t lenc[OV8856_NUM_REG_LENC];
    uint16_t lenc_cnt;
    uint32_t r_gain, g_gain, b_gain;
	
}ov8856_otp_data_t;
static ov8856_otp_data_t ov8856_otp_data = {0};
static struct msm_sensor_ctrl_t *ov8856_sctrl = NULL;
static int32_t ov8856_otp_i2c_write(uint32_t addr, uint16_t data);
static uint16_t ov8856_otp_i2c_read(uint32_t addr);
static uint32_t ov8856_otp_update_awb(void);
static uint32_t ov8856_otp_update_lenc(void);

int32_t ov8856_otp_config(struct msm_sensor_ctrl_t *s_ctrl)
{
	mutex_lock(s_ctrl->msm_sensor_mutex);
    ov8856_sctrl = s_ctrl;
    // awb
    ov8856_otp_update_awb();
    // lenc
    ov8856_otp_update_lenc();
	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return 0;
}

uint16_t ov8856_otp_get_mid(struct msm_sensor_ctrl_t *s_ctrl)
{
    uint32_t i = 0;
	uint16_t temp = 0, flag = 0, mid = 0;
    pr_err("%s: entry\n", __FUNCTION__);
	mutex_lock(s_ctrl->msm_sensor_mutex);
    if(0 != ov8856_otp_data.mid){
        pr_err("%s, use buff, mid:%d\n", __FUNCTION__, ov8856_otp_data.mid);
        mid = ov8856_otp_data.mid;
    }else{
        ov8856_sctrl = s_ctrl;
        // start stream
        ov8856_otp_i2c_write(0x0100, 0x01);
        // set 0x5001[3] to '0'
        temp = ov8856_otp_i2c_read(0x5001);
        ov8856_otp_i2c_write(0x5001, (0x00 & 0x08) |(temp & (~0x08)));
		
        //enable partial OTP read mode
        ov8856_otp_i2c_write(0x3d84, 0xC0);
        // partial mode OTP write start address
        ov8856_otp_i2c_write(0x3d88, 0x70);
        ov8856_otp_i2c_write(0x3d89, 0x10);
        // partial mode OTP write end address
        ov8856_otp_i2c_write(0x3d8A, 0x72);
        ov8856_otp_i2c_write(0x3d8B, 0x0a);
        // read otp into buffer
        ov8856_otp_i2c_write(0x3d81, 0x01);
        mdelay(10);
        flag = ov8856_otp_i2c_read(OV8856_REG_MODULE_FLAG);
        if(0x40 == (flag & 0xC0)){ // group 1
            mid = ov8856_otp_i2c_read(OV8856_REG_MODULE_GROUP1_START);
        }else if(0x10 == (flag & 0x30)){ // group 2
            mid = ov8856_otp_i2c_read(OV8856_REG_MODULE_GROUP2_START);
        }else{
            pr_err("%s, module info flag error:%d\n", __FUNCTION__, flag);
        }
        if((0x40 == (flag & 0xC0)) || (0x10 == (flag & 0x30))){
            ov8856_otp_data.mid = mid;
            pr_err("%s, mid:0x%x\n", __FUNCTION__, mid);
        }
        // clear otp buffer
        for (i = OV8856_REG_MODULE_FLAG; i <= OV8856_REG_MODULE_GROUP2_END; i++){
        	ov8856_otp_i2c_write(i, 0x00);
        }
        // set 0x5001[3] to '1'
        temp = ov8856_otp_i2c_read(0x5001);
        ov8856_otp_i2c_write(0x5001, (0x08 & 0x08) | (temp & (~0x08)));
        // stop stream
        ov8856_otp_i2c_write(0x0100, 0x00);
    }
	mutex_unlock(s_ctrl->msm_sensor_mutex);
    pr_err("%s: X\n", __FUNCTION__);
	return mid;
}

static int32_t ov8856_otp_i2c_write(uint32_t addr, uint16_t data)
{
	int32_t rc = 0;

    if(NULL != ov8856_sctrl){
    	rc = ov8856_sctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(ov8856_sctrl->sensor_i2c_client, 
    		    addr, data, MSM_CAMERA_I2C_BYTE_DATA);
    }
    
	return rc;
}

static uint16_t ov8856_otp_i2c_read(uint32_t addr)
{
	uint16_t data = 0;

    if(NULL != ov8856_sctrl){
    	ov8856_sctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(ov8856_sctrl->sensor_i2c_client, 
    		addr, &data, MSM_CAMERA_I2C_BYTE_DATA);
    }
    
	return data;
}

static uint32_t ov8856_otp_update_awb()
{
    uint16_t temp = 0,  rg_gain = 0, bg_gain = 0, rg_msb = 0, bg_msb = 0, lsb = 0, 
             typical_rg = 0, typical_bg = 0, otp_flag = 0,addr = 0;
	uint32_t i = 0, r_g_gain=0, b_g_gain=0, g_g_gain=0, base_gain=0, r_gain=0, b_gain=0, g_gain=0;
    pr_err("%s: entry\n", __FUNCTION__);
    //start stream
    ov8856_otp_i2c_write(0x0100, 0x01);
	//enable partial OTP read mode
	ov8856_otp_i2c_write(0x3d84, 0xC0);
	// partial mode OTP write start address
	ov8856_otp_i2c_write(0x3d88, 0x70);
	ov8856_otp_i2c_write(0x3d89, 0x10);
	// partial mode OTP write end address
	ov8856_otp_i2c_write(0x3d8A, 0x72);
	ov8856_otp_i2c_write(0x3d8B, 0x0a);
	// read otp into buffer
	ov8856_otp_i2c_write(0x3d81, 0x01);
	mdelay(10);

	// OTP base information and WB calibration data
	otp_flag = ov8856_otp_i2c_read(OV8856_REG_MODULE_FLAG);
    if((otp_flag & 0xc0) == 0x40) {
       addr = 0x7011; // base address of info group 1
    }else if((otp_flag & 0x30) == 0x10) {
       addr = 0x7019; // base address of info group 2
    }
	pr_err("%s: addr:%x\n", __FUNCTION__,addr);
    //read  rg_msb bg_msb lsb
    if(0 != addr)
    {
        rg_msb = ov8856_otp_i2c_read(addr + 5);
	    bg_msb = ov8856_otp_i2c_read(addr + 6);
        lsb  = ov8856_otp_i2c_read(addr + 7);
	
	    rg_gain = (rg_msb << 2) | ((lsb >> 6) & 0x03);
        bg_gain = (bg_msb << 2) | ((lsb >> 4) & 0x03);
        //calculate G gain
	    typical_rg = OV8856_TYPICAL_RG_SUNNY;
	    typical_bg = OV8856_TYPICAL_BG_SUNNY;
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
        // update wb gain
        if (r_gain>0x400) {
	        ov8856_otp_i2c_write(0x5019, r_gain>>8);
	        ov8856_otp_i2c_write(0x501a, r_gain & 0x00ff);
	    }
	    if (g_gain>0x400) {
	        ov8856_otp_i2c_write(0x501b, g_gain>>8);
	        ov8856_otp_i2c_write(0x501c, g_gain & 0x00ff);
	    }
	    if (b_gain>0x400) {
	        ov8856_otp_i2c_write(0x501d, b_gain>>8);
	        ov8856_otp_i2c_write(0x501e, b_gain & 0x00ff);
	    }
        ov8856_otp_data.r_gain = r_gain;
        ov8856_otp_data.g_gain = g_gain;
        ov8856_otp_data.b_gain = b_gain;
        pr_err("%s, gain(rgb):(%d, %d, %d), typical(rg, bg):(%d, %d)\n",
           __FUNCTION__, r_gain, g_gain, b_gain, typical_rg, typical_bg);
	}else{
       pr_err("%s,gain value error \n",__FUNCTION__);

	}
		
    //clear buffer
	for (i = OV8856_REG_MODULE_FLAG; i <= OV8856_REG_MODULE_GROUP2_END; i++){
        ov8856_otp_i2c_write(i, 0x00);
    }
    // set 0x5001[3] to '1'
    temp = ov8856_otp_i2c_read(0x5001);
    ov8856_otp_i2c_write(0x5001, (0x08 & 0x08) | (temp & (~0x08)));
    //stop stream
    ov8856_otp_i2c_write(0x0100, 0x00);
    pr_err("%s: X\n", __FUNCTION__);
    return 0;
}

static uint32_t ov8856_otp_update_lenc()
{
    uint16_t temp = 0, flag = 0, addr = 0;
    uint32_t  i = 0, sum_lenc = 0, checksum = 0;
    pr_err("%s: entry\n", __FUNCTION__);
    // start stream
    ov8856_otp_i2c_write(0x0100, 0x01);
    if(0 < ov8856_otp_data.lenc_cnt){
        pr_err("%s, use buff, num:%d\n", __FUNCTION__, ov8856_otp_data.lenc_cnt);
        temp = ov8856_otp_i2c_read(0x5000);
        ov8856_otp_i2c_write(0x5000, (temp | 0x80));
        for(i = 0; i < ov8856_otp_data.lenc_cnt; i++){
            ov8856_otp_i2c_write((0x5900 + i), ov8856_otp_data.lenc[i]);
#ifdef OV8856_DEBUG
            pr_err("%s, buff write, %d, 0x%x:0x%x\n", __FUNCTION__,
                i, (0x5900 + i), ov8856_otp_data.lenc[i]);
#endif
        }
    }else{
		//start stream
		ov8856_otp_i2c_write(0x0100, 0x01);
		//enable partial OTP read mode
		ov8856_otp_i2c_write(0x3d84, 0xC0);
		// partial mode OTP write start address
		ov8856_otp_i2c_write(0x3d88, 0x70);
		ov8856_otp_i2c_write(0x3d89, 0x10);
		// partial mode OTP write end address
		ov8856_otp_i2c_write(0x3d8A, 0x72);
		ov8856_otp_i2c_write(0x3d8B, 0x0a);
		// read otp into buffer
		ov8856_otp_i2c_write(0x3d81, 0x01);
		mdelay(10);

		// OTP Lenc Calibration
		flag = ov8856_otp_i2c_read(OV8858_REG_LENC_FLAG);
		if((flag & 0xc0) == 0x40) {
		    addr = 0x7029; // base address of Lenc Calibration group 1
		}else if((flag & 0x30) == 0x10) {
		    addr = 0x711a; // base address of Lenc Calibration group 2
		}
		if(addr != 0) {
		   for(i=0;i<240;i++) {
		       ov8856_otp_data.lenc[i]=ov8856_otp_i2c_read(addr + i);
		       sum_lenc += ov8856_otp_data.lenc[i];
		   }
		   sum_lenc = (sum_lenc)%255 +1;
		   checksum = ov8856_otp_i2c_read(addr + 240);
		
		   if(sum_lenc == checksum){
           	// apply OTP Lenc Calibration
				temp = ov8856_otp_i2c_read(0x5000);
				temp = 0x20 | temp;
				ov8856_otp_i2c_write(0x5000, temp);
				for(i=0;i<240;i++) {
					 ov8856_otp_i2c_write(0x5900 + i, ov8856_otp_data.lenc[i]);
		#ifdef OV8856_DEBUG
					 pr_err("%s, buff write, %d, 0x%x:0x%x\n", __FUNCTION__,
                             i, (0x5900 + i), ov8856_otp_data.lenc[i]);
		#endif
			    }
			}			
	     }else{
              pr_err("%s, lenc get data error\n", __FUNCTION__);
         
		 }
		
        // clear otp buffer
            for (i = OV8858_REG_LENC_FLAG; i <= OV8858_REG_LENC_GROUP2_END; i++){
                ov8856_otp_i2c_write(i, 0x00);
            }
        // set 0x5001[3] to '1'
        temp = ov8856_otp_i2c_read(0x5001);
        ov8856_otp_i2c_write(0x5001, (temp | 0x08));
    
        //stop stream
        ov8856_otp_i2c_write(0x0100, 0x00);
    }
	pr_err("%s: X\n", __FUNCTION__);
    return 0;
}


