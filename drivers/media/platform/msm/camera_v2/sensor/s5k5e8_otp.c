#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"
#include "s5k5e8_otp.h"

//#define S5K5E8_DEBUG
#define S5K5E8_TYPICAL_RG 601
#define S5K5E8_TYPICAL_BG 565
#define S5K5E8_REG_PAGE_START 0x0A04
#define S5K5E8_REG_PAGE_END 0x0A43
#define S5K5E8_REG_AWB_FLAG 0x0A43
#define S5K5E8_REG_AWB_GROUP1_PAGE 0x0E
#define S5K5E8_REG_AWB_GROUP2_PAGE 0x0F
#ifdef S5K5E8_DEBUG
#define S5K5E8_REG_LENC_START_PAGE 0x04
#define S5K5E8_REG_LENC_END_PAGE 0x0A
#define S5K5E8_LENC_NUMBER 400
#endif

typedef struct{
	uint16_t r_gain, g_gain, b_gain;
#ifdef S5K5E8_DEBUG
    uint16_t lenc[S5K5E8_LENC_NUMBER];
    uint16_t lenc_cnt;
#endif
}s5k5e8_otp_data_t;
static s5k5e8_otp_data_t s5k5e8_otp_data = {0};
static struct msm_sensor_ctrl_t *s5k5e8_sctrl = NULL;
static int32_t s5k5e8_otp_i2c_write(uint32_t addr, uint16_t data);
static uint16_t s5k5e8_otp_i2c_read(uint32_t addr);
static uint32_t s5k5e8_otp_update_awb(void);
#ifdef S5K5E8_DEBUG
static uint32_t s5k5e8_otp_update_lenc(void);
#endif

int32_t s5k5e8_otp_config(struct msm_sensor_ctrl_t *s_ctrl)
{
	mutex_lock(s_ctrl->msm_sensor_mutex);
	s5k5e8_sctrl = s_ctrl;
    // awb
    s5k5e8_otp_update_awb();
#ifdef S5K5E8_DEBUG
    s5k5e8_otp_update_lenc();
#endif
	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return 0;
}


static int32_t s5k5e8_otp_i2c_write(uint32_t addr, uint16_t data)
{
	int32_t rc = 0;

	if(NULL != s5k5e8_sctrl){
		rc = s5k5e8_sctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s5k5e8_sctrl->sensor_i2c_client, 
			    addr, data, MSM_CAMERA_I2C_BYTE_DATA);
	}
	
	return rc;
}

static uint16_t s5k5e8_otp_i2c_read(uint32_t addr)
{
	uint16_t data = 0;

	if(NULL != s5k5e8_sctrl){
		s5k5e8_sctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s5k5e8_sctrl->sensor_i2c_client, 
			addr, &data, MSM_CAMERA_I2C_BYTE_DATA);
	}
	
	return data;
}

static uint32_t s5k5e8_otp_update_awb()
{
    uint16_t flag = 0, rg_gain = 0, bg_gain = 0, rg_msb = 0, rg_lsb = 0, bg_msb = 0, bg_lsb = 0,
        typical_rg=0, typical_bg=0;
    uint32_t r_g_gain=0, b_g_gain=0, g_g_gain=0, base_gain=0, r_gain=0, b_gain=0, g_gain=0;

	//start stream
	s5k5e8_otp_i2c_write(0x0100, 0x01);
	if(0 != s5k5e8_otp_data.r_gain ||
		0 != s5k5e8_otp_data.g_gain ||
		0 != s5k5e8_otp_data.b_gain){
		pr_err("%s, use buff, gain(rgb):(%d,%d,%d)\n", __FUNCTION__, s5k5e8_otp_data.r_gain,
			s5k5e8_otp_data.g_gain, s5k5e8_otp_data.b_gain);
		//R gain
        s5k5e8_otp_i2c_write(0x0210, (s5k5e8_otp_data.r_gain >> 8));
        s5k5e8_otp_i2c_write(0x0211, (s5k5e8_otp_data.r_gain & 0xff));
        //GR gain
        s5k5e8_otp_i2c_write(0x020E, (s5k5e8_otp_data.g_gain >> 8));
        s5k5e8_otp_i2c_write(0x020F, (s5k5e8_otp_data.g_gain & 0xff));
        //GB gain
        s5k5e8_otp_i2c_write(0x0214, (s5k5e8_otp_data.g_gain >> 8));
        s5k5e8_otp_i2c_write(0x0215, (s5k5e8_otp_data.g_gain & 0xff));
        //B gain
        s5k5e8_otp_i2c_write(0x0212, (s5k5e8_otp_data.b_gain >> 8));
        s5k5e8_otp_i2c_write(0x0213, (s5k5e8_otp_data.b_gain & 0xff));
	}else{
	    //make initial state
	    s5k5e8_otp_i2c_write(0x0A00, 0x04);
	    //set page of OTP
	    s5k5e8_otp_i2c_write(0x0A02, S5K5E8_REG_AWB_GROUP1_PAGE);
	    //set read mode of NVM controller interface1
	    s5k5e8_otp_i2c_write(0x0A00, 0x01);
	    mdelay(10);
	    flag = s5k5e8_otp_i2c_read(S5K5E8_REG_AWB_FLAG);
	    if(0x01 != flag){
	        pr_err("%s, awb group1 flag error:%d, page1:%d\n", __FUNCTION__, flag, S5K5E8_REG_AWB_GROUP1_PAGE);
            //make initial state
            s5k5e8_otp_i2c_write(0x0A00, 0x04);
            //set page of OTP
            s5k5e8_otp_i2c_write(0x0A02, S5K5E8_REG_AWB_GROUP2_PAGE);
            //set read mode of NVM controller interface1
            s5k5e8_otp_i2c_write(0x0A00, 0x01);
            mdelay(10);
            flag = s5k5e8_otp_i2c_read(S5K5E8_REG_AWB_FLAG);
            if(0x01 != flag){
                pr_err("%s, awb group2 flag error:%d, page2:%d\n", __FUNCTION__, flag, S5K5E8_REG_AWB_GROUP2_PAGE);
            }
	    }
	    if(0x01 == flag){
            rg_msb = s5k5e8_otp_i2c_read(S5K5E8_REG_PAGE_START + 11);
            rg_lsb = s5k5e8_otp_i2c_read(S5K5E8_REG_PAGE_START + 12);
            bg_msb = s5k5e8_otp_i2c_read(S5K5E8_REG_PAGE_START + 13);
            bg_lsb = s5k5e8_otp_i2c_read(S5K5E8_REG_PAGE_START + 14);
	        rg_gain = (rg_msb << 8) | (rg_lsb & 0xFF);
	        bg_gain = (bg_msb << 8) | (bg_lsb & 0xFF);
	        if((0 != rg_gain) && (0 != bg_gain)){
                typical_rg = S5K5E8_TYPICAL_RG;
                typical_bg = S5K5E8_TYPICAL_BG;
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
	            //1x gain = 0x100
	            r_gain = 0x100 * r_g_gain / (base_gain);
				g_gain = 0x100 * g_g_gain / (base_gain);
	            b_gain = 0x100 * b_g_gain / (base_gain);
	            //R gain
                s5k5e8_otp_i2c_write(0x0210, (r_gain >> 8));
                s5k5e8_otp_i2c_write(0x0211, (r_gain & 0xff));
                //GR gain
                s5k5e8_otp_i2c_write(0x020E, (g_gain >> 8));
                s5k5e8_otp_i2c_write(0x020F, (g_gain & 0xff));
                //GB gain
                s5k5e8_otp_i2c_write(0x0214, (g_gain >> 8));
                s5k5e8_otp_i2c_write(0x0215, (g_gain & 0xff));
                //B gain
                s5k5e8_otp_i2c_write(0x0212, (b_gain >> 8));
                s5k5e8_otp_i2c_write(0x0213, (b_gain & 0xff));
				s5k5e8_otp_data.r_gain = r_gain;
				s5k5e8_otp_data.g_gain = g_gain;
				s5k5e8_otp_data.b_gain = b_gain;
	            pr_err("%s, gain(rgb):(%d, %d, %d), typical(rg, bg):(%d, %d)\n",
	                __FUNCTION__, r_gain, g_gain, b_gain, typical_rg, typical_bg);
	        }else{
	            pr_err("%s, wrong value 0, gain(rg, bg):(%d, %d)\n", __FUNCTION__, rg_gain, bg_gain);
	        }
	    }
        //make initial state
        s5k5e8_otp_i2c_write(0x0A00, 0x04);
        //disable NVM controller
        s5k5e8_otp_i2c_write(0x0A00, 0x00);
	}
	//stop stream
	s5k5e8_otp_i2c_write(0x0100, 0x00);

    return 0;
}

#ifdef S5K5E8_DEBUG
static uint32_t s5k5e8_otp_update_lenc()
{
    uint32_t i = 0, j = 0, k = 0;
    uint16_t lenc[S5K5E8_LENC_NUMBER] = {0};
    
	//start stream
	s5k5e8_otp_i2c_write(0x0100, 0x01);
	if(0 != s5k5e8_otp_data.lenc_cnt){
	    pr_err("%s, user buff, lenc number:%d\n", __FUNCTION__, s5k5e8_otp_data.lenc_cnt);
	    for(i = 0; i < s5k5e8_otp_data.lenc_cnt; i++){
	        pr_err("%s, use buff, %d, 0x%x\n", __FUNCTION__, i, s5k5e8_otp_data.lenc[i]);
	    }
	}else{
	    for(i = S5K5E8_REG_LENC_START_PAGE; i <= S5K5E8_REG_LENC_END_PAGE; i++){
            //make initial state
	        s5k5e8_otp_i2c_write(0x0A00, 0x04);
	        //set page of OTP
	        s5k5e8_otp_i2c_write(0x0A02, i);
	        //set read mode of NVM controller interface1
	        s5k5e8_otp_i2c_write(0x0A00, 0x01);
	        mdelay(10);
	        for(j = 0; j <= (S5K5E8_REG_PAGE_END - S5K5E8_REG_PAGE_START); j++){
	            if((S5K5E8_REG_LENC_START_PAGE == i) && (0x0A34 > (S5K5E8_REG_PAGE_START + j))){
	                continue;
	            }
	            if(S5K5E8_LENC_NUMBER <= k){
	                pr_err("%s, lenc number is over limited:%d\n", __FUNCTION__, k);
	                break;
	            }
	            lenc[k] = s5k5e8_otp_i2c_read(S5K5E8_REG_PAGE_START + j);
	            pr_err("%s, read, %d, page:%d, %d, 0x%x:0x%x\n", __FUNCTION__, k, i, j, (S5K5E8_REG_PAGE_START + j), lenc[k]);
                s5k5e8_otp_data.lenc[k] = lenc[k];
	            k++;
	        }
	    }
	    s5k5e8_otp_data.lenc_cnt = k;
	    pr_err("%s, read, lenc number:%d\n", __FUNCTION__, k);
        //make initial state
        s5k5e8_otp_i2c_write(0x0A00, 0x04);
        //disable NVM controller
        s5k5e8_otp_i2c_write(0x0A00, 0x00);
	}
	//stop stream
	s5k5e8_otp_i2c_write(0x0100, 0x00);

    return 0;
}
#endif
