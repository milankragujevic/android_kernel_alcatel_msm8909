#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"
#include "hi553_kingcome_otp.h"

//#define HI553_DEBUG 1
#define HI553_TYPICAL_RG 351  //0x15f
#define HI553_TYPICAL_BG 280  //0x118

#define HI553_REG_AWB_FLAG 0x535
#define HI553_REG_AWB_LEN_CNT 29
#ifdef  HI553_DEBUG
#define CDBG(fmt,args...) pr_err(fmt,##args)
#else
#define CDBG(fmt,args...) pr_debug(fmt,##args)
#endif

typedef struct{
    uint16_t r_gain, g_gain, b_gain;
	uint32_t data[91];
}hi553_otp_data_t;
static hi553_otp_data_t hi553_otp_data = {0};
static struct msm_sensor_ctrl_t *hi553_sctrl = NULL;
static int32_t hi553_otp_i2c_write(uint32_t addr, uint16_t data);
static uint16_t hi553_otp_i2c_read(uint32_t addr);
static uint32_t hi553_otp_update_awb(void);
static uint16_t single_read_mode(uint32_t addr);
static uint16_t single_write_with_verify_mode(uint32_t addr,uint32_t value);
static void continue_read_mode(uint32_t addr,uint32_t cnt);



int32_t hi553_otp_config(struct msm_sensor_ctrl_t *s_ctrl)
{
    mutex_lock(s_ctrl->msm_sensor_mutex);
    hi553_sctrl = s_ctrl;
    // awb
    hi553_otp_update_awb();
    mutex_unlock(s_ctrl->msm_sensor_mutex);

    return 0;
}


static int32_t hi553_otp_i2c_write(uint32_t addr, uint16_t data)
{
	int32_t rc = 0;

	if(NULL != hi553_sctrl){
		rc = hi553_sctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(hi553_sctrl->sensor_i2c_client, 
			    addr, data, MSM_CAMERA_I2C_BYTE_DATA);
	}
	
	return rc;
}

static uint16_t hi553_otp_i2c_read(uint32_t addr)
{
	uint16_t data = 0;

	if(NULL != hi553_sctrl){
		hi553_sctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(hi553_sctrl->sensor_i2c_client, 
			addr, &data, MSM_CAMERA_I2C_BYTE_DATA);
	}
	
	return data;
}

static uint32_t hi553_otp_update_awb()
{
    uint16_t flag = 0, rg_gain = 0, bg_gain = 0, rg_msb = 0, rg_lsb = 0, bg_msb = 0, bg_lsb = 0,
        typical_rg=0, typical_bg=0;
    uint32_t r_g_gain=0, b_g_gain=0, g_g_gain=0, base_gain=0, r_gain=0, b_gain=0, g_gain=0;
    uint32_t addr = 0, checksum = 0, sum = 0, i,rc;

	CDBG("%s entry",__FUNCTION__);
	//start stream
	hi553_otp_i2c_write(0x0100, 0x01);

	if(0 != hi553_otp_data.r_gain ||
		0 != hi553_otp_data.g_gain ||
		0 != hi553_otp_data.b_gain){
		pr_err("%s, use buff, gain(rgb):(%d,%d,%d)\n", __FUNCTION__, hi553_otp_data.r_gain,
			hi553_otp_data.g_gain, hi553_otp_data.b_gain);
	}else{
	     
         //read reg prepare
         hi553_otp_i2c_write(0x0a02,0x01); //fast sleep on
         hi553_otp_i2c_write(0x0a00, 0x00);//stand by on
         mdelay(100);
         hi553_otp_i2c_write(0x0f02, 0x00);//pll disable
         hi553_otp_i2c_write(0x011a, 0x01);// CP TRIM_H
         hi553_otp_i2c_write(0x011b, 0x09);// IPGM TRIM_H
         hi553_otp_i2c_write(0x0d04, 0x01);// Fsync(OTP busy) Output Enable
         hi553_otp_i2c_write(0x0d00, 0x07);// Fsync(OTP busy) Output Drivability
         hi553_otp_i2c_write(0x003f, 0x10);// OTP R/W mode
         hi553_otp_i2c_write(0x0a00, 0x01); //stand by off
		
	    //mdelay(10);
	    //read awb flag
        flag = single_read_mode(0x501);
	    CDBG("%s: base-flag=%d \n",__FUNCTION__,flag);
		
	    flag = single_read_mode(0x535);
	    CDBG("%s: flag=%d \n",__FUNCTION__,flag);
         
		if(0x01 == flag){//group 1
            addr = 0x536 ; 
		}else if(0x13 == flag ){//group 2
            addr = 0x554;
		}else if(0x37 == flag){//group 3
            addr = 0x572;
		}
		CDBG("%s: flag=%d \n",__FUNCTION__,flag);
        //check sum
        continue_read_mode(addr,HI553_REG_AWB_LEN_CNT);
        for (i= 0; i< HI553_REG_AWB_LEN_CNT;i++)
        {              
			sum += hi553_otp_data.data[i];
			CDBG("%s:1 sum=%d, hi553_otp_data.data[i]=%d addr+i=%x\n",__FUNCTION__,
				         sum,hi553_otp_data.data[i],(addr+i));				
		}
        sum = sum%255 +1;
		checksum = single_read_mode(0x553) ;
        
		CDBG("%s: sum=%d, checksum=%d \n",__FUNCTION__,sum,checksum);
	    if(0 != addr&&(checksum == sum)){
            rg_msb = single_read_mode(addr);
            rg_lsb = single_read_mode(addr + 1);
			
            bg_msb = single_read_mode(addr + 2);
            bg_lsb = single_read_mode(addr + 3);
			
	        rg_gain = (rg_msb << 8) | (rg_lsb & 0xFF);
	        bg_gain = (bg_msb << 8) | (bg_lsb & 0xFF);
			
	        if((0 != rg_gain) && (0 != bg_gain)){
                typical_rg = HI553_TYPICAL_RG;
                typical_bg = HI553_TYPICAL_BG;
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
                rc = single_write_with_verify_mode(0x012a, (r_gain >> 8));
                CDBG("%s:0x012a  rc=%d \n",__FUNCTION__,rc);
				rc = single_write_with_verify_mode(0x012b, (r_gain & 0xff));
				CDBG("%s:0x012b  rc=%d \n",__FUNCTION__,rc);
                //B gain
                rc = single_write_with_verify_mode(0x012c, (b_gain >> 8));
				CDBG("%s:0x012c  rc=%d \n",__FUNCTION__,rc);
                rc = single_write_with_verify_mode(0x012d, (b_gain & 0xff));
				CDBG("%s:0x012d  rc=%d \n",__FUNCTION__,rc);
				hi553_otp_data.r_gain = r_gain;
				hi553_otp_data.g_gain = g_gain;
				hi553_otp_data.b_gain = b_gain;
	            pr_err("%s, gain(rgb):(%d, %d, %d), typical(rg, bg):(%d, %d)\n",
	                __FUNCTION__, r_gain, g_gain, b_gain, typical_rg, typical_bg);
	        }else{
	            pr_err("%s, wrong value 0, gain(rg, bg):(%d, %d)\n", __FUNCTION__, rg_gain, bg_gain);
	        }
	    }
       hi553_otp_i2c_write(0x0a00, 0x00);// stand by on
       mdelay (100);
       hi553_otp_i2c_write(0x003f, 0x00);// display mode
       hi553_otp_i2c_write(0x0a00, 0x01);// stand by off
	}
	//stop stream
	hi553_otp_i2c_write(0x0100, 0x00);

    return 0;
}

static uint16_t single_read_mode(uint32_t addr)
{
    uint16_t data_v = 0;
	hi553_otp_i2c_write(0x10a, (addr>>8)&0xff);// start address H
	hi553_otp_i2c_write(0x10b,  addr&0xff);// start address L
	hi553_otp_i2c_write(0x102, 0x01);// single read
	data_v = hi553_otp_i2c_read(0x108);
    return data_v;
}

static void continue_read_mode(uint32_t addr,uint32_t cnt)
{
    int i =0 ;
	hi553_otp_i2c_write(0x10a, (addr>>8)&0xff);// start address H
	hi553_otp_i2c_write(0x10b,   addr&0xff);// start address L
	hi553_otp_i2c_write(0x102, 0x01);// single read
	for(i =0;i < cnt;i++ ){
	    hi553_otp_data.data[i]= hi553_otp_i2c_read(0x108);
	}	
    //return 	&hi553_otp_data;
}


static uint16_t single_write_with_verify_mode(uint32_t addr,uint32_t value)

{
	     uint16_t temp = value , rc = 0;
		 hi553_otp_i2c_write(0x10a, (addr>>8)&0xff);// start address H
		 hi553_otp_i2c_write(0x10b,  addr&0xff);// start address L
		 hi553_otp_i2c_write(0x102, 0x02);// single read
		 hi553_otp_i2c_write(0x106, temp);// otp datawrite
         mdelay(4);
		 if(temp != hi553_otp_i2c_read(0x010c))//verify
		 {
             rc = -1;
		 }
	return rc;
}


