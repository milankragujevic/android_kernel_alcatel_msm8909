#ifndef OV5670_OTP_H
#define OV5670_OTP_H

//SZ xuejian.zhong@tcl.com add 2016.03.07
#if defined(JRD_PROJECT_PIXI464G) || defined(JRD_PROJECT_PIXI464GCRICKET)
#define OV5670_MID_EWELLY 0x44
#define OV5670_MID_QTECH 0x06
#endif
//end

int32_t ov5670_otp_config(struct msm_sensor_ctrl_t *s_ctrl);
//SZ xuejian.zhong@tcl.com add 2016.03.07
#if defined(JRD_PROJECT_PIXI464G) || defined(JRD_PROJECT_PIXI464GCRICKET)
uint16_t ov5670_otp_get_mid(struct msm_sensor_ctrl_t *s_ctrl);
#endif
//end
#endif //OV5670_OTP_H
