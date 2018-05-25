#ifndef OV8865_OTP_H
#define OV8865_OTP_H

#define OV8865_MID_SUNRISE 0x0C
#define OV8865_MID_QTECH 0x06

int32_t ov8865_otp_config(struct msm_sensor_ctrl_t *s_ctrl);
uint16_t ov8865_otp_get_mid(struct msm_sensor_ctrl_t *s_ctrl);

#endif //OV8865_OTP_H
