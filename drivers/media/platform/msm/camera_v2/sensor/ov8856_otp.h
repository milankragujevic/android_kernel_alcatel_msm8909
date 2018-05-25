#ifndef OV8856_OTP_H
#define OV8856_OTP_H

/*[BUGFIX]-Mod-BEGIN by TCTSZ.(gaoxiang.zou@tcl.com), PR1193020 12/21/2015*/

#define OV8856_MID_SUNNY 0x01

/*[BUGFIX]-Mod-END by TCTSZ.(gaoxiang.zou@tcl.com), PR1193020*/

int32_t ov8856_otp_config(struct msm_sensor_ctrl_t *s_ctrl);
uint16_t ov8856_otp_get_mid(struct msm_sensor_ctrl_t *s_ctrl);

#endif //OV8858_OTP_H

