#ifndef OV8858_OTP_H
#define OV8858_OTP_H

/*[BUGFIX]-Mod-BEGIN by TCTSZ.(gaoxiang.zou@tcl.com), PR1193020 12/21/2015*/
#if defined(JRD_PROJECT_PIXI464G) || defined(JRD_PROJECT_PIXI464GCRICKET)
#define OV8858_MID_SUNNY 0x01
#endif
/*[BUGFIX]-Mod-END by TCTSZ.(gaoxiang.zou@tcl.com), PR1193020*/

#define OV8858_MID_SUNRISE 0x0C
#define OV8858_MID_QTECH 0x06

int32_t ov8858_otp_config(struct msm_sensor_ctrl_t *s_ctrl);
uint16_t ov8858_otp_get_mid(struct msm_sensor_ctrl_t *s_ctrl);

#endif //OV8858_OTP_H
