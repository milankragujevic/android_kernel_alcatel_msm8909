/*
 * Definitions for akm0991x compass chip.
 */
#ifndef AKM0991x_H
#define AKM0991x_H

#include <linux/ioctl.h>

/* Device specific constant values */
#define AK0991x_REG_WIA1			0x00
#define AK0991x_REG_WIA2			0x01
#define AK0991x_REG_INFO1			0x02
#define AK0991x_REG_INFO2			0x03
#define AK0991x_REG_ST1				0x10
#define AK0991x_REG_HXL				0x11
#define AK0991x_REG_HXH				0x12
#define AK0991x_REG_HYL				0x13
#define AK0991x_REG_HYH				0x14
#define AK0991x_REG_HZL				0x15
#define AK0991x_REG_HZH				0x16
#define AK0991x_REG_TMPS			0x17
#define AK0991x_REG_ST2				0x18
#define AK0991x_REG_CNTL1			0x30
#define AK0991x_REG_CNTL2			0x31
#define AK0991x_REG_CNTL3			0x32

#define AK0991x_FUSE_ASAX			0x60
#define AK0991x_FUSE_ASAY			0x61
#define AK0991x_FUSE_ASAZ			0x62

#define AK0991x_MODE_SNG_MEASURE	0x01
#define AK0991x_MODE_SELF_TEST		0x10
#define AK0991x_MODE_FUSE_ACCESS	0x1F
#define AK0991x_MODE_POWERDOWN		0x00
#define AK0991x_MODE_CONTINUOUS_10HZ	0x02 /* 10Hz */
#define AK0991x_MODE_CONTINUOUS_20HZ	0x04 /* 20Hz */
#define AK0991x_MODE_CONTINUOUS_50HZ	0x06 /* 50Hz */
#define AK0991x_MODE_CONTINUOUS_100HZ	0x08 /* 100Hz */
#define AK0991x_RESET_DATA			0x01

#define AK0991x_REGS_SIZE		13
#define AK0991x_WIA1_VALUE		0x48


/*** Limit of factory shipment test *******************************************/
#define TLIMIT_TN_REVISION_0991x				""
#define TLIMIT_TN_RST_WIA1_0991x				"RST_WIA1"
#define TLIMIT_LO_RST_WIA1_0991x				0x48
#define TLIMIT_HI_RST_WIA1_0991x				0x48
#define TLIMIT_TN_RST_WIA2_0991x				"RST_WIA2"
#define TLIMIT_LO_RST_WIA2_0991x				0x05
#define TLIMIT_HI_RST_WIA2_0991x				0x05

#define TLIMIT_TN_ASAX_0991x					"ASAX"
#define TLIMIT_LO_ASAX_0991x					1
#define TLIMIT_HI_ASAX_0991x					254
#define TLIMIT_TN_ASAY_0991x					"ASAY"
#define TLIMIT_LO_ASAY_0991x					1
#define TLIMIT_HI_ASAY_0991x					254
#define TLIMIT_TN_ASAZ_0991x					"ASAZ"
#define TLIMIT_LO_ASAZ_0991x					1
#define TLIMIT_HI_ASAZ_0991x					254

#define TLIMIT_TN_SNG_ST1_0991x				"SNG_ST1"
#define TLIMIT_LO_SNG_ST1_0991x				1
#define TLIMIT_HI_SNG_ST1_0991x				1

#define TLIMIT_TN_SNG_HX_0991x				"SNG_HX"
#define TLIMIT_LO_SNG_HX_0991x				-8189
#define TLIMIT_HI_SNG_HX_0991x				8189

#define TLIMIT_TN_SNG_HY_0991x				"SNG_HY"
#define TLIMIT_LO_SNG_HY_0991x				-8189
#define TLIMIT_HI_SNG_HY_0991x				8189

#define TLIMIT_TN_SNG_HZ_0991x				"SNG_HZ"
#define TLIMIT_LO_SNG_HZ_0991x				-8189
#define TLIMIT_HI_SNG_HZ_0991x				8189

#define TLIMIT_TN_SNG_ST2_0991x				"SNG_ST2"
#define TLIMIT_LO_SNG_ST2_0991x				0
#define TLIMIT_HI_SNG_ST2_0991x				0

#define TLIMIT_TN_SLF_ST1_0991x				"SLF_ST1"
#define TLIMIT_LO_SLF_ST1_0991x				1
#define TLIMIT_HI_SLF_ST1_0991x				1

#define TLIMIT_TN_SLF_RVHX_0991x				"SLF_REVSHX"
#define TLIMIT_LO_SLF_RVHX_0991x				-30
#define TLIMIT_HI_SLF_RVHX_0991x				30

#define TLIMIT_TN_SLF_RVHY_0991x				"SLF_REVSHY"
#define TLIMIT_LO_SLF_RVHY_0991x				-30
#define TLIMIT_HI_SLF_RVHY_0991x				30

#define TLIMIT_TN_SLF_RVHZ_0991x				"SLF_REVSHZ"
#define TLIMIT_LO_SLF_RVHZ_0991x				-400
#define TLIMIT_HI_SLF_RVHZ_0991x				-50

#define TLIMIT_TN_SLF_ST2_0991x				"SLF_ST2"
#define TLIMIT_LO_SLF_ST2_0991x				0
#define TLIMIT_HI_SLF_ST2_0991x				0

/* To avoid device dependency, convert to general name */
#define AKM_I2C_NAME			"akm0991x"
#define AKM_MISCDEV_NAME		"akm0991x_dev"
#define AKM_SYSCLS_NAME			"compass"
#define AKM_SYSDEV_NAME			"akm0991x"
#define AKM_REG_MODE			AK0991x_REG_CNTL2
#define AKM_REG_RESET			AK0991x_REG_CNTL3
#define AKM_REG_STATUS			AK0991x_REG_ST1
#define AKM_MEASURE_TIME_US		10000
#define AKM_DRDY_IS_HIGH(x)		((x) & 0x01)
#define AKM_DOR_IS_HIGH(x)		((x) & 0x02)
#define AKM_SENSOR_INFO_SIZE	2
#define AKM_SENSOR_CONF_SIZE	3
#define AKM_SENSOR_DATA_SIZE	9

#define AKM_YPR_DATA_SIZE		16
#define AKM_RWBUF_SIZE			16
#define AKM_REGS_SIZE			AK0991x_REGS_SIZE
#define AKM_REGS_1ST_ADDR		AK0991x_REG_WIA1
#define AKM_FUSE_1ST_ADDR		AK0991x_FUSE_ASAX

#define AKM_MODE_SNG_MEASURE	AK0991x_MODE_SNG_MEASURE
#define AKM_MODE_SELF_TEST		AK0991x_MODE_SELF_TEST
#define AKM_MODE_FUSE_ACCESS	AK0991x_MODE_FUSE_ACCESS
#define AKM_MODE_POWERDOWN		AK0991x_MODE_POWERDOWN
#define AKM_MODE_CONTINUOUS_10HZ	AK0991x_MODE_CONTINUOUS_10HZ
#define AKM_MODE_CONTINUOUS_20HZ	AK0991x_MODE_CONTINUOUS_20HZ
#define AKM_MODE_CONTINUOUS_50HZ	AK0991x_MODE_CONTINUOUS_50HZ
#define AKM_MODE_CONTINUOUS_100HZ	AK0991x_MODE_CONTINUOUS_100HZ
#define AKM_RESET_DATA			AK0991x_RESET_DATA

#define ACC_DATA_FLAG		0
#define MAG_DATA_FLAG		1
#define FUSION_DATA_FLAG	2
#define AKM_NUM_SENSORS		3

#define ACC_DATA_READY		(1<<(ACC_DATA_FLAG))
#define MAG_DATA_READY		(1<<(MAG_DATA_FLAG))
#define FUSION_DATA_READY	(1<<(FUSION_DATA_FLAG))

#define AKMIO				0xA1

/* IOCTLs for AKM library */
#define ECS_IOCTL_READ				_IOWR(AKMIO, 0x01, char)
#define ECS_IOCTL_WRITE				_IOW(AKMIO, 0x02, char)
#define ECS_IOCTL_RESET				_IO(AKMIO, 0x03)
#define ECS_IOCTL_SET_MODE			_IOW(AKMIO, 0x10, char)
#define ECS_IOCTL_SET_YPR			_IOW(AKMIO, 0x11, int[AKM_YPR_DATA_SIZE])
#define ECS_IOCTL_GET_INFO			_IOR(AKMIO, 0x20, unsigned char[AKM_SENSOR_INFO_SIZE])
#define ECS_IOCTL_GET_CONF			_IOR(AKMIO, 0x21, unsigned char[AKM_SENSOR_CONF_SIZE])
#define ECS_IOCTL_GET_DATA			_IOR(AKMIO, 0x22, unsigned char[AKM_SENSOR_DATA_SIZE])
#define ECS_IOCTL_GET_OPEN_STATUS	_IOR(AKMIO, 0x23, int)
#define ECS_IOCTL_GET_CLOSE_STATUS	_IOR(AKMIO, 0x24, int)
#define ECS_IOCTL_GET_DELAY			_IOR(AKMIO, 0x25, long long int)
#define ECS_IOCTL_GET_LAYOUT		_IOR(AKMIO, 0x26, char)
#define ECS_IOCTL_GET_ACCEL			_IOR(AKMIO, 0x30, short[3])

struct akm0991x_platform_data {
	char layout;
	int gpio_DRDY;
	int gpio_RSTN;
};

#define AK0991x_WIA2_VALUE_09911		0x05
#define AK0991x_WIA2_VALUE_09916		0x09

#endif

