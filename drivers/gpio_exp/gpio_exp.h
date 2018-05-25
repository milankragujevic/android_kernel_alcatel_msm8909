#ifndef __GPIO_EXP_H__
#define __GPIO_EXP_H__

#define GPIO_EXP_DEV_NAME	"pcal6416a"

#define INPUT_PORT0_REG		0x00
#define INPUT_PORT1_REG		0x01
#define OUTPUT_PORT0_REG	0x02
#define OUTPUT_PORT1_REG	0x03
#define POLARITY_INVERSION_PORT0_REG	0x04
#define POLARITY_INVERSION_PORT1_REG	0x05
#define CONFIGURATION_PORT0_REG		0x06
#define CONFIGURATION_PORT1_REG		0x07

/**
 * struct gpio_exp_platform_data - data to set up gpio_exp driver
 * @gpio_base: number of the chip's first GPIO
 * @n_latch: optional bit-inverse of initial register value; if
 *           you leave this initialized to zero the driver will act
 *           like the chip was just reset
 * @setup: optional callback issued once the GPIOs are valid
 * @teardown: optional callback issued before the GPIOs are invalidated
 * @context: optional parameter passed to setup() and teardown()
 */
struct gpio_exp_platform_data {
	unsigned	gpio_base;
	u8		n_latch;
	int 		(*setup)(struct i2c_client *client,
					int gpio, unsigned ngpio,
					void *context);
	int		(*teardown)(struct i2c_client *client,
					int gpio, unsigned ngpio,
					void *context);
	void		*context;
	unsigned int	reset_gpio;
};

#endif /* __GPIO_EXP_H__ */
