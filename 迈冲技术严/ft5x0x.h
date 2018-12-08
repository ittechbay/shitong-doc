#ifndef __LINUX_I2C_FT5X0X_H
#define __LINUX_I2C_FT5X0X_H


/*
// Example  
#if defined(CONFIG_TOUCHSCREEN_FT5X0X)
static struct ft5x0x_platform_data ft5x0x_data = {
        .gpio_wake = AT91_PIN_PA4,
};
#endif

static struct i2c_board_info __initdata ek_i2c_devices[] = {
#if defined(CONFIG_TOUCHSCREEN_FT5X0X)

        {
                I2C_BOARD_INFO("ft5x0x_ts", 0x38),
                .platform_data = &ft5x0x_data,
				.irq = gpio_to_irq(AT91_PIN_PA5),
        },
#endif
};
*/


/* linux/i2c/ft5x0x.h */

struct ft5x0x_platform_data {
	int gpio_wake;
};

#endif

