exports = {
	config: [
	],
	templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/i2c/templates/i2c_v0_config.c.xdt",
            driver_init: "/drivers/i2c/templates/i2c_v0_init.c.xdt",
            driver_deinit: "/drivers/i2c/templates/i2c_v0_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/i2c/templates/i2c_v0.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/i2c/templates/i2c_v0_open_close_config.c.xdt",
            driver_open: "/drivers/i2c/templates/i2c_v0_open.c.xdt",
            driver_close: "/drivers/i2c/templates/i2c_v0_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/i2c/templates/i2c_v0_open_close.h.xdt",
        },
	}
};