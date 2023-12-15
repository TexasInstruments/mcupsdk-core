exports = {
	config: [
	],
	templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/uart/templates/uart_config_v0_lld.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/uart/templates/uart_lld.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/uart/templates/uart_open_close_config_v0_lld.c.xdt",
            driver_open: "/drivers/uart/templates/uart_open_lld.c.xdt",
            driver_close: "/drivers/uart/templates/uart_close_lld.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/uart/templates/uart_open_close_v0_lld.h.xdt",
        },
	}
};