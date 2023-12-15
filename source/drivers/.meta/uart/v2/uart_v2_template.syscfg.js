exports = {
	config: [
	],
	templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/uart/templates/uart_config_v2.c.xdt",
            driver_init: "/drivers/uart/templates/uart_init.c.xdt",
            driver_deinit: "/drivers/uart/templates/uart_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/uart/templates/uart.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/uart/templates/uart_open_close_config_v2.c.xdt",
            driver_open: "/drivers/uart/templates/uart_open.c.xdt",
            driver_close: "/drivers/uart/templates/uart_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/uart/templates/uart_open_close.h.xdt",
        },
	}
};