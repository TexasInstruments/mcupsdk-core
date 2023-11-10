exports = {
	config: [
	],
	templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/mcspi/templates/mcspi_v1_config.c.xdt",
            driver_init: "/drivers/mcspi/templates/mcspi_init.c.xdt",
            driver_deinit: "/drivers/mcspi/templates/mcspi_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/mcspi/templates/mcspi.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/mcspi/templates/mcspi_v1_open_close_config.c.xdt",
            driver_open: "/drivers/mcspi/templates/mcspi_open.c.xdt",
            driver_close: "/drivers/mcspi/templates/mcspi_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/mcspi/templates/mcspi_v1_open_close.h.xdt",
        },
	}
};