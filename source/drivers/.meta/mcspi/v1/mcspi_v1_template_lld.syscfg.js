exports = {
	config: [
	],
	templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/mcspi/templates/mcspi_v1_config_lld.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/mcspi/templates/mcspi_lld.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/mcspi/templates/mcspi_v1_open_close_config_lld.c.xdt",
            driver_open: "/drivers/mcspi/templates/mcspi_open_lld.c.xdt",
            driver_close: "/drivers/mcspi/templates/mcspi_close_lld.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/mcspi/templates/mcspi_v1_open_close_lld.h.xdt",
        },
	}
};