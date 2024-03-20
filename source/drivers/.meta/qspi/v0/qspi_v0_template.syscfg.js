exports = {
	config: [
	],
	templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/qspi/templates/qspi_config.c.xdt",
            driver_init: "/drivers/qspi/templates/qspi_init.c.xdt",
            driver_deinit: "/drivers/qspi/templates/qspi_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/qspi/templates/qspi.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/qspi/templates/qspi_open_close_config.c.xdt",
            driver_open: "/drivers/qspi/templates/qspi_open.c.xdt",
            driver_close: "/drivers/qspi/templates/qspi_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/qspi/templates/qspi_open_close.h.xdt",
        },
	}
};