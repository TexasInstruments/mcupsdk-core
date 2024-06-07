let common = system.getScript("/common");

let threadx_module_name = "/eclipse_threadx/threadx/threadx";

let threadx_module = {
	displayName: "ThreadX",
     templates: {
        "/eclipse_threadx/eclipse_threadx/eclipse_threadx_config.h.xdt": {
            config: "/eclipse_threadx/threadx/templates/threadx.h.xdt",
        },
        "/eclipse_threadx/eclipse_threadx/eclipse_threadx_open_close.c.xdt": {
            open_close_config: "/eclipse_threadx/threadx/templates/threadx_open_close_config.c.xdt",
            open: "/eclipse_threadx/threadx/templates/threadx_open.c.xdt",
            close: "/eclipse_threadx/threadx/templates/threadx_close.c.xdt",
        },
        "/eclipse_threadx/eclipse_threadx/eclipse_threadx_open_close.h.xdt": {
            open_close_config: "/eclipse_threadx/threadx/templates/threadx_open_close.h.xdt",
        },
    },
	defaultInstanceName: "CONFIG_THREADX",
};

exports = threadx_module;