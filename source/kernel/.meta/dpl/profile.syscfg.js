let common = system.getScript("/common");

function getInstanceConfig(moduleInstance) {

    return {
        ...moduleInstance,
    };
};

let profile_intr_module = {
    displayName: "PROFILE",

    templates: {
        "/kernel/dpl/dpl_config.c.xdt": {
            dpl_config: "/kernel/dpl/profile.c.xdt",
            dpl_init: "/kernel/dpl/profile_init.c.xdt",
            dpl_deinit: "/kernel/dpl/profile_deinit.c.xdt",
        },
        "/kernel/dpl/dpl_config.h.xdt": "/kernel/dpl/profile.h.xdt",
    },

    defaultInstanceName: "CONFIG_PROFILE_INTR",
    maxInstances: 1,

    config: [
        {
            name: "profEnable",
            displayName: "Interrupts",
            default: true,
            description: `Enable Interrupts Profiling`,
        },
    ]
};

exports = profile_intr_module;

