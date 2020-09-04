
let common = system.getScript("/common");

let cache_c6x_module = {
    displayName: "Cache C66x",
    longDescription: "Refer to C66x DSP Cache Manual for more details",
    templates: {
        "/kernel/dpl/dpl_config.c.xdt": {
            dpl_config: "/kernel/dpl/cache_c6x.c.xdt",
            dpl_init: "/kernel/dpl/cache_c6x_init.c.xdt",
        },
        "/kernel/dpl/dpl_config.h.xdt": "/kernel/dpl/cache_c6x.h.xdt",
    },
    moduleStatic: {
        moduleInstances: function (inst) {
            let modInstances = new Array();
            modInstances.push({
                name: "marRegionConfig",
                displayName: "MAR Configuration",
                moduleName: '/kernel/dpl/cache_c6x_mar',
                useArray: true,
                minInstanceCount: 0,
                maxInstanceCount: 240,
                defaultInstanceCount: 0,
            });
            return (modInstances);
        },
        config: [
            {
                name: "l1psize",
                displayName: "L1P Cache Size",
                default: 32,
                options: [
                    {
                        name: 0,
                        displayName: "0 KB"
                    },
                    {
                        name: 4,
                        displayName: "4 KB"
                    },
                    {
                        name: 8,
                        displayName: "8 KB"
                    },
                    {
                        name: 16,
                        displayName: "16 KB"
                    },
                    {
                        name: 32,
                        displayName: "32 KB"
                    },
                ],
            },
            {
                name: "l1dsize",
                displayName: "L1D Cache Size",
                default: 32,
                options: [
                    {
                        name: 0,
                        displayName: "0 KB"
                    },
                    {
                        name: 4,
                        displayName: "4 KB"
                    },
                    {
                        name: 8,
                        displayName: "8 KB"
                    },
                    {
                        name: 16,
                        displayName: "16 KB"
                    },
                    {
                        name: 32,
                        displayName: "32 KB"
                    },
                ],
            },
            {
                name: "l2size",
                displayName: "L2 Cache Size",
                default: 0,
                options: [
                    {
                        name: 0,
                        displayName: "0 KB"
                    },
                    {
                        name: 32,
                        displayName: "32 KB"
                    },
                    {
                        name: 64,
                        displayName: "64 KB"
                    },
                    {
                        name: 128,
                        displayName: "128 KB"
                    },
                    {
                        name: 256,
                        displayName: "256 KB"
                    },
                    {
                        name: 512,
                        displayName: "512 KB"
                    },
                    {
                        name: 1024,
                        displayName: "1024 KB"
                    },
                ],
            },
        ],
        validate: validate,
    },
};

function validate(instance, report) {
    let cache_config = system.getScript(`../soc/dpl_${common.getSocName()}`).getConfigArr();

    if(instance.l1psize > cache_config.max_l1psize) {
        report.logError(`This SOC supports only up to ${cache_config.max_l1psize} KB L1P Cache!! `, instance, "l1psize");
    }
    if(instance.l1dsize > cache_config.max_l1dsize) {
        report.logError(`This SOC supports only up to ${cache_config.max_l1dsize} KB L1D Cache!! `, instance, "l1dsize");
    }
    if(instance.l2size > cache_config.max_l2size) {
        report.logError(`This SOC supports only up to ${cache_config.max_l2size} KB L2 Cache!! `, instance, "l2size");
    }
}

exports = cache_c6x_module;
