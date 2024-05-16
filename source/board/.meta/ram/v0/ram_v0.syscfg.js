let common = system.getScript("/common");
let soc = system.getScript(`/board/soc/board_${common.getSocName()}`);

function getDriver(drvName) {
    return system.getScript(`/drivers/${drvName}/${drvName}`);
}

function getInstanceConfig(moduleInstance) {

    return {
        ...moduleInstance,
    };
};

let ram_module = {
    displayName: "RAM",
    templates: {
        "/board/board/board_open_close.c.xdt": {
            board_open_close_config: "/board/ram/templates/ram_open_close_config.c.xdt",
            board_open: "/board/ram/templates/ram_open.c.xdt",
            board_close: "/board/ram/templates/ram_close.c.xdt",
        },
        "/board/board/board_open_close.h.xdt": {
            board_open_close_config: "/board/ram/templates/ram_open_close.h.xdt",
        },
        "/board/board/board_config.h.xdt": {
            board_config: "/board/ram/templates/ram.h.xdt",
        },
    },
    defaultInstanceName: "CONFIG_RAM",
    config: getConfigurables(),
    validate: validate,
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    moduleInstances: moduleInstances,
    getInstanceConfig,
};

function  getConfigurables()
{
    let config = [];

    config.push(
        {
            name: "ramTopology",
            displayName: "RAM Topology",
            default: "parallelRam",
            options: [
                { name: "parallelRam", displayName: "Parallel RAM" },
                { name: "serialRam", displayName: "Serial RAM"},
            ],
            getDisabledOptions : (inst) =>
            {
                if(soc.getDriverInstanceValid("serialRam") == false)
                {
                    let disabledOptions = [
                        { name: "serialRam" , displayName: "Serial RAM", reason: "Not supported" },
                    ];
                    return disabledOptions;
                } else if (soc.getDriverInstanceValid("parallelRam") == false)
                {
                    let disabledOptions = [
                        { name: "parallelRam" , displayName: "Parallel RAM", reason: "Not supported"},
                    ];
                    return disabledOptions;
                }
                else
                {
                    return [];
                }
            }
        },
    );

    return config;
}

function validate(inst, report) {

    common.validate.checkSameFieldName(inst, "ramTopology", report);

}

function moduleInstances(inst) {
    let modInstances = new Array();
    let requiredArgs = {};

    if(inst.ramTopology == "serialRam")
    {
        if(soc.getDriverInstanceValid("serialRam") == true){
            modInstances.push({
                name: "serialRamDriver",
                displayName: "Serial RAM Configuration",
                moduleName: "/board/ram/serialRam/serialRam",
                requiredArgs: requiredArgs,
                useArray: false,
            })
        }
    }

    if(inst.ramTopology == "parallelRam")
    {
        if(soc.getDriverInstanceValid("parallelRam") == true){
            modInstances.push({
                name: "parallelRamDriver",
                displayName: "Parallel RAM Configuration",
                moduleName: "/board/ram/parallelRam/parallelram",
                requiredArgs: requiredArgs,
                useArray: false,
            })
        }
    }

    return (modInstances);
}
exports = ram_module;