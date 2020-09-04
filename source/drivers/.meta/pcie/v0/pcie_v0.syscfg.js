
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/pcie/soc/pcie_${common.getSocName()}`);

let pcie_module_name = "/drivers/pcie/pcie"

let pcie_module = {
    displayName: "PCIE",

    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/pcie/templates/pcie_config.c.xdt",
            driver_init: "/drivers/pcie/templates/pcie_init.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/pcie/templates/pcie.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/pcie/templates/pcie_open_close_config.c.xdt",
            driver_open: "/drivers/pcie/templates/pcie_open.c.xdt",
            driver_close: "/drivers/pcie/templates/pcie_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/pcie/templates/pcie_open_close.h.xdt",
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: pcie_module_name,
        },
    },
    defaultInstanceName: "CONFIG_PCIE",
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
    maxInstances: soc.getMaxInstances(),
    moduleInstances: atuCfg,
    getPcieSpeed,
    getInstanceConfig,
    getClockEnableIds,
    getInterfaceName
}

function getInstanceConfig(moduleInstance) {
    return {
        ...moduleInstance,
    };
};

function getClockEnableIds(inst) {

    let clocks = [];
    clocks = soc.getClockIds(inst.deviceNum);

    return clocks;
}

function atuCfg(instance) {
    let modInstances = new Array();

    modInstances.push({
        name: "ibAtu",
        displayName: "Inbound Address Translation",
        moduleName: "/drivers/pcie/v0/pcieIbAtu",
        useArray: true,
        minInstanceCount: 0,
        defaultInstanceCount: 0,
    });

    modInstances.push({
        name: "obAtu",
        displayName: "Outbound Address Translation",
        moduleName: "/drivers/pcie/v0/pcieObAtu",
        useArray: true,
        minInstanceCount: 0,
        defaultInstanceCount: 0,
    });

    return modInstances;
}

function getConfigurables()
{
    let config = [];

    config.push(
        {
            name: "deviceNum",
            displayName: "PCIe Instance",
            default: "0",
            options: soc.getPcieInst(),
        },
        {
            name: "operMode",
            displayName: "Operational Mode",
            default: "PCIE_RC_MODE",
            options: [
                {
                    name: "PCIE_RC_MODE",
                    displayName: "RC mode",
                    description: "Root complex mode of operation",
                },
                {
                    name: "PCIE_EP_MODE",
                    displayName: "EP mode",
                    description: "End point mode of operation",
                }
            ],
        },
        {
            name: "gen",
            displayName: "Operation Speed",
            default: "PCIE_GEN1",
            options: soc.getPcieGenArr()
        },
        {
            name: "numLanes",
            displayName: "Number of Lanes",
            default: "1",
            description: "Number of lanes for the PCIe instance",
        }
    )

    return config;
}

function getPcieSpeed(inst) {
    let pcie_gen = [soc.getPcieGenArr(inst.deviceNum)];
    return pcie_gen;
}

function validate(inst, report) {
    common.validate.checkNumberRange(inst, report, "numLanes", 1, soc.getMaxLanes(inst.deviceNum), "dec");

    /* Check if Pcie speed is valid */
    let pcieGen = soc.getPcieInstGen(inst.deviceNum);
    if (!pcieGen.includes(inst.gen))
    {
        report.logError("Invalid operation speed for this instance!", inst, "gen");
    }
}

function getInterfaceName(inst) {
	return inst.deviceNum;
}

exports = pcie_module;
