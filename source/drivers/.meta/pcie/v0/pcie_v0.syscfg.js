
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
    let operMode = getOperMode(instance);

    if (operMode == "PCIE_EP_MODE"){
        modInstances.push({
            name: "ibAtu",
            displayName: "Inbound Address Translation",
            moduleName: "/drivers/pcie/v0/pcieIbAtu",
            useArray: true,
            minInstanceCount: 0,
            defaultInstanceCount: 0,
            requiredArgs: {
                tlpType: "PCIE_TLP_TYPE_MEM",
                windowSize: 0x00,
            },
        });
    }
    else if (operMode == "PCIE_RC_MODE"){
        modInstances.push({
            name: "ibAtu",
            displayName: "Inbound Address Translation",
            moduleName: "/drivers/pcie/v0/pcieIbAtu",
            useArray: true,
            minInstanceCount: 0,
            defaultInstanceCount: 0,
        });
    }

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
            name: "vendorId",
            displayName: "Vendor ID",
            default: "0x17CD",
            description: "Vendor ID of the PCIe Device",
        },
        {
            name: "deviceId",
            displayName: "Device ID",
            default: "0xeeee",
            description: "Device ID of the PCIe Device",
        },
        {
            name: "subSysVendorId",
            displayName: "Subsystem Vendor ID",
            default: "0x17CD",
            description: "Subsystem Vendor ID of the PCIe Device",
        },
        {
            name: "subSystemId",
            displayName: "Subsystem ID",
            default: "0x0000",
            description: "Subsystem ID of the PCIe Device",
        },
        {
            name: "classCode",
            displayName: "Class Code",
            default: "0",
            description: "Bass Class Code",
        },
        {
            name: "subClassCode",
            displayName: "Sub-Class Code",
            default: "0",
            description: "Base Sub-Class Code",
        },
        {
            name: "progIntrface",
            displayName: "Programming Interface",
            default: "0",
            description: "Programming Interface",
        },
        {
            name: "revId",
            displayName: "Revision ID",
            default: "0",
            description: "Revision ID",
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
        },
        {
            name: "refClkMode",
            displayName: "Reference Clock Mode",
            default: "PCIE_REFCLK_MODE_EXT_NOSSC",
            description: "Reference clock modes per clock source, SSC functionality and clock output",
            options: [
                {
                    name: "PCIE_REFCLK_MODE_INT_NOSSC_OUTDIS",
                    displayName: "Internal Reference Clock, no SSC, Output disabled",
                    description: "Internal Reference Clock with no SSC and Output disabled",
                },
                {
                    name: "PCIE_REFCLK_MODE_INT_SSC_OUTDIS",
                    displayName: "Internal Reference Clock, with SSC, Output disabled",
                    description: "Internal Reference Clock with SSC functionality and Output disabled",
                },
                {
                    name: "PCIE_REFCLK_MODE_INT_NOSSC_OUTEN",
                    displayName: "Internal Reference Clock, no SSC, Output enabled",
                    description: "Internal Reference Clock with no SSC and Output enabled",
                },
                {
                    name: "PCIE_REFCLK_MODE_INT_SSC_OUTEN",
                    displayName: "Internal Reference Clock, with SSC, Output enabled",
                    description: "Internal Reference Clock with SSC functionality and Output enabled",
                },
                {
                    name: "PCIE_REFCLK_MODE_EXT_NOSSC",
                    displayName: "External Reference Clock, no SSC",
                    description: "External Reference Clock and no SSC",
                },
                {
                    name: "PCIE_REFCLK_MODE_EXT_SSC",
                    displayName: "External Reference Clock with SSC",
                    description: "External Reference Clock with SSC functionality",
                }
            ],
        },
        {
            name: "srisMode",
            displayName: "SRIS Configuration",
            default: "PCIE_REFCLK_SRIS_DISABLED",
            description: "Separate Reference Clock with Independet Spread (SRIS) Configuration",
            options: [
                {
                    name: "PCIE_REFCLK_SRIS_DISABLED",
                    displayName: "Disable",
                    description: "Disable SRIS",
                },
                {
                    name: "PCIE_REFCLK_SRIS_ENABLED",
                    displayName: "Enable",
                    description: "Enable SRIS",
                }
            ],
        },
        {
            name: "intPin",
            displayName: "Legacy Interrupt Pin",
            default: "PCIE_INT_PINNONE",
            description: "Legacy Interrupt Pin. Select Pin None to disable Legacy Interrupt",
            options: [
                {
                    name: "PCIE_INT_PINNONE",
                    displayName: "Pin None",
                },
                {
                    name: "PCIE_INT_PINA",
                    displayName: "Pin A",
                },
                {
                    name: "PCIE_INT_PINB",
                    displayName: "Pin B",
                },
                {
                    name: "PCIE_INT_PINC",
                    displayName: "Pin C",
                },
                {
                    name: "PCIE_INT_PIND",
                    displayName: "Pin D",
                },
            ],
        },
        {
            name: "msiMmc",
            displayName: "Number of MSI Vectors",
            default: "1",
            description: "Number of distinct MSI vectors that the EP is capable to generate",
            options: [
                {
                    name: "1",
                    displayName: "1",
                },
                {
                    name: "2",
                    displayName: "2",
                },
                {
                    name: "4",
                    displayName: "4",
                },
                {
                    name: "8",
                    displayName: "8",
                },
                {
                    name: "16",
                    displayName: "16",
                },
                {
                    name: "32",
                    displayName: "32",
                }
            ],
        },
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

    common.validate.checkValidIdName(inst, report, "vendorId", inst.vendorId, "Vendor ID");
    common.validate.checkValidIdName(inst, report, "deviceId", inst.deviceId, "Device ID");
    common.validate.checkValidIdName(inst, report, "subSysVendorId", inst.subSysVendorId, "Subsystem Vendor ID");
    common.validate.checkValidIdName(inst, report, "subSystemId", inst.subSystemId, "Subsystem ID");

    common.validate.checkNumberRange(inst, report, "classCode",    0, 255, "dec");
    common.validate.checkNumberRange(inst, report, "subClassCode", 0, 255, "dec");
    common.validate.checkNumberRange(inst, report, "progIntrface", 0, 255, "dec");
    common.validate.checkNumberRange(inst, report, "revId",        0, 255, "dec");

    /* Check Pcie MSI multi message capability */
    let operMode = getOperMode(inst);
    let msiMmc = getMsiMmc(inst);
    if (operMode != "PCIE_EP_MODE" && msiMmc > 1)
    {
        report.logError("The number of MSI vectors is only configurable in EP mode", inst, "gen");
    }
}

function getInterfaceName(inst) {
	return inst.deviceNum;
}

function getOperMode(inst) {
	return inst.operMode;
}

function getMsiMmc(inst) {
	return inst.msiMmc;
}

exports = pcie_module;
