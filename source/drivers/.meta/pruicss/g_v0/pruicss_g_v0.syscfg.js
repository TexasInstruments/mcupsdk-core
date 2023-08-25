
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/pruicss/soc/pruicss_${common.getSocName()}`);

function getConfigArr() {
    return soc.getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let configArr = getConfigArr();
    let config = configArr.find(o => o.name === moduleInstance.instance);

    /* make IEP clock == core clock */
    config.clockFrequencies[0].clkRate = moduleInstance.coreClk;
    config.clockFrequencies[2].clkRate = moduleInstance.iepClk;

    return {
        ...config,
        ...moduleInstance,
    };
};

function getMdioBaseAddr(pruicssInstance)
{
    let configArr = getConfigArr();
    let config = configArr.find(o => o.name === pruicssInstance);

    return config.mdioBaseAddr;
}

function getClockEnableIds(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockIds;
}

function getClockFrequencies(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockFrequencies;
}

let pruicss_top_module_name = "/drivers/pruicss/pruicss";

let pruicss_top_module = {
    displayName: "PRU (ICSS)",

    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/pruicss/templates/pruicss_config.c.xdt",
            driver_init: "/drivers/pruicss/templates/pruicss_init.c.xdt",
            driver_deinit: "/drivers/pruicss/templates/pruicss_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/pruicss/templates/pruicss.h.xdt",
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: pruicss_top_module_name,
        },
    },

    defaultInstanceName: "CONFIG_PRU_ICSS",
    config: [
        {
            name: "instance",
            displayName: "Instance",
            default: "ICSSG0",
            options: [
                {
                    name: "ICSSG0",
                },
                {
                    name: "ICSSG1",
                }
            ],
        },
        {
            name: "coreClk",
            displayName: "Core Clk (Hz)",
            default: 200*1000000,
            options: [
                {
                    name: 200*1000000,
                },
                {
                    name: 225*1000000,
                },
                {
                    name: 250*1000000,
                },
                {
                    name: 300*1000000,
                },
                {
                    name: 333333333,
                },
            ],
            onChange: (inst, ui) => {
                if(inst.iepSyncMode)
                    inst.iepClk = inst.coreClk;
            },
        },
        {
            name: "iepSyncMode",
            displayName: "IEP Clk Sync Mode",
            longDescription: "In this mode the async IEP bridge is bypassed and the source of IEP CLK is ICSSGn_CORE_CLK. This means all PRU-ICSSG IOs which use internal IEP clock will use internal core clock.",
            default: false,
            onChange: (inst, ui) => {
                ui.iepClk.readOnly = inst.iepSyncMode;
                if(inst.iepSyncMode)
                    inst.iepClk = inst.coreClk;
            },
        },
        {
            name: "iepClk",
            displayName: "IEP Clk (Hz)",
            default: 200*1000000,
            options: [
                {
                    name: 200*1000000,
                },
                {
                    name: 225*1000000,
                },
                {
                    name: 250*1000000,
                },
                {
                    name: 300*1000000,
                },
                {
                    name: 333333333,
                },
                {
                    name: 500*1000000,
                },
            ],
        },
        {
            name: "EDLoadSharingMode",
            displayName: "EnDat Interface Load Sharing Mode",
            longDescription: "In this mode the RTU_PRU and TX_PRU cores can access and control the EnDat HW. RTU_PRU owns ED Ch-0, PRU owns ED Ch-1, TX_PRU owns ED Ch-2",
            default: "Disabled",
            options: [
                {
                    name: "Disabled",
                    description: "Load Sharing disabled",
                },
                {
                    name: "Slice0",
                    description: "Enable Load Sharing for Slice 0",
                },
                {
                    name: "Slice1",
                    description: "Enable Load Sharing for Slice 1",
                },
                {
                    name: "Slice0 & Slice1",
                    description: "Enable Load Sharing for both Slice0 and Slice 1",
                },
            ],
            hidden: true,
        },
        {
            name: "SDLoadSharingMode",
            displayName: "Sigma-Delta Interface Load Sharing Mode",
            longDescription: "In this mode the RTU_PRU and TX_PRU cores can access and control the Sigma-Delta HW. RTU_PRU owns SD0-SD2, PRU owns SD3-SD5, TX_PRU owns SD6-SD8",
            default: "Disabled",
            options: [
                {
                    name: "Disabled",
                    description: "Load Sharing disabled",
                },
                {
                    name: "Slice0",
                    description: "Enable Load Sharing for Slice 0",
                },
                {
                    name: "Slice1",
                    description: "Enable Load Sharing for Slice 1",
                },
                {
                    name: "Slice0 & Slice1",
                    description: "Enable Load Sharing for both Slice0 and Slice 1",
                },
            ],
            hidden: true,
        },
    ],
    validate: validate,
    moduleInstances: moduleInstances,
    getInstanceConfig,
    getClockFrequencies,
    getClockEnableIds,
    getMdioBaseAddr,
};

function validate(inst, report) {
    common.validate.checkSameInstanceName(inst, report);
}

function moduleInstances(instance) {
    let device = common.getDeviceName();
    let modInstances = new Array();
    if((device === "am64x-evm") || (device === "am243x-evm") || (device === "am243x-lp"))
    {
         modInstances.push({
             name: "AdditionalICSSSettings",
             displayName: "Additional ICSS Settings",
             moduleName: '/drivers/pruicss/g_v0/pruicss_g_v0_gpio',
             useArray: true,
             minInstanceCount: 1,
             defaultInstanceCount: 1,
             maxInstanceCount: 1,
         });
    }

    if((device === "am64x-evm") || (device === "am243x-evm") || (device === "am243x-lp"))
    {
        // Interrupt Mapping:
        let submodule = "/drivers/pruicss/icss_intc/";
        if(instance.instance === "ICSSG0")
            submodule += "icss0_intc_mapping";
        else if(instance.instance === "ICSSG1")
            submodule += "icss1_intc_mapping";
        else
            submodule += "icss0_intc_mapping";
        modInstances.push({
            name: "intcMapping",
            displayName: instance.instance + " INTC Internal Signals Mapping",
            moduleName: submodule,
            useArray: true,
            defaultInstanceCount: 0,
        });
    }
    return (modInstances);
}
exports = pruicss_top_module;
