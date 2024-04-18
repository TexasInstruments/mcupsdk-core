
let common = system.getScript("/common");
let hwi = system.getScript("/kernel/dpl/hwi.js");

function getConfigArr() {
    return system.getScript(`/drivers/edma/soc/edma_${common.getSocName()}`).getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let configArr = getConfigArr();
    let config = configArr.find( o => o.name === moduleInstance.instance);

    return config;
};

function getMaxRegionId(moduleInstance) {

    let instConfig = getInstanceConfig(moduleInstance);

    return instConfig.maxRegions;
}

function getMaxQueue(moduleInstance) {

    let instConfig = getInstanceConfig(moduleInstance);

    return instConfig.maxQueue;
}

function getDefaultRegion() {
    return system.getScript(`/drivers/edma/soc/edma_${common.getSocName()}`).getDefaultRegion();
}

function isReservedChannelSupported() {
    return system.getScript(`/drivers/edma/soc/edma_${common.getSocName()}`).isReservedChannelSupported();
}

function isChannelTriggerXbarSupported() {
    return system.getScript(`/drivers/edma/soc/edma_${common.getSocName()}`).isChannelTriggerXbarSupported();
}

function getRmDefaultStart(moduleInstance, resType) {
    let defOwnDmaChStart = 0;
    let defOwnQdmaChStart = 0;
    let defOwnTccStart = 0;
    let defOwnParamStart = 0;
    let defReservedDmaChStart = 0;
    let selfCoreName = common.getSelfSysCfgCoreName();
    let instConfig = getInstanceConfig(moduleInstance);
    if (selfCoreName === "r5fss0-0") {
        defOwnDmaChStart = instConfig.defaultOwnDmaChannelStart_r5fss0_0;
        defOwnQdmaChStart = instConfig.defaultOwnQdmaChannelStart_r5fss0_0;
        defOwnTccStart = instConfig.defaultOwnTccStart_r5fss0_0;
        defOwnParamStart = instConfig.defaultOwnParamStart_r5fss0_0;
        defReservedDmaChStart = instConfig.defaultReservedDmaChannelStart_r5fss0_0;
    }
    if (selfCoreName === "r5fss0-1") {
        defOwnDmaChStart = instConfig.defaultOwnDmaChannelStart_r5fss0_1;
        defOwnQdmaChStart = instConfig.defaultOwnQdmaChannelStart_r5fss0_1;
        defOwnTccStart = instConfig.defaultOwnTccStart_r5fss0_1;
        defOwnParamStart = instConfig.defaultOwnParamStart_r5fss0_1;
        defReservedDmaChStart = instConfig.defaultReservedDmaChannelStart_r5fss0_1;
    }
    if (selfCoreName === "r5fss1-0") {
        defOwnDmaChStart = instConfig.defaultOwnDmaChannelStart_r5fss1_0;
        defOwnQdmaChStart = instConfig.defaultOwnQdmaChannelStart_r5fss1_0;
        defOwnTccStart = instConfig.defaultOwnTccStart_r5fss1_0;
        defOwnParamStart = instConfig.defaultOwnParamStart_r5fss1_0;
        defReservedDmaChStart = instConfig.defaultReservedDmaChannelStart_r5fss1_0;
    }
    if (selfCoreName === "r5fss1-1") {
        defOwnDmaChStart = instConfig.defaultOwnDmaChannelStart_r5fss1_1;
        defOwnQdmaChStart = instConfig.defaultOwnQdmaChannelStart_r5fss1_1;
        defOwnTccStart = instConfig.defaultOwnTccStart_r5fss1_1;
        defOwnParamStart = instConfig.defaultOwnParamStart_r5fss1_1;
        defReservedDmaChStart = instConfig.defaultReservedDmaChannelStart_r5fss1_1;
    }
    if (selfCoreName === "c66ss0") {
        defOwnDmaChStart = instConfig.defaultOwnDmaChannelStart_c66ss0;
        defOwnQdmaChStart = instConfig.defaultOwnQdmaChannelStart_c66ss0;
        defOwnTccStart = instConfig.defaultOwnTccStart_c66ss0;
        defOwnParamStart = instConfig.defaultOwnParamStart_c66ss0;
        defReservedDmaChStart = instConfig.defaultReservedDmaChannelStart_c66ss0;
    }
    if(resType == "ownDmaCh") {
        return Number(defOwnDmaChStart);
    }
    if(resType == "ownQdmaCh") {
        return Number(defOwnQdmaChStart);
    }
    if(resType == "ownTcc") {
        return Number(defOwnTccStart);
    }
    if(resType == "ownParam") {
        return Number(defOwnParamStart);
    }
    if(resType == "reservedDmaCh") {
        return Number(defReservedDmaChStart);
    }
}

function getRmDefaultEnd(moduleInstance, resType) {
    let defOwnDmaChEnd = 0;
    let defOwnQdmaChEnd = 0;
    let defOwnTccEnd = 0;
    let defOwnParamEnd = 0;
    let defReservedDmaChEnd = 0;
    let selfCoreName = common.getSelfSysCfgCoreName();
    let instConfig = getInstanceConfig(moduleInstance);
    if (selfCoreName === "r5fss0-0") {
        defOwnDmaChEnd = instConfig.defaultOwnDmaChannelEnd_r5fss0_0;
        defOwnQdmaChEnd = instConfig.defaultOwnQdmaChannelEnd_r5fss0_0;
        defOwnTccEnd = instConfig.defaultOwnTccEnd_r5fss0_0;
        defOwnParamEnd = instConfig.defaultOwnParamEnd_r5fss0_0;
        defReservedDmaChEnd = instConfig.defaultReservedDmaChannelEnd_r5fss0_0;
    }
    if (selfCoreName === "r5fss0-1") {
        defOwnDmaChEnd = instConfig.defaultOwnDmaChannelEnd_r5fss0_1;
        defOwnQdmaChEnd = instConfig.defaultOwnQdmaChannelEnd_r5fss0_1;
        defOwnTccEnd = instConfig.defaultOwnTccEnd_r5fss0_1;
        defOwnParamEnd = instConfig.defaultOwnParamEnd_r5fss0_1;
        defReservedDmaChEnd = instConfig.defaultReservedDmaChannelEnd_r5fss0_1;
    }
    if (selfCoreName === "r5fss1-0") {
        defOwnDmaChEnd = instConfig.defaultOwnDmaChannelEnd_r5fss1_0;
        defOwnQdmaChEnd = instConfig.defaultOwnQdmaChannelEnd_r5fss1_0;
        defOwnTccEnd = instConfig.defaultOwnTccEnd_r5fss1_0;
        defOwnParamEnd = instConfig.defaultOwnParamEnd_r5fss1_0;
        defReservedDmaChEnd = instConfig.defaultReservedDmaChannelEnd_r5fss1_0;
    }
    if (selfCoreName === "r5fss1-1") {
        defOwnDmaChEnd = instConfig.defaultOwnDmaChannelEnd_r5fss1_1;
        defOwnQdmaChEnd = instConfig.defaultOwnQdmaChannelEnd_r5fss1_1;
        defOwnTccEnd = instConfig.defaultOwnTccEnd_r5fss1_1;
        defOwnParamEnd = instConfig.defaultOwnParamEnd_r5fss1_1;
        defReservedDmaChEnd = instConfig.defaultReservedDmaChannelEnd_r5fss1_1;
    }
    if (selfCoreName === "c66ss0") {
        defOwnDmaChEnd = instConfig.defaultOwnDmaChannelEnd_c66ss0;
        defOwnQdmaChEnd = instConfig.defaultOwnQdmaChannelEnd_c66ss0;
        defOwnTccEnd = instConfig.defaultOwnTccEnd_c66ss0;
        defOwnParamEnd = instConfig.defaultOwnParamEnd_c66ss0;
        defReservedDmaChEnd = instConfig.defaultReservedDmaChannelEnd_c66ss0;
    }
    if(resType == "ownDmaCh") {
        return Number(defOwnDmaChEnd);
    }
    if(resType == "ownQdmaCh") {
        return Number(defOwnQdmaChEnd);
    }
    if(resType == "ownTcc") {
        return Number(defOwnTccEnd);
    }
    if(resType == "ownParam") {
        return Number(defOwnParamEnd);
    }
    if(resType == "reservedDmaCh") {
        return Number(defReservedDmaChEnd);
    }
}

let edma_module = {
    displayName: "EDMA",
    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/edma/templates/edma_config.c.xdt",
            driver_init: "/drivers/edma/templates/edma_init.c.xdt",
            driver_deinit: "/drivers/edma/templates/edma_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/edma/templates/edma.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/edma/templates/edma_open_close_config.c.xdt",
            driver_open: "/drivers/edma/templates/edma_open.c.xdt",
            driver_close: "/drivers/edma/templates/edma_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/edma/templates/edma_open_close.h.xdt",
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_EDMA",
    config: [
        {
            name: "regionId",
            displayName: "Region Id",
            default: getDefaultRegion(),
            options: [
                { name: 0, displayName: "Region 0"},
                { name: 1, displayName: "Region 1"},
                { name: 2, displayName: "Region 2"},
                { name: 3, displayName: "Region 3"},
                { name: 4, displayName: "Region 4"},
                { name: 5, displayName: "Region 5"},
                { name: 6, displayName: "Region 6"},
                { name: 7, displayName: "Region 7"},
            ],
            description: "EDMA region id. Same region number should not be used by different cores sharing the edma instance",
            displayFormat: "dec",
        },
        {
            name: "queNum",
            displayName: "Default Queue Number",
            default: 0,
            options: [
                { name: 0, displayName: "Queue 0"},
                { name: 1, displayName: "Queue 1"},
                { name: 2, displayName: "Queue 2"},
                { name: 3, displayName: "Queue 3"},
            ],
            description: "Queue number corresponds to the priority of the transfer. Lower the queue number higher the priority of transfer",
            displayFormat: "dec",
        },
        {
            name: "initParamSet",
            displayName: "Initialize Param Memory",
            default: "FALSE",
            options: [
                {
                    name: "FALSE",
                    displayName: "FALSE"
                },
                {
                    name: "TRUE",
                    displayName: "TRUE"
                },
            ],
            description: "This determines whether the Param Memory is initialized",
        },
        {
            name: "intrEnable",
            displayName: "Enable Interrupt",
            default: "TRUE",
            options: [
                {
                    name: "FALSE",
                    displayName: "FALSE"
                },
                {
                    name: "TRUE",
                    displayName: "TRUE"
                },
            ],
            description: "This determines whether the interrupt needs to be registered",
        },
        {
            name: "intrPriority",
            displayName: "Interrupt Priority",
            default: hwi.getHwiDefaultPriority(),
            hidden: !hwi.getPriorityConfigSupported(),
            description: `Interrupt Priority: 0 (highest) to ${hwi.getHwiMaxPriority()} (lowest)`,
        },
        {
            name: "enableOwnDmaChannelConfig",
            displayName: "Enable Core Dma Channel Config",
            default: true,
            description: `Enable Core Dma Channel Config`,
        },
        {
            name: "enableOwnQdmaChannelConfig",
            displayName: "Enable Core Qdma Channel Config",
            default: true,
            description: `Enable Core Qdma Channel Config`,
        },
        {
            name: "enableOwnTccConfig",
            displayName: "Enable Core Tcc Config",
            default: true,
            description: `Enable Core Tcc Config`,
        },
        {
            name: "enableOwnParamConfig",
            displayName: "Enable Core Param Config",
            default: true,
            description: `Enable Core Param Config`,
        },
        {
            name: "enableReservedChannelConfig",
            displayName: "Enable Reserved Dma Channel Config",
            default: isReservedChannelSupported(),
            description: `Enable Reserved Dma Channel Config`,
            hidden: !isReservedChannelSupported(),
        },
        common.ui.makeInstanceConfig(getConfigArr()),
    ],
    moduleInstances: moduleInstances,
    validate: validate,
    moduleStatic: {
        modules: function(instance) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    getInstanceConfig,
};

function validate(instance, report) {
    let maxRegion = getMaxRegionId(instance);
    let maxQueue  = getMaxQueue(instance);

    common.validate.checkNumberRange(instance, report, "intrPriority", 0, hwi.getHwiMaxPriority(), "dec");
    common.validate.checkNumberRange(instance, report, "regionId", 0, Number(maxRegion) - Number(1), "dec");
    common.validate.checkNumberRange(instance, report, "queNum", 0, Number(maxQueue) - Number(1), "dec");
    common.validate.checkSameInstanceName(instance, report);
    remoteCoreValidate(instance, report);
    let instConfig = getInstanceConfig(instance);
    rmValidate(instance, report, "ownDmaCh", "edmaRmDmaCh", instConfig.maxDmaChannels);
    rmValidate(instance, report, "ownQdmaCh", "edmaRmQdmaCh", 7);
    rmValidate(instance, report, "ownTcc", "edmaRmTcc", instConfig.maxDmaChannels);
    rmValidate(instance, report, "ownParam", "edmaRmParam", instConfig.maxPaRAM);
    rmValidate(instance, report, "reservedDmaCh", "edmaRmReservedDmaCh", instConfig.maxDmaChannels);
}

function remoteCoreValidate(instance, report) {
    let coreNames = common.getSysCfgCoreNames();
    let selfCoreName = common.getSelfSysCfgCoreName();
    let edma_module_name = '/drivers/edma/edma';

    for ( let remoteCoreName of coreNames)
    {
        if( remoteCoreName != selfCoreName)
        {
            let remote_core_module = common.getModuleForCore(edma_module_name, remoteCoreName);
            if (remote_core_module != undefined) {
                let remote_core_moduleInstances = remote_core_module.$instances;
                let moduleInstances = instance.$module.$instances;
                for (let i = 0; i < moduleInstances.length; i++) {
                    for (let j = 0; j < remote_core_moduleInstances.length; j++) {
                        /* Check if the same instance is added in remote core. */
                        if (moduleInstances[i]["instance"] === remote_core_moduleInstances[j]["instance"]) {
                            if (moduleInstances[i]["regionId"] === remote_core_moduleInstances[j]["regionId"]) {
                                report.logError("Same region id cant be used for same EDMA instance", instance, "regionId");
                            }
                        }
                    }
                }
            }
        }
    }
}

function rmValidate(instance, report, resType, rmInstanceName, maxInstance) {
    let rm_instances;
    let resArray = [];
    for (let index = 0; index <= maxInstance; index++) {
        resArray[index] = false;
    }

    let coreNames = common.getSysCfgCoreNames();
    let selfCoreName = common.getSelfSysCfgCoreName();
    let edma_module_name = '/drivers/edma/edma';

    for ( let remoteCoreName of coreNames) {
        rm_instances = undefined;
        if( remoteCoreName != selfCoreName) {
            let remote_core_module = common.getModuleForCore(edma_module_name, remoteCoreName);
            if (remote_core_module != undefined) {
                let remote_core_moduleInstances = remote_core_module.$instances;
                for (let j = 0; j < remote_core_moduleInstances.length; j++) {
                    if (instance["instance"] === remote_core_moduleInstances[j]["instance"]) {
                        rm_instances = remote_core_moduleInstances[j][rmInstanceName];
                        break;
                    }
                }
            }
        } else {
            rm_instances = instance[rmInstanceName];
        }
        if (rm_instances !== undefined) {
            for(let rm = 0; rm < rm_instances.length; rm++) {
                let rm_instance = rm_instances[rm];
                for (let index = rm_instance.startIndex; index <= rm_instance.endIndex; index++) {
                    if(rm_instance["resourceType"] == resType) {
                        if (resArray[index] == false) {
                            resArray[index] = true;
                        }
                        else {
                            report.logError(`Resource Range Overlaps`, instance, rmInstanceName);
                            break;
                        }
                    }
                }
            }
        }
    }
}


function moduleInstances(instance) {

    let rmInstances = new Array();

    if (instance.enableOwnDmaChannelConfig === true) {
        let ownDmaChStart = getRmDefaultStart(instance, "ownDmaCh");
        let ownDmaChEnd   = getRmDefaultEnd(instance, "ownDmaCh");

        rmInstances.push({
            name: "edmaRmDmaCh",
            displayName: "Core Dma Channel Resource Manager",
            moduleName: '/drivers/edma/v0/edma_v0_rm',
            useArray: true,
            minInstanceCount: 1,
            requiredArgs: {
                resourceType: "ownDmaCh",
            },
            args: {
                startIndex: ownDmaChStart,
                endIndex: ownDmaChEnd,
            }
        });
    }

    if (instance.enableOwnQdmaChannelConfig === true) {
        let ownQdmaChStart = getRmDefaultStart(instance, "ownQdmaCh");
        let ownQdmaChEnd   = getRmDefaultEnd(instance, "ownQdmaCh");

        rmInstances.push({
            name: "edmaRmQdmaCh",
            displayName: "Core Qdma Channel Resource Manager",
            moduleName: '/drivers/edma/v0/edma_v0_rm',
            useArray: true,
            minInstanceCount: 1,
            requiredArgs: {
                resourceType: "ownQdmaCh",
            },
            args: {
                startIndex: ownQdmaChStart,
                endIndex: ownQdmaChEnd,
            }
        });
    }

    if (instance.enableOwnTccConfig === true) {
        let ownTccStart = getRmDefaultStart(instance, "ownTcc");
        let ownTccEnd   = getRmDefaultEnd(instance, "ownTcc")

        rmInstances.push({
            name: "edmaRmTcc",
            displayName: "Core Tcc Resource Manager",
            moduleName: '/drivers/edma/v0/edma_v0_rm',
            useArray: true,
            minInstanceCount: 1,
            requiredArgs: {
                resourceType: "ownTcc",
            },
            args: {
                startIndex: ownTccStart,
                endIndex: ownTccEnd,
            }
        });
    }

    if (instance.enableOwnParamConfig === true) {
        let ownParamStart = getRmDefaultStart(instance, "ownParam")
        let ownParamEnd   = getRmDefaultEnd(instance, "ownParam");

        rmInstances.push({
            name: "edmaRmParam",
            displayName: "Core Param Resource Manager",
            moduleName: '/drivers/edma/v0/edma_v0_rm',
            useArray: true,
            minInstanceCount: 1,
            requiredArgs: {
                resourceType: "ownParam",
            },
            args: {
                startIndex: ownParamStart,
                endIndex: ownParamEnd,
            }
        });
    }

    if (instance.enableReservedChannelConfig === true) {
        let reservedDmaChStart = getRmDefaultStart(instance, "reservedDmaCh");
        let reservedDmaChEnd   = getRmDefaultEnd(instance, "reservedDmaCh");

        rmInstances.push({
            name: "edmaRmReservedDmaCh",
            displayName: "Reserved Dma Channel Resource Manager",
            moduleName: '/drivers/edma/v0/edma_v0_rm',
            useArray: true,
            minInstanceCount: 1,
            requiredArgs: {
                resourceType: "reservedDmaCh",
            },
            args: {
                startIndex: reservedDmaChStart,
                endIndex: reservedDmaChEnd,
            }
        });
    }

    if(isChannelTriggerXbarSupported() == true)  {
        let maxCh = 64;
        if(maxCh > 0) {
            rmInstances.push({
                name: "edmaChannel",
                displayName: "EDMA Channel Trigger Configuration",
                moduleName: '/drivers/edma/v0/edma_v0_channel',
                useArray: true,
                minInstanceCount: 0,
                maxInstanceCount: maxCh,
                defaultInstanceCount: 0,
            });
        }
    }

    return (rmInstances);
}

exports = edma_module;
