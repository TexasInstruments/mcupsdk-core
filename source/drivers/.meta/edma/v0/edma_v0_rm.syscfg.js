
let common = system.getScript("/common");

function getConfigArr() {
    return system.getScript(`/drivers/edma/soc/edma_${common.getSocName()}`).getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let configArr = getConfigArr();
    let config = configArr.find( o => o.name === moduleInstance.instance);

    return {
        ...config,
        ...moduleInstance,
    };
};

let edma_rm_module_name = "/drivers/edma/v0/edma_v0_rm";

let edma_rm_module = {
    displayName: "EDMA RM configuration",
    defaultInstanceName: "CONFIG_EDMA_RM",
    moduleName: edma_rm_module_name,
    config: [
        /*  attributes */
        {
            name: "resourceType",
            displayName: "Resource Type",
            default: "ownDmaCh",
            options: [
                { name: "ownDmaCh",  displayName: "Core Dma Channel" },
                { name: "ownQdmaCh", displayName: "Core Qdma Channel" },
                { name: "ownTcc",    displayName: "Core Tcc" },
                { name: "ownParam",  displayName: "Core Param" },
                { name: "reservedDmaCh",  displayName: "Reserved Dma Channel" },
            ],
            description: "Own Resource Type",
        },
        {
            name: "startIndex",
            displayName: "Start Index",
            default: 0,
            description: "Own Channel Start Index",
            displayFormat: "dec",
        },
        {
            name: "endIndex",
            displayName: "End Index",
            default: 31,
            description: "Own Channel End Index",
            displayFormat: "dec",
        },
    ],
    validate : validate,
};

/*
 *  ======== validate ========
 */
function validate(inst, report) {
    let parent = inst.$ownedBy;
    let instConfig = getInstanceConfig(parent);
    if (inst["resourceType"] == "ownDmaCh")
    {
        common.validate.checkNumberRange(inst, report, "startIndex", 0, Number(instConfig.maxDmaChannels -1), "dec");
        common.validate.checkNumberRange(inst, report, "endIndex", Number(inst["startIndex"]), Number(instConfig.maxDmaChannels - 1), "dec");
    }
    if (inst["resourceType"] == "ownQdmaCh")
    {
        common.validate.checkNumberRange(inst, report, "startIndex", 0, Number(7), "dec");
        common.validate.checkNumberRange(inst, report, "endIndex", Number(inst["startIndex"]), Number(7), "dec");
    }
    if (inst["resourceType"] == "ownTcc")
    {
        common.validate.checkNumberRange(inst, report, "startIndex", 0, Number(instConfig.maxTcc -1), "dec");
        common.validate.checkNumberRange(inst, report, "endIndex", Number(inst["startIndex"]), Number(instConfig.maxTcc -1), "dec");
    }
    if (inst["resourceType"] == "ownParam")
    {
        common.validate.checkNumberRange(inst, report, "startIndex", 0, Number(instConfig.maxPaRAM -1), "dec");
        common.validate.checkNumberRange(inst, report, "endIndex", Number(inst["startIndex"]), Number(instConfig.maxPaRAM -1), "dec");
    }
    if (inst["resourceType"] == "reservedDmaCh")
    {
        common.validate.checkNumberRange(inst, report, "startIndex", 0, Number(instConfig.maxDmaChannels -1), "dec");
        common.validate.checkNumberRange(inst, report, "endIndex", Number(inst["startIndex"]), Number(instConfig.maxDmaChannels - 1), "dec");
    }
}

exports = edma_rm_module;
