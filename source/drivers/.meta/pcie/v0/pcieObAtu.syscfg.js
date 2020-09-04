
let common = system.getScript("/common");

let maxObRegion = 32;

function getInstanceConfig(moduleInstance) {
    return {
        ...moduleInstance,
    };
};

let pcieObAtuCfg = {
    displayName: "PCIe Outbound Address Translation",
    defaultInstanceName: "OB_ATU_CFG",
    maxInstances: maxObRegion,
    config: [
        {
            name: "regIndex",
            displayName: "Region Index",
            default: 1,
        },
        {
            name: "tlpType",
            displayName: "Transaction Layer Packet Type",
            default: "PCIE_TLP_TYPE_MEM",
            options: [
                {
                    name: "PCIE_TLP_TYPE_MEM",
                    displayName: "Memory Type TLP",
                },
                {
                    name: "PCIE_TLP_TYPE_IO",
                    displayName: "IO Type TLP",
                },
                {
                    name: "PCIE_TLP_TYPE_CFG",
                    displayName: "CFG Type TLP",
                }
            ]
        },
        {
            name: "lowerBase",
            displayName: "Lower Base Address",
            default: "0x0",
        },
        {
            name: "upperBase",
            displayName: "Upper Base Address",
            default: "0x0",
        },
        {
            name: "lowerTarget",
            displayName: "Lower Target Address",
            default: "0x0",
        },
        {
            name: "upperTarget",
            displayName: "Upper Target Address",
            default: "0x0",
        },
        {
            name: "windowSize",
            displayName: "Region Window Size",
            displayFormat: "hex",
            default: 0xFFF
        },
        {
            name: "externs",
            displayName: "Extern Variables",
            description: "Specify the external variable used if any here",
            default: ""
        }
    ],
    validate: (inst, report) => {
        validate(inst, report);
        checkDuplicateIndex(inst, report);
    },
    getInstanceConfig,
};

function validate(inst, report) {
    if(inst.$ownedBy.operMode == "PCIE_RC_MODE")
    {
        common.validate.checkNumberRange(inst, report, "regIndex", 1, maxObRegion, "dec");
    }
    else
    {
        common.validate.checkNumberRange(inst, report, "regIndex", 0, maxObRegion, "dec");
    }
}

exports = pcieObAtuCfg;

function checkDuplicateIndex(inst,report) {
    if (system.modules["/drivers/pcie/v0/pcieObAtu"]) {
        for (let instances of system.modules["/drivers/pcie/v0/pcieObAtu"].$instances) {
            if(instances != inst && inst.$ownedBy == instances.$ownedBy && instances.regIndex == inst.regIndex) {
                report.logError("Duplicate index" , inst,"regIndex");
            }
        }
    }
}
