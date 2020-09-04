
let common = system.getScript("/common");

let maxIbRegionEP = 5;
let maxIbRegionRC = 1;

function getInstanceConfig(moduleInstance) {
    return {
        ...moduleInstance,
    };
};

let pcieIbAtuCfg = {
    displayName: "PCIe Outbound Address Translation",
    defaultInstanceName: "IB_ATU_CFG",
    maxInstances: maxIbRegionEP,
    config: [
        {
            name: "regIndex",
            displayName: "Region Index",
            default: 0,
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
        },
        {
            name: "barAperture",
            displayName: "BAR Aperture",
            default: "4K",
            options: [
                {
                    name: "4B",
                    displayName: "4 bytes",
                },
                {
                    name: "8B",
                    displayName: "8 bytes",
                },
                {
                    name: "16B",
                    displayName: "16 bytes",
                },
                {
                    name: "32B",
                    displayName: "32 bytes",
                },
                {
                    name: "64B",
                    displayName: "64 bytes",
                },
                {
                    name: "128B",
                    displayName: "128 bytes",
                },
                {
                    name: "256B",
                    displayName: "256 bytes",
                },
                {
                    name: "512B",
                    displayName: "512 bytes",
                },
                {
                    name: "1K",
                    displayName: "1 KB",
                },
                {
                    name: "2K",
                    displayName: "2 KB",
                },
                {
                    name: "4K",
                    displayName: "4 KB",
                },
                {
                    name: "8K",
                    displayName: "8 KB",
                },
                {
                    name: "16K",
                    displayName: "16 KB",
                },
                {
                    name: "32K",
                    displayName: "32 KB",
                },
                {
                    name: "64K",
                    displayName: "64 KB",
                },
                {
                    name: "128K",
                    displayName: "128 KB",
                },
                {
                    name: "256K",
                    displayName: "256 KB",
                },
                {
                    name: "512K",
                    displayName: "512 KB",
                },
                {
                    name: "1M",
                    displayName: "1 MB",
                },
                {
                    name: "2M",
                    displayName: "2 MB",
                },
                {
                    name: "4M",
                    displayName: "4 MB",
                },
                {
                    name: "8M",
                    displayName: "8 MB",
                },
                {
                    name: "16M",
                    displayName: "16 MB",
                },
                {
                    name: "32M",
                    displayName: "32 MB",
                },
                {
                    name: "64M",
                    displayName: "64 MB",
                },
                {
                    name: "128M",
                    displayName: "128 MB",
                },
                {
                    name: "256M",
                    displayName: "256 MB",
                },
                {
                    name: "512M",
                    displayName: "512 MB",
                },
                {
                    name: "1G",
                    displayName: "1 GB",
                },
                {
                    name: "2G",
                    displayName: "2 GB",
                },
                {
                    name: "4G",
                    displayName: "4 GB",
                },
                {
                    name: "8G",
                    displayName: "8 GB",
                },
                {
                    name: "16G",
                    displayName: "16 GB",
                },
                {
                    name: "32G",
                    displayName: "32 GB",
                },
                {
                    name: "64G",
                    displayName: "64 GB",
                },
                {
                    name: "128G",
                    displayName: "128 GB",
                },
                {
                    name: "256G",
                    displayName: "256 GB",
                },
            ]
        },
        {
            name: "barConfig",
            displayName: "Bar Configuration",
            default: "PCIE_BARC_32B_MEM_BAR_NON_PREFETCH",
            options: [
                {
                    name: "PCIE_BARC_DISABLED",
                    displayName: "Disabled"
                },
                {
                    name: "PCIE_BARC_32B_IO_BAR",
                    displayName: "32bit IO BAR"
                },
                {
                    name: "PCIE_BARC_32B_MEM_BAR_NON_PREFETCH",
                    displayName: "32bit Mem BAR Non Prefetchable"
                },
                {
                    name: "PCIE_BARC_32B_MEM_BAR_PREFETCH",
                    displayName: "32bit IO BAR Prefetchable"
                },
                {
                    name: "PCIE_BARC_64B_MEM_BAR_NON_PREFETCH",
                    displayName: "64bit Mem BAR Non Prefetchable"
                },
                {
                    name: "PCIE_BARC_64B_MEM_BAR_PREFETCH",
                    displayName: "64bit IO BAR Prefetchable"
                },
            ]
        }
    ],
    validate: (inst, report) => {
        validate(inst, report);
        checkDuplicateIndex(inst, report);
        checkValidAperture(inst, report);
        checkValidBarCfg(inst, report);
    },
    getInstanceConfig,
}

function validate(inst, report) {

    if (inst.$ownedBy.operMode == "PCIE_RC_MODE")
    {
        common.validate.checkNumberRange(inst, report, "regIndex", 0, maxIbRegionRC, "dec");
    }
    else
    {
        common.validate.checkNumberRange(inst, report, "regIndex", 0, maxIbRegionEP, "dec");
    }
}

function checkDuplicateIndex(inst,report) {
    if (system.modules["/drivers/pcie/v0/pcieIbAtu"]) {
        for (let instances of system.modules["/drivers/pcie/v0/pcieIbAtu"].$instances) {
            if(instances != inst && inst.$ownedBy == instances.$ownedBy && instances.regIndex == inst.regIndex) {
                report.logError("Duplicate index" , inst,"regIndex");
            }
        }
    }
}

function checkValidAperture(inst, report) {
    if(inst.$ownedBy.operMode == "PCIE_RC_MODE")
    {
        if(inst.regIndex == 1)
        {
            if(inst.barAperture == "4G" || inst.barAperture == "8G" || inst.barAperture == "16G"
                || inst.barAperture == "32G" || inst.barAperture == "64G" || inst.barAperture == "128G"
                || inst.barAperture == "256G")
            {
                report.logError("Invalid aperture for RC Inbound region 1", inst, "barAperture");
            }
        }
    }
    else if(inst.$ownedBy.operMode == "PCIE_EP_MODE")
    {
        if(inst.barAperture == "4G" || inst.barAperture == "8G" || inst.barAperture == "16G"
                || inst.barAperture == "32G" || inst.barAperture == "64G" || inst.barAperture == "128G"
                || inst.barAperture == "256G" || inst.barAperture == "4B" || inst.barAperture == "8B"
                || inst.barAperture == "16B" || inst.barAperture == "32B" || inst.barAperture == "64B")
        {
            report.logError("Invalid aperture for EP Inbound region", inst, "barAperture");
        }
    }
}

function checkValidBarCfg(inst, report) {
    if (inst.$ownedBy.operMode == "PCIE_RC_MODE") {
        if(inst.regIndex == 1 && ( inst.barConfig == "PCIE_BARC_64B_MEM_BAR_NON_PREFETCH" || inst.barConfig == "PCIE_BARC_64B_MEM_BAR_PREFETCH"))
        {
            report.logError("Invalid BAR config for RC mode region 1", inst, "barConfig");
        }
    }
    else if (inst.$ownedBy.operMode == "PCIE_EP_MODE") {
        if(inst.barConfig == "PCIE_BARC_64B_MEM_BAR_NON_PREFETCH" || inst.barConfig == "PCIE_BARC_64B_MEM_BAR_PREFETCH")
        {
            report.logError("Invalid BAR config for EP mode", inst, "barConfig");
        }
    }
}

exports = pcieIbAtuCfg;
