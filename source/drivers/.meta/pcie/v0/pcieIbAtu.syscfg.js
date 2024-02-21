
let common = system.getScript("/common");

let maxIbRegionEP = 6;
let maxIbRegionRC = 1;

function getInstanceConfig(moduleInstance) {
    return {
        ...moduleInstance,
    };
};

let pcieIbAtuCfg = {
    displayName: "PCIe Inbound Address Translation",
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
                    displayName: "32bit Mem BAR Prefetchable"
                },
                {
                    name: "PCIE_BARC_64B_MEM_BAR_NON_PREFETCH",
                    displayName: "64bit Mem BAR Non Prefetchable"
                },
                {
                    name: "PCIE_BARC_64B_MEM_BAR_PREFETCH",
                    displayName: "64bit Mem BAR Prefetchable"
                },
            ]
        }
    ],
    validate: (inst, report) => {
        validate(inst, report);
        checkDuplicateIndex(inst, report);
        check64bitMemBarIndex(inst, report);
        checkValidAperture(inst, report);
        checkValidBarCfg(inst, report);
    },
    getInstanceConfig,
}

let Mem64BitBarAtIndx0 = new Array(3).fill(0);

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
function check64bitMemBarIndex(inst,report) {
    if(inst.regIndex%2 == 1)
    {
        let ind

        switch (inst.regIndex) {
            case 1:
                ind = 0;
                break;
            case 3:
                ind = 1;
                break;
            case 5:
                ind = 2;
                break;
        }

        if (Mem64BitBarAtIndx0[ind] == 1)
        {
            report.logError("Invalid BAR config for EP mode. A BAR cannot be configured on this Region Index since a 64bit memory BAR is already configured at the previous region index. If the BAR configuration on the previous Index shall remain then the current entry must be empty and deleted", inst, "regIndex");
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
        if( inst.barConfig == "PCIE_BARC_32B_MEM_BAR_NON_PREFETCH" || inst.barConfig == "PCIE_BARC_32B_MEM_BAR_PREFETCH" || inst.barConfig == "PCIE_BARC_32B_IO_BAR")
        {

            if (inst.barAperture == "4G"   || inst.barAperture == "8G"  || inst.barAperture == "16G"
                 || inst.barAperture == "32G"  || inst.barAperture == "64G" || inst.barAperture == "128G"
                 || inst.barAperture == "256G")
            {
                report.logError("Invalid aperture for 32bit BAR. Aperture should be smaller.", inst, "barAperture");
            }
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

        if(inst.regIndex%2 == 0)
        {
            let ind

            switch (inst.regIndex) {
                case 0:
                    ind = 0;
                    break;
                case 2:
                    ind = 1;
                    break;
                case 4:
                    ind = 2;
                    break;
            }

            if (inst.barConfig == "PCIE_BARC_64B_MEM_BAR_NON_PREFETCH" || inst.barConfig == "PCIE_BARC_64B_MEM_BAR_PREFETCH")
            {
                Mem64BitBarAtIndx0[ind] = 1;
            }
            else
            {
                Mem64BitBarAtIndx0[ind] = 0;
            }
        }

        /* 64bit cannot be on index 1,3,5 */
        if(inst.regIndex%2 == 1 && (inst.barConfig == "PCIE_BARC_64B_MEM_BAR_NON_PREFETCH" || inst.barConfig == "PCIE_BARC_64B_MEM_BAR_PREFETCH"))
        {
            report.logError("Invalid BAR config for EP mode. A 64bit memory BAR can only be configured on Region Index 0, 2 or 4", inst, "barConfig");
        }

    }
}

exports = pcieIbAtuCfg;
