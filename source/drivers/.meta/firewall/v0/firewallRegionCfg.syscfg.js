
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/firewall/soc/firewall_${common.getSocName()}`);
let firewallConfig = system.getScript(`/drivers/firewall/soc/firewall_am64x_am243x.json`);

function getInstanceConfig(moduleInstance) {
    let privConfigArr = soc.getMasterConfigArr();
    let fwlArr = soc.getFirewallConfig();
    let priv = {};
    let addr = {};
    let maxRegion = 0;
    let ownerInst = moduleInstance.$ownedBy
    let fwlId = ownerInst.fwlId;
    let config = fwlArr.find(o => o.fwl_name === fwlId);
    priv['privConfig1'] = privConfigArr.find(o => o.name === moduleInstance.fwlPriv1);
    priv['privConfig2'] = privConfigArr.find(o => o.name === moduleInstance.fwlPriv2);
    priv['privConfig3'] = privConfigArr.find(o => o.name === moduleInstance.fwlPriv3);
    maxRegion = getFirewallMaxRegions(ownerInst);
    if (moduleInstance.manualAddr === true) {
        addr['startAddr'] = moduleInstance.startAddr;
        addr['endAddr'] = moduleInstance.endAddr;
    }
    else {
        let regInstance = config['Modules'].find(o => (o.Instance === moduleInstance.instName && o.Interface === moduleInstance.interName));
        if (regInstance == undefined) {
            addr['startAddr'] = 0;
            addr['endAddr'] = 0;
        }
        else {
            addr['startAddr'] = Number(regInstance.start_addr);
            addr['endAddr'] = Number(regInstance.end_addr);
        }
    }
    return {
        ...moduleInstance,
        ...priv,
        maxRegion,
        ...addr,
    };
};

let custPermDescription = `

  The firewall permission register layout is
  ----------------------------------------------------
  |  15:12     |   11:8     |   7:4      |   3:0      |
  ----------------------------------------------------

  In each of the 4 nibbles from 15:0 the 4 bits means Debug, Cache, Read, Write Access for
  Non-secure user, Non-secure Priv, Secure user, Secure Priv respectively.

  For Debug, Write permission (1001) user can give permission word like this
  Eg. 0x9999
`

function getPermissionWord(user) {
    let permList = []
    if (user.secure) {
        permList.push('FWL_PERM_SEC_MASK')
    }
    if (user.nsecure) {
        permList.push('FWL_PERM_NSEC_MASK')
    }
    if (user.priv) {
        permList.push('FWL_PERM_PRIV_MASK')
    }
    if (user.user) {
        permList.push('FWL_PERM_USER_MASK')
    }
    return permList;
}

function getMasterConfigArr() {
    return soc.getMasterConfigArr();
}

function getMasterPrivIds() {
    let result = getMasterConfigArr().map(({ name }) => ({ name }));
    return result;
}

function getFirewallMaxRegions(inst) {
    let fwlId = inst.fwlId;
    if (!fwlId) return 0
    let fwlArr = soc.getFirewallConfig();
    let config = fwlArr.find(o => o.fwl_name === fwlId);
    return config.fwl_regions;
}

function getFirewallLockedRegions(inst) {
    let fwlId = inst.fwlId;
    if (!fwlId) return [];
    let fwlArr = soc.getFirewallConfig();
    let config = fwlArr.find(o => o.fwl_name === fwlId);
    return config.locked_regions;
}

let firewallRegionCfg = {
    displayName: "Firewall Region Configuration",
    defaultInstanceName: "FWL_REG_CFG",
    config: [
        {
            name: "regIndex",
            displayName: "Region Index",
            default: 0,
        },
        {
            name: "instName",
            options: (inst) => {
                let ownerInst = inst.$ownedBy;
                let fwlArr = soc.getFirewallConfig();
                let config = fwlArr.find(o => o.fwl_name === ownerInst.fwlId);
                let result = config["Modules"].map(({ Instance }) => ({ name: Instance }));
                result = result.filter((value, index, self) =>
                    index === self.findIndex((t) =>
                        (t.name === value.name)))
                return result;
            },
            description: "Instance for the configuring firewall",
            longDescription: "",
            displayName: "Instance",
            default: "none",
        },
        {
            name: "interName",
            options: (inst) => {
                if (inst.instName == 'none') {
                    return ([{ name: 'none' }])
                }
                let ownerInst = inst.$ownedBy
                let fwlArr = soc.getFirewallConfig();
                let config = fwlArr.find(o => o.fwl_name === ownerInst.fwlId);
                let instConfig = config['Modules'].filter(o => o.Instance === inst.instName);
                let result = instConfig.map(({ Interface }) => ({ name: Interface }));
                return result;
            },
            description: "Interface of the Instance",
            longDescription: "",
            displayName: "Interface",
            default: "none"
        },
        {
            name: "manualAddr",
            displayName: "Manual Region Address",
            description: 'Check here to enter manual address configuration',
            hidden: false,
            default: false,
            onChange: function (inst, ui) {
                let hideConfigs = true;
                let hideInstName = false;
                if (inst.manualAddr == true) {
                    hideConfigs = false;
                    hideInstName = true;
                }
                ui.startAddr.hidden = hideConfigs;
                ui.endAddr.hidden = hideConfigs;
                ui.instName.hidden = hideInstName;
                ui.interName.hidden = hideInstName;
            }
        },
        {
            name: "startAddr",
            displayName: "Start Address",
            description: "Start Address of the Region",
            hidden: true,
            default: 0,
            displayFormat: { radix: "hex", bitSize: 32 },
        },
        {
            name: "endAddr",
            displayName: "End Address",
            description: "End Address of the Region",
            hidden: true,
            default: 4095,
            displayFormat: { radix: "hex", bitSize: 32 },
        },
        {
            name: "fwlEnable",
            displayName: "Enable Region",
            description: 'Check to Enable firewall for this region',
            hidden: false,
            default: true,
        },
        {
            name: "fwlBg",
            displayName: "Background Region",
            description: 'Check to configure this region as background',
            hidden: false,
            default: false
        },
        {
            name: "fwlLock",
            displayName: "Lock Region",
            description: 'Check to lock this region',
            hidden: false,
            default: false
        },
        {
            name: "fwlCache",
            displayName: "Cache Ignore",
            description: 'Check for Ignoring cacheable transaction',
            hidden: false,
            default: false
        },
        {
            name: "fwlPermCfg1",
            displayName: "Permission Configuration 1",
            collapsed: false,
            config: [
                {
                    name: "fwlPriv1",
                    displayName: "Privilege ID",
                    default: getMasterPrivIds()[0].name,
                    description: "The Slave name of the Firewall region",
                    options: getMasterPrivIds(),
                },
                {
                    name: "fwlPerm1",
                    displayName: "Permission",
                    default: "FWL_PERM_RW_ALL",
                    description: "The Slave name of the Firewall region",
                    options: [{
                        name: "FWL_PERM_RW_ALL",
                        displayName: "Allow all"
                    },
                    {
                        name: "FWL_PERM_DENY_ALL",
                        displayName: "Deny all"
                    },
                    {
                        name: "FWL_PERM_RO_ALL",
                        displayName: "Read Only"
                    },
                    {
                        name: "FWL_PERM_WO_ALL",
                        displayName: "Write Only"
                    },
                    {
                        name: "custom",
                        displayName: "Custom"
                    }
                    ],
                    onChange: function (inst, ui) {
                        let hideConfigs = true;
                        if (inst.fwlPerm1 == "custom") {
                            hideConfigs = false;
                        }
                        ui.fwlPrivCustom1.hidden = hideConfigs;
                    }
                },
                {
                    name: "fwlPrivCustom1",
                    displayName: "Custom Permission",
                    description: "Binary value of custom permission",
                    longDescription: custPermDescription,
                    hidden: true,
                    default: 0,
                    displayFormat: { radix: "hex", bitSize: 16 },
                }
            ]
        },
        {
            name: "fwlPermCfg2",
            displayName: "Permission Configuration 2",
            collapsed: true,
            config: [
                {
                    name: "fwlPriv2",
                    displayName: "Privilege ID",
                    default: getMasterPrivIds()[0].name,
                    description: "The Slave name of the Firewall region",
                    options: getMasterPrivIds(),
                },
                {
                    name: "fwlPerm2",
                    displayName: "Permission",
                    default: "FWL_PERM_RW_ALL",
                    description: "The Slave name of the Firewall region",
                    options: [{
                        name: "FWL_PERM_RW_ALL",
                        displayName: "Allow all"
                    },
                    {
                        name: "FWL_PERM_DENY_ALL",
                        displayName: "Deny all"
                    },
                    {
                        name: "FWL_PERM_RO_ALL",
                        displayName: "Read Only"
                    },
                    {
                        name: "FWL_PERM_WO_ALL",
                        displayName: "Write Only"
                    },
                    {
                        name: "custom",
                        displayName: "Custom"
                    }
                    ],
                    onChange: function (inst, ui) {
                        let hideConfigs = true;
                        if (inst.fwlPerm2 == "custom") {
                            hideConfigs = false;
                        }
                        ui.fwlPrivCustom2.hidden = hideConfigs;
                    }
                },
                {
                    name: "fwlPrivCustom2",
                    displayName: "Custom Permission",
                    description: "Binary value of custom permission",
                    longDescription: custPermDescription,
                    hidden: true,
                    default: 0,
                    displayFormat: { radix: "hex", bitSize: 16 },
                }
            ]
        },
        {
            name: "fwlPermCfg3",
            displayName: "Permission Configuration 3",
            collapsed: true,
            config: [
                {
                    name: "fwlPriv3",
                    displayName: "Privilege ID",
                    default: getMasterPrivIds()[0].name,
                    description: "The Slave name of the Firewall region",
                    options: getMasterPrivIds(),
                },
                {
                    name: "fwlPerm3",
                    displayName: "Permission",
                    default: "FWL_PERM_RW_ALL",
                    description: "The Slave name of the Firewall region",
                    options: [{
                        name: "FWL_PERM_RW_ALL",
                        displayName: "Allow all"
                    },
                    {
                        name: "FWL_PERM_DENY_ALL",
                        displayName: "Deny all"
                    },
                    {
                        name: "FWL_PERM_RO_ALL",
                        displayName: "Read Only"
                    },
                    {
                        name: "FWL_PERM_WO_ALL",
                        displayName: "Write Only"
                    },
                    {
                        name: "custom",
                        displayName: "Custom"
                    }
                    ],
                    onChange: function (inst, ui) {
                        let hideConfigs = true;
                        if (inst.fwlPerm3 == "custom") {
                            hideConfigs = false;
                        }
                        ui.fwlPrivCustom3.hidden = hideConfigs;
                    }
                },
                {
                    name: "fwlPrivCustom3",
                    displayName: "Custom Permission",
                    description: "Binary value of custom permission",
                    longDescription: custPermDescription,
                    hidden: true,
                    default: 0,
                    displayFormat: { radix: "hex", bitSize: 16 },
                }]
        }
    ],
    validate: (inst, report) => {
        checkIndexRange(inst, report);
        checkLockedRegion(inst, report);
        checkDuplicateIndex(inst, report);
        checkAddress4kbAlignment(inst, report);
        checkRegionOverlap(inst, report);
        checkInstanceSelection(inst, report);
    },
    getInstanceConfig,
    getPermissionWord,
}

function checkIndexRange(inst, report) {
    let maxRegion = getFirewallMaxRegions(inst.$ownedBy);
    common.validate.checkNumberRange(inst, report, "regIndex", 0, maxRegion - 1, "dec");
}

function checkLockedRegion(inst, report) {
    let lockedRegions = getFirewallLockedRegions(inst.$ownedBy);
    if (lockedRegions.includes(inst.regIndex)) {
        report.logError("This Region is Locked by DMSC", inst, "regIndex");
    }
}

function checkDuplicateIndex(inst, report) {
    if (system.modules["/drivers/firewall/v0/firewallRegionCfg"]) {
        for (let instances of system.modules["/drivers/firewall/v0/firewallRegionCfg"].$instances) {
            if (instances != inst && inst.$ownedBy == instances.$ownedBy && instances.regIndex == inst.regIndex) {
                report.logError("Duplicate index", inst, "regIndex");
            }
        }
    }
}

function checkAddress4kbAlignment(inst, report) {
    if ((inst.endAddr - inst.startAddr) < 4095) {
        report.logInfo("Region is smaller than 4Kb", inst, "startAddr");
    }
    if (inst.startAddr % 4096 != 0) {
        report.logError("Start Address is not 0x000 Aligned", inst, "startAddr");
    }
    if ((inst.endAddr >= 4096 && inst.endAddr + 1) % 4096 != 0) {
        report.logError("End Address is not 0xFFF Aligned", inst, "endAddr");
    }
}

function checkRegionOverlap(inst, report) {
    if (system.modules["/drivers/firewall/v0/firewallRegionCfg"]) {
        let enabledRegions = 0;
        let allowedRegions = 0;
        for (let instances of system.modules["/drivers/firewall/v0/firewallRegionCfg"].$instances) {
            if (instances != inst && inst.$ownedBy == instances.$ownedBy && instances.fwlEnable && inst.fwlEnable) {
                enabledRegions++;
                //  Check if overlap is allowed
                let currInst = getInstanceConfig(inst);
                let otherInst = getInstanceConfig(instances);
                if (((currInst.startAddr >= otherInst.startAddr) &&
                    (currInst.startAddr <= otherInst.endAddr)) ||
                    ((currInst.endAddr >= otherInst.startAddr) &&
                        (currInst.endAddr <= otherInst.endAddr)) ||
                    ((currInst.startAddr <= otherInst.startAddr) &&
                        (currInst.endAddr >= otherInst.endAddr))) {
                    //  Check if background bit is set mutual exclusive.
                    if (((instances.fwlBg) &&
                        (!inst.fwlBg)) ||
                        ((!instances.fwlBg) &&
                            (inst.fwlBg))) {
                        allowedRegions++;
                    }
                }
                else {
                    allowedRegions++;
                }
            }
        }
        if (enabledRegions != allowedRegions) {
            report.logError("Region Overlap Error", inst, "regIndex");
        }
    }
}

function checkInstanceSelection(inst, report) {
    if(inst.manualAddr == false){
        if (inst.instName == 'none') {
            report.logInfo("Select Instance to Configure", inst, "instName");
        }
        if (inst.instName != 'none' && inst.interName == 'none') {
            report.logInfo("Select Interface of the Instance to Configure", inst, "interName");
        }
    }
}

exports = firewallRegionCfg;
