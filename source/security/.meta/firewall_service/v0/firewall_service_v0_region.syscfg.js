
let common = system.getScript("/common");
let device_peripheral = system.getScript(`/security/firewall_service/soc/mpu_firewall_config_${common.getSocName()}`);

function getConfigArr() {
    return system.getScript(`/security/firewall_service/soc/mpu_firewall_config_${common.getSocName()}`).getConfigArr();
}

function getFirewallNameArr() {
    let mpu_firewall_names = []
    let mpu_firewall_config = getConfigArr()
    for(let i = 0; i < mpu_firewall_config.length; i++)
    {
        let instance = { name: mpu_firewall_config[i].name, displayName: mpu_firewall_config[i].name }
        mpu_firewall_names.push(instance)
    }
    return mpu_firewall_names
}

function getFirewallConfig(firewallId) {
    const mpu_firewall_config_arr = getConfigArr()
    const {regionCount, memSpace} = mpu_firewall_config_arr.find(element => element.name == firewallId)
    const mpu_firewall_config = {regionCount, memSpace}
    return mpu_firewall_config
}


function getInstanceConfig(moduleInstance) {
    let configArr = getConfigArr();
    let config = configArr.find( o => o.name === moduleInstance.instance);

    return {
        ...config,
        ...moduleInstance,
    };
};

let id_list = device_peripheral.getAidList();
let default_id_list = device_peripheral.getdefaultAidList();
let mpu_firewall_names = getFirewallNameArr()
let permission_list = [
	{ name: "PER0", displayName:"User Execute" },
	{ name: "PER1", displayName:"User Write" },
    { name: "PER2", displayName:"User Read" },
	{ name: "PER3", displayName:"Supervisor Execute" },
    { name: "PER4", displayName:"Supervisor Write" },
	{ name: "PER5", displayName:"Supervisor Read" },
    { name: "PER6", displayName:"Emulation" },
    { name: "PER7", displayName:"Non Secure Access" },
]

let mpu_firewall_region_module = {
    displayName: "Runtime MPU Firewall Region Configuration",
    longDescription: `This adds and configures a MPU Region.`,
    defaultInstanceName: "CONFIG_MPU_FIREWALL_REGION",
    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/security/firewall_service/templates/firewall_service_config.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/security/firewall_service/templates/firewall_service.h.xdt",
        },
    },
    config: [
        {
            name: "firewallId",
            displayName: "Firewall Id",
            description: "MUST be <= maximum number of configurable regions",
            default     : mpu_firewall_names[0].name,
            options     : mpu_firewall_names,
            onChange: function (inst, ui) {

                let mpu_firewall_config = getFirewallConfig(inst.firewallId)
                inst.startAddr = mpu_firewall_config.memSpace[0].startAddr;
                inst.endAddr = mpu_firewall_config.memSpace[0].startAddr + mpu_firewall_config.memSpace[0].size - 1;
            }
        },
        {
            name: "regionNum",
            displayName: "Region Number",
            description: "MUST be <= maximum number of configurable regions",
            default: 0,
            displayFormat: "dec",
        },
        {
            name: "startAddr",
            displayName: "Region Start Address (HEX)",
            description: "MUST be <= 32 bits and within firewall space",
            default: getFirewallConfig(mpu_firewall_names[0].name).memSpace[0].startAddr,
            displayFormat: "hex",
        },
        {
            name: "endAddr",
            displayName: "Region End Address (HEX)",
            description: "MUST be <= 32 bits and within firewall space",
            default: getFirewallConfig(mpu_firewall_names[0].name).memSpace[0].startAddr +  getFirewallConfig(mpu_firewall_names[0].name).memSpace[0].size -1,
            displayFormat: "hex",
        },
        {
            name: "allaidConfig",
            displayName: "Enable All ID's",
            default: false,
            onChange: function (inst, ui) {
                if(inst.allaidConfig == true) {
                    let idArray = [];
                    for (let i = 0; i < id_list.length; i++) {
                        idArray[i] = id_list[i].name;
                    }
                    inst.aidConfig = idArray;
                    ui.aidConfig.readOnly = true;
                }
                else
                {
                    ui.aidConfig.readOnly = false;
                }
            },
        },
        {
            name: "aidConfig",
            displayName: "Priv ID's Allowed",
            description: "Select ID's which can access the region",
            hidden      : false,
            readOnly    : false,
            default     : default_id_list,
            minSelections: 1,
            options     : id_list
        },
        {
            name: "allPermissions",
            displayName: "Enable All Permissions",
            default: false,
            onChange: function (inst, ui) {
                if(inst.allPermissions == true) {
                    var permArray = [];
                    for (let i = 0; i < permission_list.length; i++) {
                        permArray[i] = permission_list[i].name;
                    }
                    inst.permissionConfig = permArray;
                    ui.permissionConfig.readOnly = true;
                }
                else{
                    ui.permissionConfig.readOnly = false;
                }
            },
        },
        {
            name: "permissionConfig",
            displayName: "Permission Config",
            description: "Select access permissions for the region",
            hidden      : false,
            readOnly    : false,
            default     : ["PER6", "PER7"],
            minSelections: 0,
            options     : permission_list
        },
    ],
    validate : function (instance, report) {
        let instConfig = getFirewallConfig(instance.firewallId);
        let startAddrErrFlag = true;
        let endAddrErrFlag = true;
        let startAddrAlignErrFlag = true;
        let endAddrAlignErrFlag = true;
        let regionNumErrFlag = true;
        let sizeErrFlag = true;
        for( let i = 0; i < instConfig.memSpace.length; i++)
        {
            let regionStartAddr = instConfig.memSpace[i].startAddr;
            let regionEndAddr = instConfig.memSpace[i].startAddr + instConfig.memSpace[i].size - 1;
            if ( (instance.startAddr >= regionStartAddr) && (instance.startAddr <= regionEndAddr))
            {
                startAddrErrFlag = false;
            }
            if ((startAddrErrFlag == false) && (instance.endAddr >= regionStartAddr) && (instance.endAddr <= regionEndAddr))
            {
                endAddrErrFlag = false;
            }
            if ( (startAddrErrFlag == false) && (endAddrErrFlag == false) && (instance.endAddr - instance.startAddr >= 1023))
            {
                sizeErrFlag = false;
            }
            if( instance.startAddr % 1024 == 0 )
            {
                startAddrAlignErrFlag = false;
            }
            if( (instance.endAddr + 1) % 1024 == 0 )
            {
                endAddrAlignErrFlag = false;
            }
            if (instConfig.regionCount > instance.regionNum)
            {
                regionNumErrFlag = false;
            }
        }

        if(startAddrErrFlag == true)
        {
            report.logError( `Region Start address must be within firewall space`, instance, "startAddr");
        }
        if(startAddrAlignErrFlag == true)
        {
            report.logError( `Region Start address is not a multiple of 1KB (1024)`, instance, "startAddr");
        }
        if(endAddrErrFlag == true)
        {
            report.logError( `Region End address must be within firewall space`, instance, "endAddr");
        }
        if(endAddrAlignErrFlag == true)
        {
            report.logError( `(Region End address - 1) is not a multiple of 1KB (1024)`, instance, "endAddr");
        }
        if( sizeErrFlag == true)
        {
            report.logError( `Region size must be >= 0x400`, instance, "endAddr");
        }
        if( regionNumErrFlag == true)
        {
            report.logError( `Region number is out of bounds`, instance, "regionNum");
        }
    },
    getInstanceConfig,
    permission_list,
};

exports = mpu_firewall_region_module;
