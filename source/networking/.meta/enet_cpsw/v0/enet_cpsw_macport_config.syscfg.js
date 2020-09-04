"use strict";
const utilsScript = system.getScript("./../../common/enet_cpsw_utils");


function enet_cpsw_macport_validate(inst, report) {

    if ((inst.DisableMacPort1 == true) && (inst.DisableMacPort2 == true))
    {
        report.logError("Atleast one MAC port should be enabled", inst);
    }
}

const enet_cpsw_macport_config = {
    name: "macPort#Cfg",
    displayName: "MAC Port # Config",
	longDescription: "Configuration of CPSW MAC PORT #",
    config: [
        {
            name: "DisableMacPort#",
            description: "Flag to selectively disable MACport#. For CPSW3G both external mac ports are enabled by default. Application may selectively choose to disable some external ports",
            displayName: "Disable Mac Port #",
            default: false,
            onChange: function(inst, ui) {
                utilsScript.hideGroup(utilsScript.getGroupHierarchyByName(inst.$module.config, "macPortCfg/macPort1Cfg"), false, ui);
                utilsScript.hideGroup(utilsScript.getGroupHierarchyByName(inst.$module.config, "macPortCfg/macPort2Cfg"), false, ui);
                if (inst.DisableMacPort1 === true)
                {
                    utilsScript.hideGroup(utilsScript.getGroupHierarchyByName(inst.$module.config, "macPortCfg/macPort1Cfg"), true, ui);
                }
                if (inst.DisableMacPort2 === true)
                {
                    utilsScript.hideGroup(utilsScript.getGroupHierarchyByName(inst.$module.config, "macPortCfg/macPort2Cfg"), true, ui);
                }
                ui.DisableMacPort1.hidden = false;
                ui.DisableMacPort2.hidden = false;
            },
        },
        {
            name: "macport#LoopbackEn",
            description: "Whether loopback mode is enabled or not",
            displayName: "Enable MAC Loop-back",
            default: false,
            hidden: false,
        },
        {
            name: "macport#CrcType",
            description: "Type of CRC",
            displayName: "CRC Type",
            default: "ENET_CRC_ETHERNET",
            options: [
                {
                    name: "ENET_CRC_ETHERNET",
                },
                {
                    name: "ENET_CRC_CASTAGNOLI",
                },
            ],
            hidden: false,
        },
        {
            name: "macport#RxMtu",
            description: "Max length of a received frame on ingress. This max length includes VLAN",
            displayName: "Rx MTU",
            default: 1518,
            isInteger: true,
            range: [0, 9604],
            hidden: false,
        },
        {
            name: "macport#PassPriorityTaggedUnchanged",
            description: "Whether priority tagged packets should be passed unchanged (if set to true) or replaced with port's VID (if set to false)",
            displayName: "Un-Change Priority Tagged Packets",
            default: false,
            hidden: false,
        },
        {
            name: "macport#TxPriorityType",
            description: "Egress Priority type: Fixed or Escalate",
            displayName: "Egress Priority Type",
            default: "ENET_EGRESS_PRI_TYPE_FIXED",
            options: [
                {
                    name: "ENET_EGRESS_PRI_TYPE_FIXED",
                },
                {
                    name: "ENET_EGRESS_PRI_TYPE_ESCALATE",
                },
            ],
            hidden: false,
        },
        {
            name: "vlanCfg",
            description: "Port VLAN configuration",
            longDescription: "Port VLAN configuration. Configuration are taken from taken from 'ALE Config' -> 'Port Default Vlan Config')",
            config : [
                {
                    name: "macport#PortVID",
                    description: "Port VLAN ID. '0': frame does not carry a VLAN ID",
                    displayName: "Port VLAN ID",
                    default: 0,
                    readOnly: true,
                    getValue: function(inst) { return inst.vlanId_macPort1 },
                    isInteger: true,
                    range: [0, 4094],
                    displayFormat: "hex",
                    hidden: false,
                },
                {
                    name: "macport#PortPri",
                    description: "Port VLAN priority Value",
                    displayName: "Port VLAN Priority",
                    default: 0,
                    readOnly: true,
                    getValue: function(inst) { return inst.vlanPrio_macPort1 },
                    isInteger: true,
                    range: [0, 7],
                    hidden: false,
                },
                {
                    name: "macport#PortCfi",
                    description: "Port CFI bit",
                    displayName: "Set Port CFI Bit",
                    default: false,
                    readOnly: true,
                    getValue: function(inst) { return inst.vlanCfiBit_macPort1 },
                    hidden: false,
                },
            ],
            collapsed:true,
        },
    ],
    collapsed:true,
};

const enet_cpsw_macport_topConfig = {
    name: "macPortCfg",
    displayName: "MAC Port Config",
    longDescription: "Configuration of CPSW MAC PORTS",
    config: [
        utilsScript.getPortSpecificConfig(enet_cpsw_macport_config, "#", "1"),
        utilsScript.getPortSpecificConfig(enet_cpsw_macport_config, "#", "2"),
    ],
    collapsed: true,
};

exports =
{
    config: enet_cpsw_macport_topConfig,
    validate: enet_cpsw_macport_validate
};
