"use strict";

let common = system.getScript("/common");
let device = common.getDeviceName();

const utilsScript = system.getScript("./../../common/enet_cpsw_utils");

function enet_cpsw_ale_validate(instance, report) {
    const ipWhiteListMaxCnt = 4;

    if (instance.IpNxtWhitelist.length  > ipWhiteListMaxCnt)
    {
        report.logError(`Max length of WhiteList cannot exceed ipWhiteListMaxCnt`, instance, "IpNxtWhitelist");        
    }
}

const enet_cpsw_ale_policer_global_config = {
    name: "alePolicerGlobalConfig",
    displayName: "Policer Config",
	longDescription: "Configuration of CPSW ALE policer init time params",
    config: [
        {
            name: "policingEn",
            description: "Enables the policing to color the packets. It also enables red or yellow drop capabilities",
            displayName: "Policing Enable",
            default: false,
            onChange: function (inst, ui) {
                /* Init delay applicable only for single master mode */
                if(inst.policingEn == false) {
                    ui.yellowDropEn.hidden = true;
                    ui.redDropEn.hidden = true;
                    ui.policerNoMatchMode.hidden = true;
                }
                else {
                    ui.yellowDropEn.hidden = false;
                    ui.redDropEn.hidden = false;
                    ui.policerNoMatchMode.hidden = false;
                }
            },
        },
        {
            name: "yellowDropEn",
            description: "Enables the ALE to drop yellow packets based on the yellow threshold value",
            displayName: "Drop Yellow-marked Packets",
            default: false,
            hidden: true,            
            onChange: function (inst, ui) {
                /* Init delay applicable only for single master mode */
                if(inst.yellowDropEn == false) {
                    ui.yellowThresh.hidden = true;
                }
                else {
                    ui.yellowThresh.hidden = false;
                }
            },
        },
        {
            name: "redDropEn",
            hidden: true,
            description: "Enables the ALE to drop the red colored packets",
            displayName: "Drop Red-marked Packets",
            default: false,
        },
        {
            name: "yellowThresh",
            description: "Percentage of yellow marked  packets to drop",
            displayName: "Yellow Packet Drop Percent",
            hidden: true,
            default: "DROP_PERCENT_100",
            options: [
                {
                    name: "DROP_PERCENT_100",
                    description: "Drop 100% of packets colored yellow by policer",
                },
                {
                    name: "DROP_PERCENT_50",
                    description: "Drop 50% of packets colored yellow by policer",
                },
                {
                    name: "DROP_PERCENT_33",
                    description: "Drop 33% of packets colored yellow by policer",
                },
                {
                    name: "DROP_PERCENT_25",
                    description: "Drop 25% of packets colored yellow by policer",
                },
                {
                    name: "DROP_PERCENT_20",
                    description: "Drop 20% of packets colored yellow by policer",
                },
                {
                    name: "DROP_PERCENT_17",
                    description: "Drop 17% of packets colored yellow by policer ",
                },
                {
                    name: "DROP_PERCENT_14",
                    description: "Drop 14% of packets colored yellow by policer ",
                },
                {
                    name: "DROP_PERCENT_13",
                    description: "Drop 13% of packets colored yellow by policer ",
                },
            ],
        },
        {
            name: "policerNoMatchMode",
            description: "Policing policy for no hit (unregulated traffic)",
            displayName: "No Match Policer",
            hidden: true,
            default: "GREEN",
            options: [
                {
                    name: "GREEN",
                    description: "Color packets not matching any classifier entry as GREEN",
                },
                {
                    name: "YELLOW",
                    description: "Color packets not matching any classifier entry as YELLOW",
                },
                {
                    name: "RED",
                    description: "Color packets not matching any classifier entry as RED",
                },
                {
                    name: "UNREGULATED_TRAFFIC_POLICER",
                    description: "No Hit packets are marked based on policing for unregulated traffic",
                },
            ],
            onChange: function (inst, ui) {
                if(inst.policerNoMatchMode === "UNREGULATED_TRAFFIC_POLICER") {
                    ui.UnregulatedTrafficPeakRateInBitsPerSec.hidden = false;
                    ui.UnregulatedTrafficCommitedRateInBitsPerSec.hidden = false;
                }
                else {
                    ui.UnregulatedTrafficPeakRateInBitsPerSec.hidden = true;
                    ui.UnregulatedTrafficCommitedRateInBitsPerSec.hidden = true;
                }
            },
        },
        {
            name: "UnregulatedTrafficPeakRateInBitsPerSec",
            displayName: "Peak Rate (Bits Per Second)",
            longDescription: "Peak rate in bits per second. 0 indicates rate limit is disabled ",
            hidden: true,
            default: 0,
            isInteger: true,
            range: [0, (1 * 1000 * 1000 * 1000)]
        },
        {
            name: "UnregulatedTrafficCommitedRateInBitsPerSec",
            displayName: "Commit Rate (Bits Per Second)",
            longDescription: "Commit rate in bits per second. 0 indicates rate limit is disabled",
            default: 0,
            hidden: true,
            isInteger: true,
            range: [0, (1 * 1000 * 1000 * 1000)]
        },
    ],
    collapsed:true,
};

const enet_cpsw_ale_aging_config = {
    name: "aleAgingConfig",
    displayName: "Aging Config",
	longDescription: "Configuration of CPSW ALE aging related init time params",
    config: [
        {
            name: "autoAgingEn",
            description: "Flag to enable auto aging",
            displayName: "Enable Auto Aging",
            default: true,
            onChange: function (inst, ui) {
                if(inst.autoAgingEn === true) {
                    ui.agingPeriodInMs.hidden = false;
                }
                else {
                    ui.agingPeriodInMs.hidden = true;
                }
            },
        },
        {
            name: "agingPeriodInMs",
            description: "Timeperiod in ms to trigger aging of ALE table entries",
            displayName: "Aging Timeperiod(ms)",
            default: 1000,
            hidden: false,
            isInteger: true,
            range: [500, (10 * 1000)],
            skipTests:["displayNameCheck"],
        },
    ],
    collapsed:true,
};

const enet_cpsw_ale_init_vlan_unknown_vlan_force_untagged_egress_config = {
    name: "unknownVlanForceUntaggedEgress",
    displayName: "Unknown VLAN Force Untagged Egress Config",
	longDescription: "Configuration of forced VLAN stripping on egress for unknown VLANs",
    config: [
        {
            name: "unknownVlanForceUntaggedEgressHostPortEn",
            description: "Enable untagged egress for HostPort",
            displayName: "HostPort Untagged Egress",
            default: false,
        },
        {
            name: "unknownVlanForceUntaggedEgressMacPort1En",
            description: "Enable untagged egress for Macport1",
            displayName: "MacPort1 Untagged Egress",
            default: false,
        },
        {
            name: "unknownVlanForceUntaggedEgressMacPort2En",
            description: "Enable untagged egress for MacPort2",
            displayName: "MacPort2 Untagged Egress",
            default: false,
        },
    ],
    collapsed:true,
};

const enet_cpsw_ale_init_vlan_unknown_vlan_registered_multicast_flood_config = {
    name: "unknownRegMcastFloodMask",
    displayName: "Unknown VLAN Registered Multicast Membership Config",
	longDescription: "Configuration of port membership for registered multicast addresses",
    config: [
        {
            name: "unknownRegMcastHostPortEn",
            description: "Enable HostPort membership for registered multicast address",
            displayName: "HostPort Membership Enable",
            default: true,
        },
        {
            name: "unknownRegMcastMacPort1En",
            description: "Enable MacPort1 membership for registered multicast address",
            displayName: "MacPort1 Membership Enable",
            default: true,
        },
        {
            name: "unknownRegMcastMacPort2En",
            description: "Enable MacPort2 membership for registered multicast address",
            displayName: "MacPort2 Membership Enable",
            default: true,
        },
    ],
    collapsed:true,
};

const enet_cpsw_ale_init_vlan_unknown_vlan_unregistered_multicast_flood_config = {
    name: "unknownUnregMcastFloodMask",
    displayName: "Unknown VLAN Unregistered Multicast Membership config",
	longDescription: "Configuration of port membership for unregistered multicast addresses",
    config: [
        {
            name: "unknownUnregMcastHostPortEn",
            description: "Enable HostPort membership for unregistered multicast address",
            displayName: "HostPort Membership Enable",
            default: true,
        },
        {
            name: "unknownUnregMcastMacPort1En",
            description: "Enable MacPort1 membership for unregistered multicast address",
            displayName: "MacPort1 Membership Enable",
            default: true,
        },
        {
            name: "unknownUnregMcastMacPort2En",
            description: "Enable MacPort2 membership for unregistered multicast address",
            displayName: "MacPort2 Membership Enable",
            default: true,
        },
    ],
    collapsed:true,
};

const enet_cpsw_ale_init_vlan_unknown_vlan_membership_config = {
    name: "unknownVlanMemberListMask",
    displayName: "Unknown VLAN Membership Config",
	longDescription: "Configuration of port membership for unknown VLANs",
    config: [
        {
            name: "unknownVlanMembershipHostPortEn",
            description: "Enable HostPort membership for unknown VLANs",
            displayName: "HostPort Membership Enable",
            default: true,
        },
        {
            name: "unknownVlanMembershipMacPort1En",
            description: "Enable MacPort1 membership for unknown VLANs",
            displayName: "MacPort1 Membership Enable",
            default: true,
        },
        {
            name: "unknownVlanMembershipMacPort2En",
            description: "Enable MacPort2 membership for unknown VLANs",
            displayName: "MacPort2 Membership Enable",
            default: true,
        },
    ],
    collapsed:true,
};

const enet_cpsw_ale_init_vlan_config = {
    name: "aleVlanConfig",
    displayName: "VLAN Config",
	longDescription: "Configuration of CPSW ALE VLAN init time params",
    config: [
        {
            name: "aleVlanAwareMode",
            description: "Configure ALE to operate in VLAN aware mode",
            displayName: "ALE Vlan Aware",
            default: true,
            onChange: function (inst, ui) {
                if(inst.aleVlanAwareMode === true) {
                    utilsScript.hideGroup(utilsScript.getGroupHierarchyByName(inst.$module.config, "aleConfig/aleVlanConfig/unknownVlanCfg"), false, ui);
                }
                else {
                    utilsScript.hideGroup(utilsScript.getGroupHierarchyByName(inst.$module.config, "aleConfig/aleVlanConfig/unknownVlanCfg"), true, ui);
                }
            },
        },
        {
            name: "cpswVlanAwareMode",
            description: "Flag indicating if default port vlan is enabled in CPSW",
            displayName: "Port VLAN Enabled",
            default: false,
            onChange: function (inst, ui) {
                /* Init delay applicable only for single master mode */
                if(inst.cpswVlanAwareMode == false) {
                    utilsScript.hideGroup(utilsScript.getGroupHierarchyByName(inst.$module.config, "aleConfig/alePortConfig/pvidCfg"), true, ui);
                }
                else {
                    utilsScript.hideGroup(utilsScript.getGroupHierarchyByName(inst.$module.config, "aleConfig/alePortConfig/pvidCfg"), false, ui);
                }
            },
        },
        {
            name: "autoLearnWithVlan",
            description: "Learn VLAN with address when autolearning addresses",
            displayName: "Learn Address With Associated VLAN",
            default: false,
        },
        {
            name: "unknownVlanNoLearn",
            description: "Do not learn addresses with unknown VLAN",
            displayName: "Disable Learning For Unknown VLAN",
            default: false,
        },
        {
            name: "unknownVlanCfg",
            displayName: "Unknown VLAN Handling",
            longDescription: "Handling of vlan ids for which entry is not present in ALE table",
            config: [
                enet_cpsw_ale_init_vlan_unknown_vlan_force_untagged_egress_config,
                enet_cpsw_ale_init_vlan_unknown_vlan_registered_multicast_flood_config,
                enet_cpsw_ale_init_vlan_unknown_vlan_unregistered_multicast_flood_config,
                enet_cpsw_ale_init_vlan_unknown_vlan_membership_config,
            ]
        },
    ],
    collapsed:true,
};

const enet_cpsw_ale_network_security_malformed_pkt_config = {
    name: "malformedPktCfg",
    displayName: "Malformed Packet Handling Config",
	longDescription: "Malformed packet handling config",
    config: [
        {
            name: "srcMcastDropDis",
            description: "Disable dropping of packets with Multicast address as SA",
            displayName: "Multicast SA Allow",
            default: false,
        },
        {
            name: "badLenPktDropEn",
            description: "Enable dropping of packets where the 802.3 length field is larger than the packet.",
            longDescription:
`Enable dropping of packets where the 802.3 length field is larger than the packet \n
Ethertypes 0-1500 are 802.3 lengths, all others are EtherTypes`,
            displayName: "Invalid Pkt Len Drop",
            default: false,
        },
    ],
    collapsed:true,
};

const enet_cpsw_ale_network_security_ip_pkt_config = {
    name: "ipPktSecurityCfg",
    displayName: "IP Packet Security Configuration",
	longDescription: "IP packet security configuration",
    config: [
        {
            name: "dfltNoFragEn",
            description: "Enable dropping of fragmented IP packets for which VLAN entry is not found",
            longDescription:"Enable dropping of fragmented IP packets for which VLAN entry is not found\nFor entries with VLAN entry the dropping of fragmented IP packets is set as part of VLAN entry",
            displayName: "Drop Unknown VLAN Fragmented IP Pkt",
            default: false,
        },
        {
            name: "dfltNxtHdrWhitelistEn",
            description: "Enable dropping of packets if VLAN not found AND IP NXT HDR does not match one of ipNxtHdrWhitelist",
            displayName: "Unknown VLAN IPNxtHdr Whitelisting Enable",
            longDescription:"Enable dropping of packets if VLAN not found AND IP NXT HDR does not match one of ipNxtHdrWhitelist",
            default: false,
            skipTests:["displayNameCheck"],
        },
        {
            name: "IpNxtWhitelist",
            displayName: "IP NXT Field Whitelist",
            longDescription: 
`IP Nxt field to be whitelisted. Total of 4 NXT HDRs field can be whitelisted.\n
 Refer https://www.iana.org/assignments/protocol-numbers/protocol-numbers.xhtml`,
            hidden: false,
            default: [],
            minSelections: 0,
            options: _.keys(Array(256)).map((index)=>({name: index})),
            placeholder: "None",
            onChange: function (inst, ui) {
            },
            skipTests:["configLongDescription"],
        },
    ],
    collapsed:true,
};

const enet_cpsw_ale_network_security_mac_authentication_config = {
    name: "macAuthCfg",
    displayName: "MAC Authentication Configuration",
	longDescription: "MAC authentication configuration",
    config: [
        {
            name: "authModeEn",
            description: "Enable MAC Authorization mode",
            displayName: "Enable MAC Authorization Mode",
            default: false,
            onChange: function (inst, ui) {
                /* Init delay applicable only for single master mode */
                if(inst.authModeEn == false) {
                    utilsScript.hideGroup(utilsScript.getGroupHierarchyByName(inst.$module.config, "aleConfig/aleNetworkSecurityConfig/macAuthCfg/macAuthPortDisable"), true, ui);
                }
                else {
                    utilsScript.hideGroup(utilsScript.getGroupHierarchyByName(inst.$module.config, "aleConfig/aleNetworkSecurityConfig/macAuthCfg/macAuthPortDisable"), false, ui);
                }
            },
        },
        {
            name: "macAuthPortDisable",
            description: "Disable mac auth for specific ports",
            displayName: "MacAuth Disable Config",
            config: [
                {
                    name: "disableMacAuthHostPortEn",
                    description: "Disable MAC authentication for HostPort",
                    displayName: "HostPort Mac Auth Disable",
                    hidden:true,
                    default: true,
                },
                {
                    name: "disableMacAuthMacPort1En",
                    description: "Disable MAC authentication for Macport1",
                    displayName: "Macport1 Auth Disable",
                    hidden:true,
                    default: false,
                },
                {
                    name: "disableMacAuthMacPort2En",
                    description: "Disable MAC authentication for MacPort2",
                    displayName: "MacPort2 Auth Disable",
                    hidden:true,
                    default: false,
                },
            ],
        },
    ],
    collapsed:true,
};

const enet_cpsw_ale_network_security_config = {
    name: "aleNetworkSecurityConfig",
    displayName: "Network Security Config",
	longDescription: "Configuration of CPSW ALE network security related init time params",
    config: [
        {
            name: "hostOuiNoMatchDeny",
            description: "Block unmatched OUI to HostPort",
            longDescription:
`When set, any packet with a non-matching OUI source address will be dropped to the host unless \n
the packet destination address matches a supervisory destination address table entry\n
When cleared, any packet source address matching an OUI address table entry will be dropped to \n
the host unless the destination address matches with a supervisory destination address table entry`,
            displayName: "Block Unmatched OUI To HostPort",
            default: false,
        },
        {
            name: "vid0ModeEn",
            description: "Allow packets with VLAN ID = 0",
            displayName: "VID0 Allow",
            default: true,
            hidden: false,
            longDescription: 
`Enable VLAN ID = 0 Mode, When cleared process the priority tagged packet \n
with VID = PORT_VLAN[11:0]. When set process the priority tagged packet \n
with VID = 0`,
        },
        enet_cpsw_ale_network_security_malformed_pkt_config,
        enet_cpsw_ale_network_security_ip_pkt_config,
        enet_cpsw_ale_network_security_mac_authentication_config,
    ],
    collapsed:true,
};

const enet_cpsw_port_learning_security_config_common = {
    name: "learningCfg_#",
    displayName: "Learning Security Config  For #",
    longDescription: "Port specific auto learning security configuration",
    config: [
        {
            name: "nolearn_#",
            description: "Flag to enable no-learning mode",
            displayName: "No Learning",
            default: false,
        },
        {
            name: "noSaUpdteEn_#",
            description: "When set will not update the source addresses for this port",
            displayName: "Disable Source Address Update",
            default: false,
        },
    ],
};

const enet_cpsw_port_learning_security_config = {
    name: "learningCfg",
    displayName: "Learning Security Config",
    longDescription: "Port specific auto learning security configuration",
    config: [
        utilsScript.getPortSpecificConfig(enet_cpsw_port_learning_security_config_common, "#", "hostPort"),
        utilsScript.getPortSpecificConfig(enet_cpsw_port_learning_security_config_common, "#", "macPort1"),
        utilsScript.getPortSpecificConfig(enet_cpsw_port_learning_security_config_common, "#", "macPort2"),
    ],
};

const enet_cpsw_port_vlan_security_config_common = {
    name: "vlanCfg_#",
    displayName: "Vlan Security Related Config For #",
    longDescription: "Port specific VLAN security configuration",
    config: [
        {
            name: "dropUntagged_#",
            description: "When set will drop untagged received packets",
            displayName: "Drop Untagged Packets",
            hidden: false,
            default: false,
        },
        {
            name: "dropDualVlan_#",
            description: "When set will drop any received packets with stag folowed by ctag",
            displayName: "Drop Dual VLAN",
            hidden: false,
            default: false,
        },
        {
            name: "dropDoubleVlan_#",
            description: "When set will drop any received packets with two stags or ctags",
            displayName: "Drop Double VLAN",
            hidden: false,
            default: false,
        },
    ],
};

const enet_cpsw_port_vlan_security_config = {
    name: "vlanCfg",
    displayName: "Vlan Security Config",
    longDescription: "Port specific VLAN security configuration",
    config: [
        utilsScript.getPortSpecificConfig(enet_cpsw_port_vlan_security_config_common, "#", "hostPort"),
        utilsScript.getPortSpecificConfig(enet_cpsw_port_vlan_security_config_common, "#", "macPort1"),
        utilsScript.getPortSpecificConfig(enet_cpsw_port_vlan_security_config_common, "#", "macPort2"),
    ],
};

const enet_cpsw_port_mac_mode_config_common = {
    name: "macModeCfg_#",
    displayName: "MAC-only mode config for #",
    longDescription: "Port specific mac only mode configuration",
    config: [
        {
            name: "macOnlyEn_#",
            description: "Enable MAC-only mode",
            displayName: "Enable MAC-only Mode For #",
            default: false,
            skipTests:["displayNameCheck"],
        },
        {
            name: "macOnlyCafEn_#",
            description: "Promiscuous Mode. MAC-only Copy all frames",
            displayName: "Copy All Frames From MAC-only",
            longDescription:
`MAC-only Copy All Frames \n
When set a Mac Only port will transfer all received good frames to the host \n
When clear a Mac Only port will transfer packets to the host based on \n
ALE destination address lookup operation`,
            hidden: false,
            default: false,
        },
    ],
};

const enet_cpsw_port_mac_mode_config = {
    name: "macModeCfg",
    displayName: "MAC-only mode config",
    longDescription: "Port specific mac only mode configuration, and Promiscuous Mode configuration",
    config: [
        utilsScript.getPortSpecificConfig(enet_cpsw_port_mac_mode_config_common, "#", "hostPort"),
        utilsScript.getPortSpecificConfig(enet_cpsw_port_mac_mode_config_common, "#", "macPort1"),
        utilsScript.getPortSpecificConfig(enet_cpsw_port_mac_mode_config_common, "#", "macPort2"),
    ],
};

const enet_cpsw_ale_port_vlan_vidinfo_config = {
    name: "vlanIdInfo_#",
    displayName: "VLAN ID Info",
    config: [
        {
            name: "vlanId_#",
            description: "VLAN ID of the port",
            displayName: "VLAN ID",
            default: 0,
            isInteger: true,
            range : [0,4094],
            displayFormat: "hex",
            hidden: true,
        },
        {
            name: "vlanPrio_#",
            description: "Port VLAN Priority Value",
            displayName: "Port VLAN Priority",
            default: 0,
            isInteger: true,
            range: [0, 7],
            hidden: true,
        },
        {
            name: "vlanCfiBit_#",
            description: "Port CFI bit",
            displayName: "Set Port CFI Bit",
            default: false,
            hidden: true,
        },
        {
            name: "tagType_#",
            description: "VLAN type is either outer or inner VLAN",
            displayName: "VLAN Type",
            default: "ENET_VLAN_TAG_TYPE_INNER",
            options:
            [
                {
                    name: "ENET_VLAN_TAG_TYPE_INNER",
                    description: "Inner or customer tag ",
                },
                {
                    name: "ENET_VLAN_TAG_TYPE_OUTER",
                    description: "Outer or service tag",
                },
            ],
            hidden:true,
        },
    ],
};

const enet_cpsw_port_vlan_config_common = {
    name: "pvidCfg_#",
    displayName: "Port Default VLAN Config For #",
    longDescription: "Default VLAN ID configuration for #",
    config: [
        enet_cpsw_ale_port_vlan_vidinfo_config,
        {
            name: "vlanMemberList_#",
            displayName: "Port Memberbership PVID",
            longDescription: "Port member mask for the VLAN entry being added",
            config: [
                {
                    name: "vlanMemberList_#_HostPortEn",
                    displayName: "HostPort:Enable PVID Membership",
                    description: "Enable HostPort membership",
                    default: true,
                    hidden:true,
                },
                {
                    name: "vlanMemberList_#_MacPort1En",
                    displayName: "MacPort1:Enable PVID Membership",
                    description: "Enable macport 1 membership",
                    default: true,
                    hidden:true,
                },
                {
                    name: "vlanMemberList_#_MacPort2En",
                    displayName: "MacPort2:Enable PVID Membership",
                    description: "Enable macport 2 membership",
                    default: true,
                    hidden:true,
                },
            ],
        },
        {
            name: "unregMcastFloodMask_#",
            displayName: "Unregistered Multicast Flood Mask",
            longDescription: "Mask used for multicast when the multicast address is not found in ALE table",
            config: [
                {
                    name: "unregMcastFloodMask_#_HostPortEn",
                    displayName: "Unregistered Mcast Flood To Hostport",
                    description: "Enable unregistered multicast flood to hostport",
                    default: true,
                    hidden:true,
                },
                {
                    name: "unregMcastFloodMask_#_MacPort1En",
                    displayName: "Unregistered Mcast Flood To MacPort1",
                    description: "Enable flood to MacPort1",
                    default: true,
                    hidden:true,
                },
                {
                    name: "unregMcastFloodMask_#_MacPort2En",
                    displayName: "Unregistered Mcast Flood To MacPort2",
                    description: "Enable flood to MacPort2",
                    default: true,
                    hidden:true,
                },
            ],
        },
        {
            name: "regMcastFloodMask_#",
            displayName: "Registered Mcast Flood Mask",
            longDescription: "Port flood mask used for multicast when the multicast address is found in ALE table",
            config: [
                {
                    name: "regMcastFloodMask_#_HostPortEn",
                    displayName: "Registered Mcast Flood To Hostport",
                    description: "Enable flood to HostPort",
                    default: true,
                    hidden:true,
                },
                {
                    name: "regMcastFloodMask_#_MacPort1En",
                    displayName: "Registered Mcast Flood To MacPort1",
                    description: "Enable flood to MacPort1",
                    default: true,
                    hidden:true,
                },
                {
                    name: "regMcastFloodMask_#_MacPort2En",
                    displayName: "Registered Mcast Flood To MacPort2",
                    description: "Enable flood to MacPort2",
                    default: true,
                    hidden:true,
                },
            ],
        },
        {
            name: "forceUntaggedEgressMask_#",
            displayName: "Force Untagged Egress",
            longDescription: "Causes the packet VLAN tag to be removed on egress based",
            config: [
                {
                    name: "forceUntaggedEgressMask_#_HostPortEn",
                    displayName: "Force Untagged Egress For Hostport",
                    default: false,
                    hidden:true,
                },
                {
                    name: "forceUntaggedEgressMask_#_MacPort1En",
                    displayName: "Force Untagged Egress For Macport1",
                    default: false,
                    hidden:true,
                },
                {
                    name: "forceUntaggedEgressMask_#_MacPort2En",
                    displayName: "Force Untagged Egress For Macport2",
                    default: false,
                    hidden:true,
                },
            ],
        },
        {
            name: "noLearnMask_#",
            displayName: "No Learn Mask",
            longDescription:
`VLAN No Learn Mask - When a bit is set in this mask, a packet with an \n
unknown source address received on the associated port will not be \n
learned (i.e. When a VLAN packet is received and the source address is \n
not in the table, the source address will not be added to the table)`,
            config: [
                {
                    name: "noLearnMask_#_HostPortEn",
                    displayName: "Disable Learning For HostPort",
                    default: false,
                    hidden:true,
                },
                {
                    name: "noLearnMask_#_MacPort1En",
                    displayName: "Disable Learning For Macport1",
                    default: false,
                    hidden:true,
                },
                {
                    name: "noLearnMask_#_MacPort2En",
                    displayName: "Disable Learning For Macport2",
                    default: false,
                    hidden:true,
                },
            ],
        },
        {
            name: "vidIngressCheck_#",
            description: "When set will check if ingress port has VLAN membership for VLAN ID",
            displayName: "VLAN ID Ingress Check",
            default: false,
            hidden:true,
        },
        {
            name: "limitIPNxtHdr_#",
            displayName: "Limit IP NXT Hdr Field",
            longDescription:
`When set IP packets only with configured \n
NXTHDR will be allowed @sa CpswAle_IPPktSecurityCfg\n`,
            default: false,
            hidden:true,
        },
        {
            name: "disallowIPFrag_#",
            description: "Causes IPv4 fragmented IP frames to be dropped",
            displayName: "Disallow IPv4 Fragmentation",
            default: false,
            hidden:true,
        },
    ],
};

const enet_cpsw_port_vlan_config = {
    name: "pvidCfg",
    displayName: "Port Default VLAN Config",
    longDescription: "Default VLAN ID configuration for each port",
    config: [
        utilsScript.getPortSpecificConfig(enet_cpsw_port_vlan_config_common, "#", "hostPort"),
        utilsScript.getPortSpecificConfig(enet_cpsw_port_vlan_config_common, "#", "macPort1"),
        utilsScript.getPortSpecificConfig(enet_cpsw_port_vlan_config_common, "#", "macPort2"),
    ],
};

const enet_cpsw_port_config = {
    name: "alePortConfig",
    displayName: "ALE Port Config",
    longDescription: "ALE init time port specific configuration params",
    config: [
        enet_cpsw_port_learning_security_config,
        enet_cpsw_port_vlan_security_config,
        enet_cpsw_port_mac_mode_config,
        enet_cpsw_port_vlan_config,
    ],
};


const enet_cpsw_ale_config = {
    name: "aleConfig",
    displayName: "ALE Config",
	longDescription: "Configuration of CPSW ALE module",
    config: [
        {
            name: "BypassEnable",
            description: "Flag to Enable ALE bypass",
            displayName: "ALE Bypass Enable",
            default: false,
        },
        {
            name: "UnknownUnicastFloodToHost",
            description: "Enable Unknown unicast packet flooding to HostPort",
            displayName: "Unknown Unicast Flood To Host",
            default: false,            
        },
        enet_cpsw_ale_policer_global_config,
        enet_cpsw_ale_aging_config,
        enet_cpsw_ale_init_vlan_config,
        enet_cpsw_ale_network_security_config,
        enet_cpsw_port_config,

    ],
    collapsed:true,
};


exports = {
    config: enet_cpsw_ale_config,
    validate: enet_cpsw_ale_validate,
};
