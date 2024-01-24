"use strict";

let common = system.getScript("/common");
let device = common.getDeviceName();

function enet_cpsw_hostport_validate(instance, report) {

}

const enet_cpsw_hostport_config = {
    name: "hostportConfig",
    displayName: "Host Port Config",
    longDescription: "Configuration of CPSW Host Port module",
    config: [
        {
            name: "hostportCrcType",
            description: "Type of CRC on all port 0 egress, regardless of the CRC type in Ethernet port ingress",
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
        },
        {
            name: "hostportRemoveCrc",
            description: "Whether Or Not CRC Is Removed On Port-0 Egress",
            displayName: "Remove CRC Bytes",
            default: false,
        },
        {
            name: "hostportPadShortPacket",
            description: "Whether short packets (ingress) are padded to 64-bytes or dropped",
            displayName: "Pad Short Packet",
            default: true,
        },
        {
            name: "hostpostPassCrcErrors",
            description: "Whether packets with CRC errors (ingress) are dropped or transferred to the destination ports",
            displayName: "Allow CRC Error Packets",
            default: false,
        },
        {
            name: "hostportRxMtu",
            description: "Max length of a received frame on ingress, including VLAN",
            displayName: "Receive MTU Length",
            default: 1518,
            isInteger: true,
            range: [0, 9604],
        },
        {
            name: "hostportPassPriorityTaggedUnchanged",
            description: "Whether priority tagged packets should be passed unchanged (if set to true) or replaced with port's VID (if set to false)",
            displayName: "Un-Change Priority Tagged Packets",
            default: false,
        },
        {
            name: "hostportTxCsumOffloadEn",
            longDescription: "Enable checksum offload feature which enables both TCP/UDP checksum computation (transmit side) and validation (receive side) to be offloaded to the CPSW HW. The Protocol specific info needs to be populated in the descriptor to indicate the location in the packet where the computed checksum should be inserted",
            displayName: "Enable Checksum Offload",
            default: true,
        },
        {
            name: "hostportRxCsumOffloadEn",
            longDescription: "(Not used in AM243x and AM64x. Hence hidden)",
            displayName: "Enable Checksum Offload",
            default: true,
            hidden: true,
        },
        {
            name: "hostportRxVlanRemapEn",
            longDescription: "RX VLAN remap controls whether the hardware switch priority for VLAN tagged or priority tagged packets is determined from CPPI thread number (remap disabled) or via \ref ENET_HOSTPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP (remap enabled)",
            displayName: "Enable RX VLAN Re-Map",
            default: false,
        },
        {
            name: "hostporRxDscpIPv4RemapEn",
            longDescription: "RX DSCP IPv4 remap controls whether the hardware switch priority for IPv4 packets is determined from CPPI thread number (remap disabled) or via \ref ENET_HOSTPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP (remap enabled)",
            displayName: "Enable RX DSCP (IPv4) Re-Map",
            default: true,
        },
       {
            name: "hostporRxDscpIPv6RemapEn",
            longDescription: "RX DSCP IPv6 remap controls whether the hardware switch priority for IPv4 packets is determined from CPPI thread number (remap disabled) or via \ref ENET_HOSTPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP (remap enabled)",
            displayName: "Enable RX DSCP (IPv6) Re-Map",
            default: false,
        },
        {
            name: "hostportRxPriorityType",
            description: "Ingress Priority type: Fixed or Round Robin",
            displayName: "Ingress Priority Type",
            default: "ENET_INGRESS_PRI_TYPE_FIXED",
            options: [
                {
                    name: "ENET_INGRESS_PRI_TYPE_FIXED",
                },
                {
                    name: "ENET_INGRESS_PRI_TYPE_RR",
                },
            ],
        },
        {
            name: "hostportTxPriorityType",
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
        },
        {
            name: "vlanCfg",
            description: "Port VLAN configuration",
            longDescription: "Port VLAN configuration. Configuration are taken from taken from 'ALE Config' -> 'Port Default Vlan Config')",
            config : [
                {
                    name: "hostportPortVID",
                    description: "Port VLAN ID. '0': frame does not carry a VLAN ID",
                    displayName: "Port VLAN ID",
                    default: 0,
                    readOnly: true,
                    getValue: function(inst) { return inst.vlanId_hostPort },
                    isInteger: true,
                    range: [0, 4094],
                    displayFormat: "hex",
                    hidden: false,
                },
                {
                    name: "hostportPortPri",
                    description: "Port VLAN priority Value",
                    displayName: "Port VLAN Priority",
                    default: 0,
                    readOnly: true,
                    getValue: function(inst) { return inst.vlanPrio_hostPort },
                    isInteger: true,
                    range: [0, 7],
                    hidden: false,
                },
                {
                    name: "hostportCfi",
                    description: "Port CFI bit",
                    displayName: "Set Port CFI Bit",
                    default: false,
                    readOnly: true,
                    getValue: function(inst) { return inst.vlanCfiBit_hostPort },
                    hidden: false,
                },
            ],
            collapsed:true,
        },
    ],
    collapsed:true,
};


exports = {
    config: enet_cpsw_hostport_config,
    validate: enet_cpsw_hostport_validate,
};
