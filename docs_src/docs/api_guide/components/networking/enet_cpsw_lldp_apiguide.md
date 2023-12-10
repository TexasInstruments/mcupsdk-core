# Ethernet LLDP Stack - API and Integration Guide {#ENET_CPSW_LLDP}

[TOC]

# Pre-requisites
Reader are expected to have basic knowledge on below IEEE specifications
- Standard ethernet (IEEE 802.1)
- Link Layer Discovery Protocol - LLDP (IEEE 802.1AB-2016)
- LLDP Amendment 1: YANG Data Model (IEEE 802.1ABcu-2021)

# Introduction
This guide is intended to enhance user's understanding of the LLDP stack and provide guidance on how to seamlessly integrate LLDP modules into their own applications.

# Demo and Examples
\ref EXAMPLES_ENET_CPSW_LLDP

# TSN Stack

## Compilation
The compilation of library is similar with \ref ENET_CPSW_TSN_GPTP.

## Modules

The TSN Stack library is composed of the following source modules:

 Module Name  | lcoation | Description
 -------------|-----------|-----------
 Unibase      | `<mcu_plus_sdk>/source/networking/tsn/tsn_unibase` | Universal utility libraries that are platform-independent
 Combase      | `<mcu_plus_sdk>/source/networking/tsn/tsn_combase` | Communication utility libraries that provide support for functions like sockets, mutexes, and semaphores
 Uniconf      | `<mcu_plus_sdk>/source/networking/tsn/tsn_uniconf` | Universal configuration daemon for Yang, provides APIs for developing a client application which retreives/writes yang parameters from/to database
 LLDP         | `<mcu_plus_sdk>/source/networking/tsn/tsn_lldp`    | Implementation of the IEEE 802.1 AB LLDP

 ## Stack Initialization

Refer \ref ENET_CPSW_TSN_GPTP. <b>Stack Initialization</b> section

## Logging

Refer \ref ENET_CPSW_TSN_GPTP. <b>Logging</b> section

## Starting uniconf and LLDP applications
Refer \ref ENET_CPSW_TSN_GPTP. <b>Starting uniconf and gPTP</b> section.

This function will start:
- The uniconf task as 1st priority task to be init
- Initial uniconf DB after uniconf is finished by uniconf runtime config or yang config file.
- After uniconf and DB initialization is finished, lldp task is able to start

## LLDP Deinitialization
Refer \ref ENET_CPSW_TSN_GPTP. <b>TSN Deinitialization</b> section

# Integration
## Source integration

Refer \ref ENET_CPSW_TSN_GPTP. <b>Source integration</b> section

## Uniconf configuration

Refer \ref ENET_CPSW_TSN_GPTP. <b>Uniconf configuration</b> section

## LLDP configuration parameters
This section describes the standard Yang parameters utilized for LLDP.
The LLDP Yang structure is defined in IEEE 802.1ABcu 2021 Amendment 1: YANG Data Model.

To configure LLDP params, refer ``EnetApp_setLldpRtConfig`` which is generating per-system and per-port configurations.

For ex, to configure LLDP global timer values like tx-interval, fast init, .... User can update the table accordingly.
```
static EnetApp_DbNameVal_t gLldpGlobalData[] =
{
    {"message-fast-tx" , "1"},
    {"message-tx-hold-multiplier" , "4"},
    {"message-tx-interval" , "30"},
    {"reinit-delay" , "2"},
    {"tx-credit-max" , "5"},
    {"tx-fast-init" , "2"},
};
```

To update local information data which is use to build LLDPDU, User can update the table below accordingly.
```
static EnetApp_DbNameVal_t gLldpLocalSysData[] =
{
    // local system data
    {"chassis-id-subtype" , "7"}, // '7' mean local filled, in case of more than one ports, subtype='4' MAC address cannot apply.
    {"chassis-id" , "00-01-02-03-04-05"}, // Just a simple string in case of subtype is 'local (7)'
    {"system-name" , "tilld"},
    {"system-description" , "tilld"},
    {"system-capabilities-supported" , "11111111111"},
    {"system-capabilities-enabled" , "11110111011"},
};
```

Current LLDP application support up to 3 destination MAC Addresses per one port. The supported destination MAC Addresses should be match with LLDP 802.1AB specification.
- Nearest bridge 01-80-C2-00-00-0E
- Nearest non-TPMR bridge 01-80-C2-00-00-03
- Nearest Customer Bridge 01-80-C2-00-00-00

To configure per-port/dest-mac information, User can update the table `gLldpPortCfgData` accordingly.
These Destination MAC Addresses are corresponding to below configuration value.

```
static EnetApp_LldpPortCfg_t gLldpPortCfgData[] =
{
    {
        .dest_mac = "01-80-c2-00-00-0e",
        .cfg_kv =
        {
            {"admin-status", "3"},
            {"tlvs-tx-enable", "1111"},
            {"port-id-subtype", "3"},               // If PortId subtype = P_MAC_Address (3), MAC addr will be re-correct follow hw info.
            {"port-id", "3c-e0-64-62-e3-03"},       // <- this value will be correct in runtime
            {"port-desc", "tilld"},
            {"message-fast-tx", "2"},               // <- In case of any field missing, global value gLldpGlobalData will be used
            {"message-tx-hold-multiplier" , "4"},   // <- In case of any field missing, global value gLldpGlobalData will be used
            {"message-tx-interval" , "30"},         // <- In case of any field missing, global value gLldpGlobalData will be used
            {"reinit-delay" , "2"},                 // <- In case of any field missing, global value gLldpGlobalData will be used
            {"tx-credit-max" , "5"},                // <- In case of any field missing, global value gLldpGlobalData will be used
            {"tx-fast-init" , "2"},                 // <- In case of any field missing, global value gLldpGlobalData will be used
        }
    },
    {
        .dest_mac = "01-80-c2-00-00-03",
        .cfg_kv =
        {
            {"admin-status", "3"},
            {"tlvs-tx-enable", "1111"},
            {"port-id-subtype", "3"},               // If PortId subtype = P_MAC_Address (3), MAC addr will be re-correct follow hw info.
            {"port-id", "3c-e0-64-62-e3-03"},       // <- this value will be correct in runtime
            {"port-desc", "tilld"},
            {"message-fast-tx", "2"},               // <- In case of any field missing, global value gLldpGlobalData will be used
            {"message-tx-hold-multiplier" , "4"},   // <- In case of any field missing, global value gLldpGlobalData will be used
            {"message-tx-interval" , "20"},         // <- In case of any field missing, global value gLldpGlobalData will be used
            {"reinit-delay" , "2"},                 // <- In case of any field missing, global value gLldpGlobalData will be used
            {"tx-credit-max" , "5"},                // <- In case of any field missing, global value gLldpGlobalData will be used
            {"tx-fast-init" , "2"},                 // <- In case of any field missing, global value gLldpGlobalData will be used
        }
    },
    {
        .dest_mac = "01-80-c2-00-00-00",
        .cfg_kv =
        {
            {"admin-status", "3"},
            {"tlvs-tx-enable", "1111"},
            {"port-id-subtype", "3"},               // If PortId subtype = P_MAC_Address (3), MAC addr will be re-correct follow hw info.
            {"port-id", "3c-e0-64-62-e3-03"},       // <- this value will be correct in runtime
            {"port-desc", "tilld"},
            {"message-fast-tx", "2"},               // Diff with global system config
            {"message-tx-hold-multiplier" , "4"},   // <- In case of any field missing, global value gLldpGlobalData will be used
            {"message-tx-interval" , "25"},         // <- In case of any field missing, global value gLldpGlobalData will be used
            {"reinit-delay" , "2"},                 // <- In case of any field missing, global value gLldpGlobalData will be used
            {"tx-credit-max" , "5"},                // <- In case of any field missing, global value gLldpGlobalData will be used
            {"tx-fast-init" , "2"},                 // <- In case of any field missing, global value gLldpGlobalData will be used
        }
    },
};
```
In case of User want to support only one Destination MAC address, the entry of gLldpPortCfgData can be reduced accordingly.

# See Also

\ref NETWORKING