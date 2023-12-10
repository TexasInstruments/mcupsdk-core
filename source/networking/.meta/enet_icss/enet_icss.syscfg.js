"use strict";

let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let device = common.getDeviceName();
let soc = system.getScript(`/networking/soc/networking_${common.getSocName()}`);
//Get packet pool configuration script
const pktPoolScript = system.getScript("./enet_pkt_pool_config");
// Get MDIO configuration script
 const mdioScript = system.getScript("./enet_icssg_mdio_config");
 // Get TimeSync configuration script
 const timesyncScript = system.getScript("./enet_icssg_timesync_config");

const enet_icssg_udma_channel_config = {
    name: "udmaChConfig",
    displayName: "DMA channel config",
	longDescription: "Configuration of Tx/Rx DMA channels",
    collapsed:true,
    config: [

    ],
};

const enet_icssg_lwipIf_config = {
    name: "lwipIfConfig",
    displayName: "LWIP Interface config",
	longDescription: "Configuration of LWIP Interface",
    collapsed:true,
    config: [

    ],
};

const enet_icssg_system_config = {
    name: "icssgSystemConfig",
    displayName: "System integration config",
    longDescription: "System integration related configuration",
    collapsed:true,
    config: [
        {
            name: "McmEnable",
            description: "Flag to enable multi-client manager. Required for multi-core, multiple Enet client use cases",
            displayName: "Mcm Enable",
            default: false,
        },
        {
            name: "ExternalPhyMgmtEnable",
            description: "Flag to enable phy management in application. The enet driver internal phy functions including phy state machine is bypassed in this mode",
            displayName: "External Phy Management Enable",
            default: false,
        },
        {
            name: "RtosVariant",
            description: "Select FreeRTOS or No RTOS",
            displayName: "RTOS Variant",
            default: "FreeRTOS",
            options: [
                {
                    name: "FreeRTOS",
                    displayName: "FreeRTOS",
                },
                {
                    name: "NoRTOS",
                    displayName: "No RTOS (Bare Metal)",
                },
            ],
        },
        {
            name: "macAddrConfig",
            description: "MAC address to set in the driver. 'Auto Assign shall select the address automatiically from EEPROM and/or EFUSES. 'Manual Entry' will allow to input MAC address",
            displayName: "MAC Address Assignment Method",
            onChange:function (inst, ui) {
                if(inst.macAddrConfig === "Auto Assign") {
                    ui.macAddrList.hidden = true;
                } else {
                    ui.macAddrList.hidden = false;
                }
            },
            options: [
                {
                    name: "Auto Assign",
                },
                {
                    name: "Manual Entry",
                },
            ],
            default: "Auto Assign"
        },
        {
            name: "macAddrList",
            description: "MAC address to set in the driver. Enter MAC address. Seperate multiple MAC address with comma. Eg.: aa:bb:bb:cc:dd:ee,01:22:33:aa:bb:ee",
            displayName: "MAC Address List",
            default: "70:ff:76:1d:ec:f2,70:ff:76:1d:ec:f2",
            hidden: true,
        },
        {
            name: "AppLinkUpPortMask",
            description: "Application config to determine which macPorts should be polled for linkup to indicate link is up.Applicable in multi port scenario only",
            displayName: "AppLinkUpPortMask Config",
            default: "ANY_PORT",
            options: [
                {
                    name: "ALL_PORTS",
                },
                {
                    name: "ANY_PORT",
                },
            ],
        },
    ],
};

function enet_icssg_getPhyaddress(platform, port)
{
    const icssgPhyAddrInfoMap = new Map(
                                           [
                                             ['am64x-evm',{phyAddr1: 15, phyAddr2: 3}],
                                             ['am243x-evm', {phyAddr1: 15, phyAddr2: 3}],
                                             ['am243x-lp',{phyAddr1: 3, phyAddr2: 15,}],
                                           ],
                                         );
    let phyInfo =  icssgPhyAddrInfoMap.get(platform);
    if (port == 1)
    {
        return phyInfo.phyAddr1;
    }
    else
    {
        return phyInfo.phyAddr2;
    }
}

const enet_icssg_board_config = {
    name: "icssgBoardConfig",
    displayName: "Board Config",
    longDescription: "Board specific configuration",
    collapsed:true,
    config: [
        {
            name: "customBoardEnable",
            description: "Enable Custom Board Configuration",
            displayName: "Custom Board",
            longDescription: "Configuration for custom board that are not supported out of box in MCU+SDK",
            default: false,
            onChange:function (inst, ui) {
                if(inst.customBoardEnable == true) {
                    ui.phyAddr1.hidden = true;
                    ui.phyAddr2.hidden = true;
                }
                else {
                    ui.phyAddr1.hidden = false;
                    ui.phyAddr2.hidden = false;
                }
            },
        },
        {
            name: "phyAddr1",
            description: "Phy Address of the port in single/dual EMAC mode or Port 1 in Switch mode. Value MUST be between 0 .. 31",
            displayName: "Phy Address 1",
            default: enet_icssg_getPhyaddress(device, 1),
            displayFormat: "dec",
            isInteger:true,
            range: [0, 31],
        },
        {
            name: "phyAddr2",
            description: "Phy Address of the port in single/dual EMAC mode or Port 2 in Switch mode. Value MUST be between 0 .. 31",
            displayName: "Phy Address 2",
            default: enet_icssg_getPhyaddress(device, 2),
            displayFormat: "dec",
            isInteger:true,
            range: [0, 31],
        },
        {
            name: "useAddMacAddr",
            description: "Use additional MAC addresses from board Mac addresses",
            displayName: "Use Additional MAC Addresses",
            default: false,
            onChange:function (inst, ui) {
                if(inst.useAddMacAddr == true) {
                    ui.addMacAddrCnt.hidden = false;
                }
                else {
                    ui.addMacAddrCnt.hidden = true;
                }
            },
        },
        {
            name: "addMacAddrCnt",
            description: "Phy Address of the port in single/dual EMAC mode or Port 2 in Switch mode. Value MUST be between 0 .. 31",
            displayName: "Additional MAC-Address Count",
            default: 0,
            hidden: true,
            displayFormat: "dec",
            isInteger:true,
            range: [0, 3],
        },
    ],
};

function getInterfaceName(inst, peripheralName)
{
    return `PRU_${inst.instance}_${peripheralName}`;
}

function getInterfacePinList(inst, peripheralName)
{
    let interfaceName = getInterfaceName(inst, peripheralName);
    let pinList = [];

    if(peripheralName=="IEP")
    {
        pinList.push("EDC_LATCH_IN0");
        pinList.push("EDC_SYNC_OUT0");
    }
    else
    {
        pinList = pinmux.getInterfacePinList(interfaceName);
    }

    return pinList;
}


function getPeripheralRequirements(inst, peripheralName, name)
{
    let interfaceName = getInterfaceName(inst, peripheralName);
    let pinList = getInterfacePinList(inst, peripheralName);
    let resources = [];

    if(name == undefined)
    {
        name = interfaceName;
    }
    else
    {
        name = getInterfaceName(inst, name);
    }

    for(let pin of pinList)
    {
        let pinResource = pinmux.getPinRequirements(interfaceName, pin);

        /* make all pins as "rx" and then override to make "rx" as false as needed  */
        pinmux.setConfigurableDefault( pinResource, "rx", true );

        resources.push( pinResource );
    }

    let peripheralRequirements = {
        name: name,
        displayName: name,
        interfaceName: interfaceName,
        resources: resources,
    };

    return peripheralRequirements;
}

function pinmuxRequirements(inst) {

    let mdio = getPeripheralRequirements(inst, "MDIO");
    let iep = getPeripheralRequirements(inst, "IEP");
    let interfaceNameList = [];

    /* set default values for "rx" for different pins, based on use case */
    pinmux.setPeripheralPinConfigurableDefault( mdio, "MDC", "rx", false);
    pinmux.setPeripheralPinConfigurableDefault( iep, "EDC_SYNC_OUT0", "rx", false);

    if( inst.phyToMacInterfaceMode === "MII")
    {
        let mii_g_rt = getPeripheralRequirements(inst, "MII_G_RT");

        pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII0_TXD0", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII0_TXD1", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII0_TXD2", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII0_TXD3", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII0_TXEN", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII1_TXD0", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII1_TXD1", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII1_TXD2", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII1_TXD3", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault( mii_g_rt, "MII1_TXEN", "rx", false);

        if (inst.mdioMdcEnable == true)
        {
            return [mdio, iep, mii_g_rt];
        }
        else
        {
            return interfaceNameList;
        }
    }
    else
    {
        let rgmii1 = getPeripheralRequirements(inst, "RGMII", "RGMII1");
        let rgmii2 = getPeripheralRequirements(inst, "RGMII", "RGMII2");

        let interfaceNameList = [];
        if (inst.mdioMdcEnable == true)
        {
            interfaceNameList.push(mdio, iep);
        }
        if (inst.mode == 'SWITCH')
        {
             interfaceNameList.push(rgmii1, rgmii2);
        }
        else
        {
            if (inst.dualMacPortSelected == 'ENET_MAC_PORT_1')
            {
                interfaceNameList.push(rgmii1);
            }
            else
            {
                interfaceNameList.push(rgmii2);
            }
        }

        return interfaceNameList;
    }
}

function getInterfaceNameList(inst)
{
    let interfaceNameList = [];
    if (inst.phyToMacInterfaceMode === "MII")
    {
        if (inst.mdioMdcEnable == true)
        {
            interfaceNameList.push(
                getInterfaceName(inst, "MDIO"),
                getInterfaceName(inst, "IEP"));
        }
        
        if (inst.mode == 'SWITCH')
        {
            interfaceNameList.push(
                getInterfaceName(inst, "MII_G_RT" ));
        }
        else
        {
            if (inst.dualMacPortSelected == 'ENET_MAC_PORT_1')
            {
                interfaceNameList.push(
                    getInterfaceName(inst, "MII_G_RT" ));
            }
        }
    }
    else if (inst.phyToMacInterfaceMode === "RGMII")
    {
        if (inst.mdioMdcEnable == true)
        {
            interfaceNameList.push(
                getInterfaceName(inst, "MDIO"),
                getInterfaceName(inst, "IEP"));
        }

        if (inst.mode == 'SWITCH')
        {
             interfaceNameList.push(
                getInterfaceName(inst, "RGMII1"),
                getInterfaceName(inst, "RGMII2"));
        }
        else
        {
            if (inst.dualMacPortSelected == 'ENET_MAC_PORT_1')
            {
                interfaceNameList.push(
                    getInterfaceName(inst, "RGMII1"));
            }
            else if (inst.dualMacPortSelected == 'ENET_MAC_PORT_2')
            {
                interfaceNameList.push(
                    getInterfaceName(inst, "RGMII2"));
            }
        }
    }
    return interfaceNameList;
}

function getPeripheralPinNames(inst)
{
    let pinList = [];

    if(inst.phyToMacInterfaceMode === "MII")
    {
        pinList = pinList.concat( getInterfacePinList(inst, "MDIO"),
                        getInterfacePinList(inst, "IEP"),
                        getInterfacePinList(inst, "MII_G_RT" )
        );
    }
    else
    {
        pinList = pinList.concat( getInterfacePinList(inst, "MDIO"),
                        getInterfacePinList(inst, "IEP"),
                        getInterfacePinList(inst, "RGMII" )
        );

    }
    return pinList;
}

function getInstIdTable(instances) {
    let tbl = '{ '
    for (var i = 0; i < instances.length; i++)
    {
        tbl += '{';
        var matchedInst = getInstId(instances[i])
        tbl += i + ', ' + matchedInst.enetType + ', ' +  matchedInst.instId
        tbl += '}, '
    }
    tbl += '}'
    return tbl;
}

function getInstId(instance) {

    let matchInstId;

    const IcssgDualMacInstIdMap = [
        {mode:"DUAL MAC", instance:"ICSSG0", macPort:"ENET_MAC_PORT_1", instId:"0", enetType:"ENET_ICSSG_DUALMAC"},
        {mode:"DUAL MAC", instance:"ICSSG0", macPort:"ENET_MAC_PORT_2", instId:"1", enetType:"ENET_ICSSG_DUALMAC"},
        {mode:"DUAL MAC", instance:"ICSSG1", macPort:"ENET_MAC_PORT_1", instId:"2", enetType:"ENET_ICSSG_DUALMAC"},
        {mode:"DUAL MAC", instance:"ICSSG1", macPort:"ENET_MAC_PORT_2", instId:"3", enetType:"ENET_ICSSG_DUALMAC"},
    ];
    if(instance.mode == "DUAL MAC")
    {
        matchInstId = IcssgDualMacInstIdMap.find(element => element.instance === instance.instance
            && element.macPort === instance.dualMacPortSelected);
    }
    else
    {
        const IcssgSwtInstIdMap = [
                                        {mode:"SWITCH", instance:"ICSSG0",instId:"0", enetType:"ENET_ICSSG_SWITCH"},
                                        {mode:"SWITCH", instance:"ICSSG1",instId:"1", enetType:"ENET_ICSSG_SWITCH"},
                                ];
        matchInstId = IcssgSwtInstIdMap.find(element => element.instance === instance.instance)
    }

    return matchInstId;
}

function isDualMacIfEnabled(instance, icssgInst, macPort) {
    let isEnabled =  (instance.instance === icssgInst)
                     &&
                     (instance.dualMacPortSelected === macPort);
    return isEnabled;
}

function isSwitchIfEnabled(instance, icssgInst) {
    let isEnabled =  ((instance.instance === icssgInst));
    return isEnabled;
}

function isIcssgIfEnabled(instance, mode, icssgInst, macPort) {
    let isEnabled = false;
    if (instance.mode == mode) {
        if (mode === "DUAL MAC") {
            isEnabled = isDualMacIfEnabled(instance, icssgInst, macPort);
        }
        else {
            isEnabled = isSwitchIfEnabled(instance, icssgInst);
        }
    }
    return isEnabled;
}

function getMacPortInfo(instance) {
    let macPortInfo = {numMacPorts:0, macPortList:[]};
    if (instance.mode == "DUAL MAC") {
        macPortInfo.numMacPorts++;
        macPortInfo.macPortList.push(instance.dualMacPortSelected);
    }
    else
    {
        macPortInfo.numMacPorts++;
        macPortInfo.macPortList.push('ENET_MAC_PORT_1');

        macPortInfo.numMacPorts++;
        macPortInfo.macPortList.push('ENET_MAC_PORT_2');
    }
    return macPortInfo;
}

function getPhyMask(instance) {
    let macPortInfo = getMacPortInfo(instance);
    let phyMask = '(' + '0';

    for (var i in macPortInfo.macPortList)
    {
        if (macPortInfo.macPortList[i] == 'ENET_MAC_PORT_1')
        {
            phyMask += ' | ' + '( 1 << ' + instance.phyAddr1 + ')';
        }
        if (macPortInfo.macPortList[i] == 'ENET_MAC_PORT_2')
        {
            phyMask += ' | ' + '( 1 << ' + instance.phyAddr2 + ')';
        }
    }
    phyMask += ')';
    return phyMask;
}

function getMacAddrCount(instance) {
    let totalMacAddr = 0;
    let dma_ch_instances = instance.rxDmaChannel;
    let module_dma_ch = system.modules[`/networking/enet_icss/enet_icssg_tx_channel`];

    for(let ch = 0; ch < dma_ch_instances.length; ch++) {
        let ch_instance = dma_ch_instances[ch];
        let ch_config = module_dma_ch.getInstanceConfig(ch_instance);
        totalMacAddr += ch_config.macAddrCount;
    }
    return totalMacAddr;
}

function getPacketsCount(instance, channelType) {
    let totalNumPackets = 0;
    let dma_ch_instances;
    let module_dma_ch;

    if (channelType === "TX")
    {
        dma_ch_instances = instance.txDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_icss/enet_icssg_tx_channel`];
        
        for(let ch = 0; ch < dma_ch_instances.length; ch++) {
        	let ch_instance = dma_ch_instances[ch];
        	let ch_config = module_dma_ch.getInstanceConfig(ch_instance);
       	 	totalNumPackets += ch_config.PacketsCount;
    	}
    }
    else
    {
        dma_ch_instances = instance.rxDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_icss/enet_icssg_rx_channel`];

        for(let ch = 0; ch < dma_ch_instances.length; ch++) 
        {
        	let ch_instance = dma_ch_instances[ch];
        	let ch_config = module_dma_ch.getInstanceConfig(ch_instance);
       	 	totalNumPackets += ch_config.PacketsCount;
    	}
    }
    return totalNumPackets;
}

function getChannelCount(instance, channelType) {
    let totalNumChannels = 0;
    let dma_ch_instances;
    let module_dma_ch;

    if (channelType === "TX")
    {
        dma_ch_instances = instance.txDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_icss/enet_icssg_tx_channel`];
    }
    else
    {
        dma_ch_instances = instance.rxDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_icss/enet_icssg_rx_channel`];
    }

    for(let ch = 0; ch < dma_ch_instances.length; ch++) {
        let ch_instance = dma_ch_instances[ch];
        let ch_config = module_dma_ch.getInstanceConfig(ch_instance);
        totalNumChannels++;
    }
    return totalNumChannels;
}

function getNetifCount(instance) {
    let totalNumNetifs = 0;
    let instances;
    let module;

    instances = instance.netifInstance;
    module = system.modules[`/networking/enet_icss/enet_icssg_lwipif_netif`];

    for(let num = 0; num < instances.length; num++) {
        let num_instance = instances[num];
        let num_config = module.getInstanceConfig(num_instance);
        totalNumNetifs++;
    }
    return totalNumNetifs;
}

function getNetifConfig(instance, InstNum) {
    let instances;
    let module;

    instances = instance.netifInstance;
    module = system.modules[`/networking/enet_icss/enet_icssg_lwipif_netif`];

    let cfgArray = new Array();


    for(let num = 0; num < instances.length; num++) {
        let num_instance = instances[num];
        let num_config = module.getInstanceConfig(num_instance);
        cfgArray.push(num_config);
    }
    return cfgArray[InstNum];
}

function getDefaultNetifCount(instance)
{
    let defaultNetifCount = 0;

    for (let Idx = 0; Idx < getNetifCount(instance); Idx++)
    {
        defaultNetifCount += (getNetifConfig(instance, Idx).isDefault === true) ? 1 : 0;
    }
    return defaultNetifCount;

}

function getTxPacketsCount(instance) {
    return getPacketsCount(instance, "TX");
}

function getRxPacketsCount(instance) {
    return getPacketsCount(instance, "RX");
}

function getTxChannelCount(instance) {
    return getChannelCount(instance, "TX");
}

function getRxChannelCount(instance) {
    return getChannelCount(instance, "RX");
}

function getChannelConfig(instance, channelType, chTypeInstNum) {
    let dma_ch_instances;
    let module_dma_ch;

    if (channelType === "TX")
    {
        dma_ch_instances = instance.txDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_icss/enet_icssg_tx_channel`];
    }
    else
    {
        dma_ch_instances = instance.rxDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_icss/enet_icssg_rx_channel`];
    }

    let channelCfgArray = new Array();
    for(let ch = 0; ch < dma_ch_instances.length; ch++) {
        let ch_instance = dma_ch_instances[ch];
        let ch_config = module_dma_ch.getInstanceConfig(ch_instance);
        channelCfgArray.push(ch_config);
    }
    return channelCfgArray[chTypeInstNum];
}

function getDefaultPacketCount(channelType) {
    let module_dma_ch;

    if (channelType === "TX")
    {
        module_dma_ch = system.modules[`/networking/enet_icss/enet_icssg_tx_channel`];
    }
    else
    {
        module_dma_ch = system.modules[`/networking/enet_icss/enet_icssg_rx_channel`];
    }
    return (module_dma_ch.config.filter(o => o.name === 'PacketsCount'))[0].default;
}


function getCpuID() {
    return system.getScript(`/drivers/soc/drivers_${common.getSocName()}`).getCpuID();
}

function getRxChIdxCount(instance, RefChIdx)
{
    let count = 0;

    for (let chIdx = 0; chIdx < getRxChannelCount(instance); chIdx++)
    {
        if (getChannelConfig(instance, "RX", chIdx).chIdx === RefChIdx)
        {
            count += 1;
        }
    }
    return count;
}

function getMatchInstances(inst)
{
    let instances = system.modules["/networking/enet_icss/enet_icss"].$instances;
    let matchedInst = new Array();
    let i = 0;
    let j = 0;
    for(let i = 0; i < instances.length; i++)
    {
        if (instances[i].instance == inst)
        {
            matchedInst.push(i);
        }
    }
    return matchedInst;
}

function getInstMacCnt(){

    let instances = system.modules["/networking/enet_icss/enet_icss"].$instances;
    let rxMacAddrCnt = 0;

    for (let i in instances)
    {
        for (var j = 0; j < getRxChannelCount(instances[i]); j++)
        {
            rxMacAddrCnt += (getChannelConfig(instances[i], "RX", j)).macAddrCount;
        }
        if((instances[i].useAddMacAddr === true))
        {
            rxMacAddrCnt += instances[i].addMacAddrCnt;
        }
    }
    return rxMacAddrCnt;
}

function validateInstances(instance, report) {
    let instances = system.modules["/networking/enet_icss/enet_icss"].$instances;
    let matchedInst0 = new Array();
    let matchedInst1 = new Array();

    if (instances.length >= 2)
    {
        matchedInst0 = getMatchInstances ("ICSSG0")
        matchedInst1 = getMatchInstances ("ICSSG1")
        if (matchedInst0.length > 2)
        {
            report.logError(`Exceeded maximum count for ICSSG0 instances`, instance);
        }
        if (matchedInst1.length > 2)
        {
            report.logError(`Exceeded maximum count for ICSSG1 instances`, instance);
        }
        if (matchedInst0.length == 2)
        {
            if (instances[matchedInst0[0]].mode != "DUAL MAC")
            {
                report.logError(`ENET ICSSG0 mode setting should "DUAL MAC" in both the instances`, instance);
            }
            if (instances[matchedInst0[1]].mode != "DUAL MAC")
            {
                report.logError(`ENET ICSSG0 mode setting should "DUAL MAC" in both the instances`, instance);
            }
            if (instances[matchedInst0[0]].dualMacPortSelected === instances[matchedInst0[1]].dualMacPortSelected)
            {
                report.logError(`ENET ICSSG0 'Dual Mac Port' setting should be different across both the instances`, instance);
            }
        }
        if (matchedInst1.length == 2)
        {
            if (instances[matchedInst1[0]].mode != "DUAL MAC")
            {
                report.logError(`ENET ICSSG1 mode setting should "DUAL MAC" in both the instances`, instance);
            }
            if (instances[matchedInst1[1]].mode != "DUAL MAC")
            {
                report.logError(`ENET ICSSG1 mode setting should "DUAL MAC" in both the instances`, instance);
            }
            if (instances[matchedInst1[0]].dualMacPortSelected === instances[matchedInst1[1]].dualMacPortSelected)
            {
                report.logError(`ENET ICSSG1 'Dual Mac Port' setting should be different across both the instances`, instance);
            }
        }


/*
        if (instances[0].mdioMdcEnable != instances[1].mdioMdcEnable)
        {
            report.logError(`ENET ICSSG 'mdioMdcEnable' setting should be same across both the instances`, instance);
        }
*/


    }
    return instances;
}

function validate(instance, report) {
    pktPoolScript.validate(instance, report);
    mdioScript.validate(instance, report);
    timesyncScript.validate(instance, report);
    validateInstances(instance, report)

    if ((getInstMacCnt() > 4) && (instance.useAddMacAddr === true))
    {
        report.logError(`Allocated MAC addresses are more than maximum available mac addresses`, instance, "addMacAddrCnt");
    }
    if (instance.mode === "SWITCH")
    {
        if (getNetifCount(instance) > 1)
        {
            report.logError(`ICSSG Switch case should have only one netif `, instance, "netifInstance");
        }

        if (getRxChIdxCount(instance, 0) != getRxChIdxCount(instance, 1))
        {
            report.logError(`Number of Rx Ch Index with value '1' should be same as'0'`, instance);
        } 
    }
    if (instance.mode === "DUAL MAC")
    {
        let numNetifsCount = getNetifCount(instance);
        if (numNetifsCount > 1)
        {
            report.logError(`ICSSG DUAL MAC case should have only one netif per ICSSG instance `, instance, "netifInstance");
        }
        if (numNetifsCount > 0)
        {
            if (getDefaultNetifCount(instance) !=1)
            {
                report.logError(`Only one netif can be set as default`, instance, "netifInstance");
            }
        }

        if (getMacAddrCount(instance) < numNetifsCount)
        {
             report.logError("Number of MAC address allocated is not enough to number of LwIP NetIFs", instance);
        }
    }
    if (/^([0-9a-fA-F]{2}[:-]){5}[0-9a-fA-F]{2}(,([0-9a-fA-F]{2}[:-]){5}[0-9a-fA-F]{2})+/.test(instance.macAddrList) == false)
    {
        report.logError(`Invalid macAddrList Entry`, instance, "macAddrList");
    }
}

function moduleInstances(instance) {

    let Instances = new Array();
    let maxTxCh     = 8;
    let maxRxCh     = 8;
    let minRxChNum  = 1;
    let maxNetif  = 2;

    if (instance.mode === 'SWITCH')
    {
        maxRxCh = 16;
        minRxChNum = 2;
    }
    else
    {
        maxRxCh = 8;
        minRxChNum = 1;
    }


    Instances.push({
        name: "txDmaChannel",
        displayName: "ENET tx dma channel",
        moduleName: `/networking/enet_icss/enet_icssg_tx_channel`,
        useArray: true,
        minInstanceCount: 1,
        maxInstanceCount: maxTxCh,
        defaultInstanceCount: 1,
        collapsed: false,
        group: "udmaChConfig",
    });

    Instances.push({
        name: "rxDmaChannel",
        displayName: "ENET rx dma channel",
        moduleName: `/networking/enet_icss/enet_icssg_rx_channel`,
        useArray: true,
        minInstanceCount: minRxChNum,
        maxInstanceCount: maxRxCh,
        defaultInstanceCount: minRxChNum,
        collapsed: false,
        group: "udmaChConfig",
    });


    Instances.push({
        name: "netifInstance",
        displayName: "NETIF instance",
        moduleName: `/networking/enet_icss/enet_icssg_lwipif_netif`,
        useArray: true,
        minInstanceCount: 0,
        maxInstanceCount: maxNetif,
        defaultInstanceCount: 0,
        collapsed:false,
        group: "lwipIfConfig",
    });

    return (Instances);
}


function addSharedModuleInstances(inst) {
    let modInstances = new Array();

    modInstances.push({
        name: "icss",
        displayName: "PRU Configuration",
        moduleName: '/drivers/pruicss/pruicss',
        requiredArgs: {
            instance: inst.instance,
            coreClk: 250*1000000,
        },
    });

    modInstances.push({
        name: "udmaDrv",
        displayName: "Udma",
        moduleName: "/drivers/udma/udma",
        requiredArgs: {
            instance: "PKTDMA_0",
            $name: "CONFIG_UDMA_PKTDMA_0",
        },
    });

    return (modInstances);
}

function getCpuInfo() {
	const cpuInfo = new Map(
                               [
                                 ['CSL_CORE_ID_R5FSS0_0',{subsystem: "R5FSS",
                                  clusternum: "0", core: "0"}],
                                 ['CSL_CORE_ID_R5FSS0_1',{subsystem: "R5FSS",
                                  clusternum: "0", core: "1"}],
                                 ['CSL_CORE_ID_R5FSS1_0',{subsystem: "R5FSS",
                                  clusternum: "1", core: "0"}],
                                 ['CSL_CORE_ID_R5FSS1_1', {subsystem: "R5FSS",
                                  clusternum: "1", core: "1"}],
                               ],
                             );
	return cpuInfo.get(getCpuID());
}

let enet_icss_module_name = "/networking/enet_icss/enet_icss";

let enet_icss_module = {

    displayName: "Enet (ICSS)",
	longDescription: "Driver for Industrial Communication Subsystem - Gigabit (ICSSG) which is std. Ethernet similar to CPSW but TI recommendation is to use ICSSG for Industrial Ethernet use-cases",
    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: enet_icss_module_name,
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/networking/enet_icss/templates/enet_icss.h.xdt",
            moduleName: enet_icss_module_name,
        },
        "/board/board/board_config.h.xdt": {
            board_config: "/networking/enet_icss/templates/enet_board_cfg_am64x_am243x.h.xdt",
            moduleName: enet_icss_module_name,
        },
        "/board/board/board_config.c.xdt": {
            board_config: "/networking/enet_icss/templates/enet_board_cfg_am64x_am243x.c.xdt",
            moduleName: enet_icss_module_name,
        },
        "/networking/common/enet_config.c.xdt": {
            enet_mem_config: "/networking/enet_icss/templates/enet_app_memutils_cfg.c.xdt",
            enet_syscfg_info: "/networking/enet_icss/templates/enet_app_syscfg_info.c.xdt",
            moduleName: enet_icss_module_name,
        },
        "/networking/common/enet_config.h.xdt": {
            enet_config: "/networking/enet_icss/templates/enet_syscfg.h.xdt",
            moduleName: enet_icss_module_name,
        },
        "/networking/common/enet_open.c.xdt": {
            enet_open: "/networking/enet_icss/templates/enet_init.c.xdt",
            enet_init_config: "/networking/enet_icss/templates/enet_app_icssg_cfg.c.xdt",
            moduleName: enet_icss_module_name,
        },
        "/networking/common/enet_open.h.xdt": {
            enet_open: "/networking/enet_icss/templates/enet_init.h.xdt",
            moduleName: enet_icss_module_name,
        },
        "/networking/common/enet_soc.c.xdt": {
            enet_soc: "/networking/enet_icss/templates/enet_soc_cfg_am64x_am243x.c.xdt",
            moduleName: enet_icss_module_name,
        },
        "/networking/common/enet_lwipif.c.xdt": {
            enet_lwipif: "/networking/enet_icss/templates/enet_lwipif.c.xdt",
            moduleName: enet_icss_module_name,
        },
        "/networking/common/enet_lwipif.h.xdt": {
            enet_lwipif: "/networking/enet_icss/templates/enet_lwipif.h.xdt",
            moduleName: enet_icss_module_name,
        },
    },
    defaultInstanceName: "CONFIG_ENET_ICSS",
    config: [
        {
            name: "instance",
            displayName: "Instance",
            default: "ICSSG1",
            options: [
                {
                    name: "ICSSG0",
                },
                {
                    name: "ICSSG1",
                }
            ],
        },
        {
            name: "mode",
            displayName: "EMAC Mode",
            default: "SWITCH",
            options: [
                {
                    name: "SWITCH",
                },
                {
                    name: "DUAL MAC",
                },
            ],
            onChange: function (inst, ui) {
                /* Init delay applicable only for single master mode */
                if(inst.mode == "SWITCH") {
                    ui.dualMacPortSelected.hidden = true;
                }
                else {
                    ui.dualMacPortSelected.hidden = false;
                }
            },
        },
        {
            name: "phyToMacInterfaceMode",
            displayName: "MII/RGMII",
            default: "MII",
            options: [
                {
                    name: "MII",
                },
                {
                    name: "RGMII",
                },
            ],
        },
        {
            name: "mdioMdcEnable",
            displayName: "Enable Mdio MDC Config",
            description: "Enable MDIO MDC config for current module",
            default: true,
        },
        {
            name: "dualMacPortSelected",
            displayName: "Dual-Mac Mode Port",
            default: 'ENET_MAC_PORT_1',
            hidden: true,
            options: [
                {
                    name: "ENET_MAC_PORT_1",
                },
                {
                    name: "ENET_MAC_PORT_2",
                },
                {
                    name: "ENET_MAC_PORT_1 & ENET_MAC_PORT_2",
                },
            ],
            getDisabledOptions: () => {
                return [{
                    name: "ENET_MAC_PORT_1 & ENET_MAC_PORT_2",
                    reason: "Not yet supported"
                }]
            },
            description: "Enabled MAC port in Dual mac mode",
        },
        {
            name: "QoS",
            description: "No of QoS level required",
            displayName: "QoS Level",
            default: 8,
            isInteger: true,
            range: [1, 8],
            displayFormat: "dec",
        },
        {
            name: "PremptionEnable",
            description: "Flag to enable premption",
            displayName: "Premption Enable",
            default: false,
        },
        {
            name: "GigabitSupportEnable",
            description: "Decides buffer pool allocation based on interface speed selected",
            displayName: "Gigabit Support",
            default: true,
        },
        enet_icssg_system_config,
        enet_icssg_udma_channel_config,
        mdioScript.config,
        timesyncScript.config,
        pktPoolScript.config,
        enet_icssg_lwipIf_config,
        enet_icssg_board_config,
    ],
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    moduleInstances: moduleInstances,
    sharedModuleInstances: addSharedModuleInstances,
    pinmuxRequirements,
    getInterfaceNameList,
    getPeripheralPinNames,
    getInstIdTable,
    getInstId,
    isIcssgIfEnabled,
    getMacPortInfo,
    getPhyMask,
    getCpuID,
    getCpuInfo,
    getTxPacketsCount,
    getRxPacketsCount,
    getRxChannelCount,
    getTxChannelCount,
    getChannelConfig,
    getDefaultPacketCount,
    getNetifCount,
    getNetifConfig,
    validate: validate,
};

exports = enet_icss_module;
