"use strict";

let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let device = common.getDeviceName();
let soc = system.getScript(`/networking/soc/networking_${common.getSocName()}`);
//Get packet pool configuration script
const pktPoolScript = system.getScript("./enet_pkt_pool_config");
//Get ALE configuration script
const aleScript = system.getScript("./enet_cpsw_ale_config");
//Get MDIO configuration script
const mdioScript = system.getScript("./enet_cpsw_mdio_config");
//Get Host Port configuration script
const hostportScript = system.getScript("./enet_cpsw_hostport_config");
//Get CPTS configuration script
const cptsScript = system.getScript("./enet_cpsw_cpts_config");
//Get MAC Port configuration script
const macportScript = system.getScript("./enet_cpsw_macport_config");
const utilsScript = system.getScript("./../../common/enet_cpsw_utils");

const enet_cpsw_pinmux_config = {
    name: "pinmuxConfig",
    displayName: "Pinmux config",
	longDescription: "Configuration of pinmux for CPSW",
    collapsed:true,
    config: [

    ],
};

const enet_cpsw_cpdma_channel_config = {
    name: "cpdmaChConfig",
    displayName: "DMA channel config",
	longDescription: "Configuration of Tx/Rx DMA channels",
    collapsed:true,
    config: [

    ],
};

const enet_cpsw_lwipIf_config = {
    name: "lwipIfConfig",
    displayName: "LWIP Interface config",
	longDescription: "Configuration of LWIP Interface",
    collapsed:true,
    config: [

    ],
};

const enet_cpsw_system_config = {
    name: "cpswSystemConfig",
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
                    name: "Auto Assign (Not Supported)",
                },
                {
                    name: "Manual Entry",
                },
            ],
            default: "Manual Entry"
        },
        {
            name: "macAddrList",
            description: "MAC address to set in the driver. Enter MAC address. Seperate multiple MAC address with comma. Eg.: aa:bb:bb:cc:dd:ee,01:22:33:aa:bb:ee",
            displayName: "MAC Address List",
            default: "70:ff:76:1d:ec:f2,70:ff:76:1d:ec:f2",
            hidden: false,
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

const enet_cpsw_phy1_config =
{
    name: "phy1Config",
    displayName: "Port 1 PHY Configuration",
    longDescription: "Configuration of PHY connected on Port 1",
    config:
    [
        {
            name: "phyAddr1",
            description: "Phy Address of the port 1. Value MUST be between 0 .. 31",
            displayName: "Address",
            default: 0,
            displayFormat: "dec",
            isInteger:true,
            range: [0, 31],
            readOnly: true,
            getValue:function (inst) {
                const cpswPhyAddrInfoMap = new Map(
                                           [
                                             ['am273x-evm',{phyAddr1: 0}],
                                           ],
                                         );
                let phyInfo =  cpswPhyAddrInfoMap.get(device);
                return phyInfo.phyAddr1;
            },
        },
        {
            name: "disableRgmiiIntDelay",
            description: "Set for disabling RGMII port internal delay mode",
            displayName: "Disable RGMII Internal Delay",
            default: true,
        },
        {
            name: "isC45Phy1",
            description: "Set if this PHY supports MDIO Clause 45 data format",
            displayName: "Clause 45 Support",
            default: false,
            readOnly: true,
        },
    ],
};

const enet_cpsw_board_config = {
    name: "cpswBoardConfig",
    displayName: "Board Config",
    longDescription: "Board specific configuration",
    collapsed:true,
    config: [
        {
            name: "customBoardEnable",
            description: "Enable Custom Board Configuration",
            displayName: "Custom Board",
            longDescription: "Configuration for custom board that are not supported out of box in MCU+ SDK",
            default: false,
            onChange:function (inst, ui) {
                if(inst.customBoardEnable == true) {
                    ui.phyAddr1.hidden = true;
                    ui.isC45Phy1.hidden = true;
                    ui.disableRgmiiIntDelay.hidden = true;
                }
                else {
                    ui.phyAddr1.hidden = false;
                    ui.isC45Phy1.hidden = false;
                    ui.disableRgmiiIntDelay.hidden = false;
                }
            },
        },
        enet_cpsw_phy1_config,
    ],
};

function getInterfaceNameList(inst) {
    let driverVer = soc.getDriverVer("enet_cpsw");
    let pinmux_instances;
    let pinmux_module;
    let pinmux_instance;
    let pinmux_config;

    pinmux_instances = inst.pinmux;
    pinmux_module = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_pinmux`];
    pinmux_instance = pinmux_instances[0];
    pinmux_config = pinmux_module.getInstanceConfig(pinmux_instance);

    return pinmux_module.getInterfaceNameList(pinmux_config);
}

function getPeripheralPinNames(inst)
{
    let driverVer = soc.getDriverVer("enet_cpsw");
    let pinmux_instances;
    let pinmux_module;
    let pinmux_instance;
    let pinmux_config;

    pinmux_instances = inst.pinmux;
    pinmux_module = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_pinmux`];
    pinmux_instance = pinmux_instances[0];
    pinmux_config = pinmux_module.getInstanceConfig(pinmux_instance);

    return pinmux_module.getPeripheralPinNames(pinmux_config);
}

let cpsw_input_clk_freq = 200000000;

const enet_config = [
    {
        clockIds        : [ "SOC_RcmPeripheralId_MSS_CPSW", "SOC_RcmPeripheralId_MSS_CPTS" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_MSS_CPSW",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2",
                clkRate : cpsw_input_clk_freq,
            },
            {
                moduleId: "SOC_RcmPeripheralId_MSS_CPTS",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2",
                clkRate : cpsw_input_clk_freq,
            },
        ],
    },
];

function getClockEnableIds(instance) {
    let instConfig = enet_config;
    return instConfig[0].clockIds;
}

function getClockFrequencies(inst) {

    let instConfig = enet_config;

    return instConfig[0].clockFrequencies;
}

function getDmaInterface(instance) {
    let cpswInstInfo = getCpswInstInfo(instance);
    return cpswInstInfo.dmaIf;
}

function getInstIdTable(instances) {
    let tbl = '{ '
    for (var i = 0; i < instances.length; i++)
    {
        tbl += '{';
        var matchedInst = getCpswInstInfo(instances[i])
        tbl += i + ', ' + matchedInst.enetType + ', ' +  matchedInst.instId
        tbl += '}, '
    }
    tbl += '}'
    return tbl;
}

function getCpswInstInfo(instance) {
    const cpswInstInfoMap = new Map(
                               [
                                 ['awr294x',{enetType: 'ENET_CPSW_2G', numMacPorts: '1', instId: '0', dmaIf:'ENET_SOC_HOSTPORT_DMA_TYPE_CPDMA', macPortList:['ENET_MAC_PORT_1']}],
                                 ['am273x', {enetType: 'ENET_CPSW_2G', numMacPorts: '1', instId: '0', dmaIf:'ENET_SOC_HOSTPORT_DMA_TYPE_CPDMA', macPortList:['ENET_MAC_PORT_1']}],
                                 ['am263x',{enetType: 'ENET_CPSW_3G', numMacPorts: '2', instId: '0', dmaIf:'ENET_SOC_HOSTPORT_DMA_TYPE_CPDMA', macPortList:['ENET_MAC_PORT_1', 'ENET_MAC_PORT_2']}],
                                 ['am243x',{enetType: 'ENET_CPSW_3G', numMacPorts: '2', instId: '0', dmaIf:'ENET_SOC_HOSTPORT_DMA_TYPE_UDMA', macPortList:['ENET_MAC_PORT_1', 'ENET_MAC_PORT_2']}],
                                 ['am64x',{enetType: 'ENET_CPSW_3G', numMacPorts: '2', instId: '0', dmaIf:'ENET_SOC_HOSTPORT_DMA_TYPE_UDMA', macPortList:['ENET_MAC_PORT_1', 'ENET_MAC_PORT_2']}],
                               ],
                             );
    return cpswInstInfoMap.get(common.getSocName());
}

function getBoardConfigTemplateInfo() {
    const boardConfigTemplate = new Map(
                               [
                                 ['am64x',{Cfile: "/networking/enet_cpsw/templates/am64x_am243x/enet_board_cfg.c.xdt",
                                  Header: "/networking/enet_cpsw/templates/am64x_am243x/enet_board_cfg.h.xdt"}],
                                 ['am243x',{Cfile: "/networking/enet_cpsw/templates/am64x_am243x/enet_board_cfg.c.xdt",
                                  Header: "/networking/enet_cpsw/templates/am64x_am243x/enet_board_cfg.h.xdt"}],
                                 ['awr294x',{Cfile: "/networking/enet_cpsw/templates/awr294x/enet_board_cfg.c.xdt",
                                  Header: "/networking/enet_cpsw/templates/awr294x/enet_board_cfg.h.xdt"}],
                                 ['am273x', {Cfile: "/networking/enet_cpsw/templates/am273x/enet_board_cfg.c.xdt",
                                 Header: "/networking/enet_cpsw/templates/am273x/enet_board_cfg.h.xdt"}],
                                 ['am263x',{Cfile: "/networking/enet_cpsw/templates/am263x/enet_board_cfg.c.xdt",
                                 Header: "/networking/enet_cpsw/templates/am263x/enet_board_cfg.h.xdt"}],
                               ],
                             );
    return boardConfigTemplate.get(common.getSocName());
}


function getSocConfigTemplateInfo() {
    const socConfigTemplate = new Map(
                               [
                                 ['am64x',{Cfile: "/networking/enet_cpsw/templates/am64x_am243x/enet_soc_cfg.c.xdt"}],
                                 ['am243x',{Cfile: "/networking/enet_cpsw/templates/am64x_am243x/enet_soc_cfg.c.xdt"}],
                                 ['awr294x',{Cfile: "/networking/enet_cpsw/templates/awr294x/enet_soc_cfg.c.xdt"}],
                                 ['am273x', {Cfile: "/networking/enet_cpsw/templates/am273x/enet_soc_cfg.c.xdt"}],
                                 ['am263x',{Cfile: "/networking/enet_cpsw/templates/am263x/enet_soc_cfg.c.xdt"}],
                               ],
                             );
    return socConfigTemplate.get(common.getSocName());
}

function getPhyMask(instance) {
    let cpswInstInfo = getCpswInstInfo(instance);
    let phyMask = '(' + '0';

    for (var i in cpswInstInfo.macPortList)
    {
        if (cpswInstInfo.macPortList[i] == 'ENET_MAC_PORT_1')
        {
            phyMask += ' | ' + '(1 << ' + instance.phyAddr1 + ' )';
        }
    }
    phyMask += ')';
    return phyMask;
}

function getPacketsCount(instance, channelType) {
    let totalNumPackets = 0;
    let driverVer = soc.getDriverVer("enet_cpsw");
    let dma_ch_instances;
    let module_dma_ch;

    if (channelType === "TX")
    {
        dma_ch_instances = instance.txDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_tx_channel`];
    }
    else
    {
        dma_ch_instances = instance.rxDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_rx_channel`];
    }

    for(let ch = 0; ch < dma_ch_instances.length; ch++) {
        let ch_instance = dma_ch_instances[ch];
        let ch_config = module_dma_ch.getInstanceConfig(ch_instance);
        totalNumPackets += ch_config.PacketsCount;
    }
    return totalNumPackets;
}

function getChannelCount(instance, channelType) {
    let totalNumChannels = 0;
    let driverVer = soc.getDriverVer("enet_cpsw");
    let dma_ch_instances;
    let module_dma_ch;

    if (channelType === "TX")
    {
        dma_ch_instances = instance.txDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_tx_channel`];
    }
    else
    {
        dma_ch_instances = instance.rxDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_rx_channel`];
    }

    for(let ch = 0; ch < dma_ch_instances.length; ch++) {
        let ch_instance = dma_ch_instances[ch];
        let ch_config = module_dma_ch.getInstanceConfig(ch_instance);
        totalNumChannels++;
    }
    return totalNumChannels;
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

function getNumCpdmaDesc(instance) {
    /* Tx packet requires Two Tx scatter gather segments + 1 csum offload descriptor */
    const txDesc2PacketScalingFactor = 3;
    /* Rx packet requires only one desc per packet till scatter gather is supported.
     * Csum info is at end of packet and no cpdma desc is used */
    const rxDesc2PacketScalingFactor = 1;
    let cpdmaNumDesc = (rxDesc2PacketScalingFactor * getRxPacketsCount(instance)) + (txDesc2PacketScalingFactor * getTxPacketsCount(instance));
    return  cpdmaNumDesc;
}

function getChannelConfig(instance, channelType, chTypeInstNum) {
    let driverVer = soc.getDriverVer("enet_cpsw");
    let dma_ch_instances;
    let module_dma_ch;

    if (channelType === "TX")
    {
        dma_ch_instances = instance.txDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_tx_channel`];
    }
    else
    {
        dma_ch_instances = instance.rxDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_rx_channel`];
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
    let driverVer = soc.getDriverVer("enet_cpsw");
    let module_dma_ch;

    if (channelType === "TX")
    {
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_tx_channel`];
    }
    else
    {
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_rx_channel`];
    }
    return (module_dma_ch.config.filter(o => o.name === 'PacketsCount'))[0].default;
}


function getNetifCount(instance) {
    let driverVer = soc.getDriverVer("enet_cpsw");
    let totalNumNetifs = 0;
    let instances;
    let module;

    instances = instance.netifInstance;
    module = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_lwipif_netif`];

    for(let num = 0; num < instances.length; num++) {
        let num_instance = instances[num];
        totalNumNetifs++;
    }
    return totalNumNetifs;
}

function getNetifConfig(instance, InstNum) {
    let driverVer = soc.getDriverVer("enet_cpsw");
    let instances;
    let module;

    instances = instance.netifInstance;
    module = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_lwipif_netif`];

    let cfgArray = new Array();


    for(let num = 0; num < instances.length; num++) {
        let num_instance = instances[num];
        let num_config = module.getInstanceConfig(num_instance)[`moduleInstance`];
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

function getDefaultNetifIdx(instance)
{
    let defaultNetifIdx = -1;

    for (let Idx = 0; Idx < getNetifCount(instance); Idx++)
    {
        if(getNetifConfig(instance, Idx).isDefault === true)
        {
            defaultNetifIdx = Idx;
            break;
        }
    }
    return defaultNetifIdx;
}

function getCpuID() {
    return system.getScript(`/drivers/soc/drivers_${common.getSocName()}`).getCpuID();
}

function validate(instance, report) {
    pktPoolScript.validate(instance, report);
    aleScript.validate(instance, report);
    mdioScript.validate(instance, report);
    macportScript.validate(instance, report);
    hostportScript.validate(instance, report);
    if (getNetifCount(instance) > 0)
    {
        if (getDefaultNetifCount(instance) !=1)
        {
            report.logError(`Only one netif can be set as default`, instance, "netifInstance");
        }

    }


    if (/^([0-9a-fA-F]{2}[:-]){5}[0-9a-fA-F]{2}(,([0-9a-fA-F]{2}[:-]){5}[0-9a-fA-F]{2})+/.test(instance.macAddrList) == false)
    {
        report.logError(`Invalid macAddrList Entry`, instance, "macAddrList");
    }

}

function moduleInstances(instance) {

    let Instances  = new Array();
    let driverVer = soc.getDriverVer("enet_cpsw");
    let maxCh     = 8;
    let maxNetif  = 2;

    Instances.push({
        name: "txDmaChannel",
        displayName: "ENET tx dma channel",
        moduleName: `/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_tx_channel`,
        useArray: true,
        minInstanceCount: 1,
        maxInstanceCount: maxCh,
        defaultInstanceCount: 1,
        collapsed:false,
        group: "cpdmaChConfig",
    });

    Instances.push({
        name: "rxDmaChannel",
        displayName: "ENET rx dma channel",
        moduleName: `/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_rx_channel`,
        useArray: true,
        minInstanceCount: 1,
        maxInstanceCount: maxCh,
        defaultInstanceCount: 1,
        collapsed:false,
        group: "cpdmaChConfig",
    });

    Instances.push({
        name: "pinmux",
        displayName: "CPSW pinmux",
        moduleName: `/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_pinmux`,
        useArray: true,
        minInstanceCount: 1,
        maxInstanceCount: 1,
        defaultInstanceCount: 1,
        collapsed:false,
        group: "pinmuxConfig",
    });

    Instances.push({
        name: "netifInstance",
        displayName: "NETIF instance",
        moduleName: `/networking/enet_cpsw/${driverVer}/enet_cpsw_lwipif_netif`,
        useArray: true,
        minInstanceCount: 0,
        maxInstanceCount: maxNetif,
        defaultInstanceCount: 0,
        collapsed:false,
        group: "lwipIfConfig",
    });

    return (Instances);
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

let enet_cpsw_module_name = "/networking/enet_cpsw/enet_cpsw";

let enet_cpsw_module = {

    displayName: "Enet (CPSW)",
	longDescription: "Driver for Common Port SWitch (CPSW). Support MAC, which is used in auto and industrial Ethernet to run TCP/IP, AVB etc. applications.",
    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: enet_cpsw_module_name,
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/networking/enet_cpsw/templates/enet_cpsw_v1.h.xdt",
            moduleName: enet_cpsw_module_name,
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: enet_cpsw_module_name,
        },
        "/board/board/board_open_close.c.xdt": {
            board_open: "/networking/enet_cpsw/templates/cpsw_board_open.c.xdt",
        },
        "/board/board/board_open_close.h.xdt": {
            board_open_close_config: "/networking/enet_cpsw/templates/cpsw_board_config.h.xdt",
        },
        "/board/board/board_config.h.xdt": {
            board_config: getBoardConfigTemplateInfo().Header,
            moduleName: enet_cpsw_module_name,
        },
        "/board/board/board_config.c.xdt": {
            board_config: getBoardConfigTemplateInfo().Cfile,
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_config.c.xdt": {
            enet_mem_config: "/networking/enet_cpsw/templates/enet_app_memutils_cfg_cpdma.c.xdt",
            enet_syscfg_info: "/networking/enet_cpsw/templates/enet_app_syscfg_info.c.xdt",
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_config.h.xdt": {
            enet_config: "/networking/enet_cpsw/templates/enet_syscfg.h.xdt",
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_open.c.xdt": {
            enet_open: "/networking/enet_cpsw/templates/enet_init.c.xdt",
            enet_init_config: "/networking/enet_cpsw/templates/enet_app_cpsw_cfg.c.xdt",
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_open.h.xdt": {
            enet_open: "/networking/enet_cpsw/templates/enet_init.h.xdt",
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_soc.c.xdt": {
            enet_soc: getSocConfigTemplateInfo().Cfile,
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_lwipif.c.xdt": {
            enet_lwipif: "/networking/enet_cpsw/templates/enet_lwipif.c.xdt",
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_lwipif.h.xdt": {
            enet_lwipif: "/networking/enet_cpsw/templates/enet_lwipif.h.xdt",
            moduleName: enet_cpsw_module_name,
        },
    },
    defaultInstanceName: "CONFIG_ENET_CPSW",
    config: [
        enet_cpsw_system_config,
        enet_cpsw_cpdma_channel_config,
        pktPoolScript.config,
        enet_cpsw_lwipIf_config,
        aleScript.config,
        mdioScript.config,
        hostportScript.config,
        macportScript.config,
        cptsScript.config,
        enet_cpsw_board_config,
        enet_cpsw_pinmux_config,
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
    utils: utilsScript,
    getInterfaceNameList,
    getPeripheralPinNames,
    getClockEnableIds,
    getClockFrequencies,
    getDmaInterface,
    getInstIdTable,
    getCpswInstInfo,
    getPhyMask,
    getBoardConfigTemplateInfo,
    getCpuID,
    getCpuInfo,
    getSocConfigTemplateInfo,
    getTxPacketsCount,
    getRxPacketsCount,
    getRxChannelCount,
    getTxChannelCount,
    getNumCpdmaDesc,
    getChannelConfig,
    getDefaultPacketCount,
    getNetifCount,
    getNetifConfig,
    getDefaultNetifIdx,
    validate: validate,
};

exports = enet_cpsw_module;
