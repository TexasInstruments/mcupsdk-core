
let common = system.getScript("/common");

function getInstanceConfig(moduleInstance) {
    return {
        ...moduleInstance,
    };
};



let enet_cpsw_rx_ch_module = {
    displayName: "ENET DMA Rx Channel Configuration",
    longDescription: `This adds and configures a Enet LLD Rx DMA channel.`,
    alwaysShowLongDescription: false,
    defaultInstanceName: "ENET_DMA_RX_CH",
    config: [
        {
            name: "PacketsCount",
            description: "No of packets required for Rx DMA channel",
            displayName: "Number Of Packets",
            default: 32,
            isInteger:true,
        },
        {
            name: "macAddrCount",
            description: "Number of MAC address required for this Rx DMA channle",
            displayName: "Number of MAC Address",
            default:1,
            isInteger:true,
            range : [0,2],
        },
    ],
    getInstanceConfig,
};


exports = enet_cpsw_rx_ch_module;
