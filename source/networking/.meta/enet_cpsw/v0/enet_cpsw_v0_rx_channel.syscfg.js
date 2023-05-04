
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
        {
            name: "useGlobalEvt",
            description: "Use the shared global event or dedicated event for the channel",
            displayName: "Use Shared Global Event",
            default: false,
        },
        {
            name: "useDefaultFlow",
            description: "The Rx Flow is the default flow",
            displayName: "Default Flow",
            default: true,
        },
        {
            name: "sizeThreshEn",
            description: "UDMAP receive flow packet size based free buffer queue enable configuration",
            displayName: "Rx Flow Size Threshold",
            default: 7,
            isInteger: true,
            range : [0,7],
            displayFormat: "hex",
        },
    ],
    getInstanceConfig,
};


exports = enet_cpsw_rx_ch_module;
