
let common = system.getScript("/common");

function getInstanceConfig(moduleInstance) {
    return {
        ...moduleInstance,
    };
};



let enet_icssg_tx_ch_module = {
    displayName: "Tx Channel config",
    longDescription: `This adds and configures a Enet LLD Tx DMA channel.`,
    alwaysShowLongDescription: false,
    defaultInstanceName: "ENET_DMA_TX_CH",
    config: [
        {
            name: "PacketsCount",
            description: "No of packets required for Tx DMA channel",
            displayName: "Number Of Packets",
            default: 16,
            isInteger:true,
        },
        {
            name: "useGlobalEvt",
            description: "Use the shared global event or dedicated event for the channel",
            displayName: "Use Shared Global Event",
            default: true,
        },
    ],
    getInstanceConfig,
};


exports = enet_icssg_tx_ch_module;
