
let common = system.getScript("/common");

function getInstanceConfig(moduleInstance) {
    return {
        ...moduleInstance,
    };
};



let enet_cpsw_tx_ch_module = {
    displayName: "Tx Channel Config",
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
    ],
    getInstanceConfig,
};


exports = enet_cpsw_tx_ch_module;
