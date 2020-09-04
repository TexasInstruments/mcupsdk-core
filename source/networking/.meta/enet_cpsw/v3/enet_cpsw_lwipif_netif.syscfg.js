let common = system.getScript("/common");

let module = system.modules["/networking/enet_cpsw/enet_cpsw"];

function getInstanceConfig(moduleInstance) {
    return {
        ...moduleInstance,
    };
};

let enet_cpsw_lwipif_netif_module = {
    displayName: "Netif configuration",
    longDescription: `This adds and configures netif for lwip interface.`,
    alwaysShowLongDescription: false,
    defaultInstanceName: "NETIF_INST_ID",
    config: [
        {
            name: "isDefault",
            description: "Set this netif to default Netif",
            displayName: "Set As Default Netif",
            default: true,
        },
    ],
    getInstanceConfig,
};


exports = enet_cpsw_lwipif_netif_module;
