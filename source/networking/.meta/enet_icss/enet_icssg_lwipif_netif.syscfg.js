
let common = system.getScript("/common");

let module = system.modules["/networking/enet_icss/enet_icss"];

function getInstanceConfig(moduleInstance) {
    return {
        ...moduleInstance,
    };
};

function getDefaultNetifCount()
{
    let defaultNetifCount = 0;
    for(let i=0; i < module.$instances.length; i++) {
        let instance = module.$instances[i];
        for (let Idx = 0; Idx < module.getNetifCount(instance); Idx++)
        {
            defaultNetifCount += (module.getNetifConfig(instance, Idx).isDefault === true) ? 1 : 0;
        }
    }
    return defaultNetifCount;
}


function getDefaultNetifIndex()
{
    let defaultNetifIdx = -1;
    for(let i=0; i < module.$instances.length; i++) {
        let instance = module.$instances[i];
        for (let Idx = 0; Idx < module.getNetifCount(instance); Idx++)
        {
            if(module.getNetifConfig(instance, Idx).isDefault === true)
            {
                defaultNetifIdx = Idx;
                break;
            }
        }
    }
    return defaultNetifIdx;
}

function netif_validate(instance, report)
{
    if (instance.mode === "SWITCH")
    {
        if (getDefaultNetifCount() > 1)
        {
            report.logError(`Switch case should have only one default netif`, instance, "netifInstance");
        }
    }
    if (instance.mode === "DUAL MAC")
    {
        if (getDefaultNetifCount() > 1)
        {
            report.logError(`DUAL MAC case should have only one default netif`, instance, "netifInstance");
        }
    }
}

let enet_icssg_lwipif_netif_module = {
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
    getDefaultNetifCount,
    getDefaultNetifIndex,
    validate: netif_validate,
};


exports = enet_icssg_lwipif_netif_module;
