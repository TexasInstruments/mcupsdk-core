    
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
    for(let i = 0; i < module.$instances.length; i++) {
        let instance = module.$instances[i];
        for (let Idx = 0; Idx < module.getNetifCount(instance); Idx++)
        {
            defaultNetifCount += (module.getNetifConfig(instance, Idx).isDefault === true) ? 1 : 0;
        }
    }
    return defaultNetifCount;
}

function getTotalNetIfCount(instances)
{
    let count = 0;
    for(let inst in instances)
    {
        count += module.getNetifCount(instances[inst]);
    }
    return count;
}

function getDefaultNetifIndex()
{
    let defaultNetifIdx = '';
    for (let i = 0; i < module.$instances.length; i++) {
        let instance = module.$instances[i];
        for (let Idx = 0; Idx < module.getNetifCount(instance); Idx++)
        {
            if (module.getNetifConfig(instance, Idx).isDefault === true)
            {
                defaultNetifIdx = module.getNetifConfig(instance, Idx).$name;
                break;
            }
        }
        if (defaultNetifIdx != '')
        {
            break;
        }
    }
    return defaultNetifIdx;
}

function getNetifIdx2EnetMap()
{
    var idx = 0;
    var ret = '{';
    for(let i = 0; i < module.$instances.length; i++) {
        let instance = module.$instances[i];
        for (let Idx = 0; Idx < module.getNetifCount(instance); Idx++)
        {
            let matchedEntry = module.getInstId(instance);
            ret += '{' + matchedEntry.enetType + ', ' +  matchedEntry.instId + '},';
        }
    }
    ret += '}'
    return ret;
}

function getEnet2RxChIdMap()
{
    var idx = 0;
    var ret = '{';
    for(let i = 0; i < module.$instances.length; i++) {
        let instance = module.$instances[i];
        let matchedEntry = module.getInstId(instance);
        ret += '{' + matchedEntry.enetType + ', ' +  matchedEntry.instId + ',' + module.getChannelConfig(instance, "RX", 0).$name.toUpperCase()  + ',' + module.getRxChannelCount(instance) + '},';
    }
    ret += '}'
    return ret;
}

function getEnet2TxChIdMap()
{
    var idx = 0;
    var ret = '{';
    for(let i = 0; i < module.$instances.length; i++) {
        let instance = module.$instances[i];
        let matchedEntry = module.getInstId(instance);
        ret += '{' + matchedEntry.enetType + ', ' +  matchedEntry.instId + ',' + module.getChannelConfig(instance, "TX", 0).$name.toUpperCase()  + ',' + module.getTxChannelCount(instance) + '},';
    }
    ret += '}'
    return ret;
}

function validate(instance, report)
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
    getTotalNetIfCount,
    getInstanceConfig,
    getDefaultNetifCount,
    getDefaultNetifIndex,
    getNetifIdx2EnetMap,
    getEnet2RxChIdMap,
    getEnet2TxChIdMap,
    validate: validate,
};


exports = enet_icssg_lwipif_netif_module;
