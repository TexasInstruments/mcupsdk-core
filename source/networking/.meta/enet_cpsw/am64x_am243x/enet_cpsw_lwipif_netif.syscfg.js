let common = system.getScript("/common");

let module = system.modules["/networking/enet_cpsw/enet_cpsw"];

function getInstanceConfig(moduleInstance) {
    return {
            moduleInstance,
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
            let matchedEntry = module.getCpswInstInfo(instance);
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
        let matchedEntry = module.getCpswInstInfo(instance);
        ret += '{' + matchedEntry.enetType + ', ' +  matchedEntry.instId + ',' + module.getChannelConfig(instance, "RX", 0).$name.toUpperCase()  + ',' + 1 + '},';
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
        let matchedEntry = module.getCpswInstInfo(instance);
        ret += '{' + matchedEntry.enetType + ', ' +  matchedEntry.instId + ',' + module.getChannelConfig(instance, "TX", 0).$name.toUpperCase()  + ',' + 1 + '},';
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
     for(let i = 0; i < module.$instances.length; i++)
     {
        for (let Idx = 0; Idx < module.getNetifCount(module.$instances[i]); Idx++)
        {
            let rxCh = module.getNetifConfig(module.$instances[i],Idx).rxDmaChNum;
            for (let j = 0; j < rxCh.length; j++)
            {
               if( rxCh[j] >= module.getRxChannelCount(module.$instances[i]) )
               {
                    report.logError(`Incorrect Rx channel`, instance);
               }
            }
            
            let txCh = module.getNetifConfig(module.$instances[i],Idx).txDmaChNum;
            for (let j = 0; j < txCh.length; j++)
            {
               if( txCh[j] >= module.getTxChannelCount(module.$instances[i]) )
               {
                    report.logError(`Incorrect Tx channel`, instance);
               }
            }
        }
     }
}

function getNetifRxCh()
{
    var ret = '{';
    for(let i = 0; i < module.$instances.length; i++)
     {
        let instance = module.$instances[i];
        for (let Idx = 0; Idx < module.getNetifCount(instance); Idx++)
        {
            let rxCh = module.getNetifConfig(instance,Idx).rxDmaChNum
            
            ret += '{' + module.getChannelConfig(instance, "RX", rxCh[0]).$name.toUpperCase() + ', ';
            if (rxCh.length == 2)
            {
                ret += module.getChannelConfig(instance, "RX", rxCh[1]).$name.toUpperCase() + ',' + '},';
            }
            else
            {
                ret += -1 + ',' + '},'
            }
        }
     }
    ret += '}';
    return ret;
}

function getNetifTxCh()
{
    var ret = '{';
    for(let i = 0; i < module.$instances.length; i++)
     {
        let instance = module.$instances[i];
        for (let Idx = 0; Idx < module.getNetifCount(instance); Idx++)
        {
            let txCh = module.getNetifConfig(instance,Idx).txDmaChNum
            
            ret += '{' + module.getChannelConfig(instance, "TX", txCh[0]).$name.toUpperCase() + '},';
            
        }
     }
    ret += '}';
    return ret;
}

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
        {
            name: "rxDmaChNum",
            description: "Rx DMA used by this Netif",
            displayName: "Rx DMA Used By This Netif",
            default: Array.from(Array(1).keys()).map(String),
            minSelections: 0,
            options: _.keys(Array(16)).map((index)=>({name: index})),
        },
        {
            name: "txDmaChNum",
            description: "Tx DMA used by this Netif",
            displayName: "Tx DMA Used By This Netif",
            default: Array.from(Array(1).keys()).map(String),
            minSelections: 0,
            options: _.keys(Array(16)).map((index)=>({name: index})),
        },
    ],
    getTotalNetIfCount,
    getInstanceConfig,
    getDefaultNetifCount,
    getDefaultNetifIndex,
    getNetifIdx2EnetMap,
    getEnet2RxChIdMap,
    getEnet2TxChIdMap,
    getNetifRxCh,
    getNetifTxCh,
    validate: validate,
};


exports = enet_cpsw_lwipif_netif_module;
