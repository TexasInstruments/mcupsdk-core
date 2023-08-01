

let common = system.getScript("/common");


let config = [
    {
        name: "r5fss0_0",
        displayName: "R5FSS0 Core 0",
    },
    {
        name: "r5fss0_1",
        displayName: "R5FSS0 Core 1",
    },
    {
        name: "r5fss1_0",
        displayName: "R5FSS1 Core 0",
    },
    {
        name: "r5fss1_1",
        displayName: "R5FSS1 Core 1",
    },
];

function getConfigurables() {

    let configurables = [];

    for( let cpuConfig of config)
    {
        if(cpuConfig.name == getSelfIpcCoreName())
        {
            /* mark self CPU with the text 'self' and make it read only */
            cpuConfig = _.cloneDeep(cpuConfig);

            cpuConfig.displayName += " (self)";
            cpuConfig.readOnly = true;
            cpuConfig.description = "CPU on which this application is running";
        }
        configurables.push(cpuConfig);
    }
    return configurables;
}

function getSelfIpcCoreName()
{
    let cpuName = common.getSelfSysCfgCoreName();

    switch(cpuName) {
        default:
        case "r5fss0-0":
            return "r5fss0_0";
        case "r5fss0-1":
            return "r5fss0_1";
        case "r5fss1-0":
            return "r5fss1_0";
        case "r5fss1-1":
            return "r5fss1_1";
    }
}

function getSysCfgCoreName(ipcCoreName)
{
    switch(ipcCoreName) {
        default:
        case "r5fss0_0":
            return "r5fss0-0";
        case "r5fss0_1":
            return "r5fss0-1";
        case "r5fss1_0":
            return "r5fss1-0";
        case "r5fss1_1":
            return "r5fss1-1";
    }
}

function getIpcSharedMemAvailable()
{
    /* Size of MSS mailbox memory */
    return (16*1024);
}

function getImplementationVersion()
{
    return "v1";
}

function getFirewallGranularity()
{
    return 1024;
}

function getSharedMemAddress()
{
    return 0x72000000;
}

exports = {
    getConfigurables,
    getSelfIpcCoreName,
    getSysCfgCoreName,
    getIpcSharedMemAvailable,
    getImplementationVersion,
    getFirewallGranularity,
    getSharedMemAddress,
};
