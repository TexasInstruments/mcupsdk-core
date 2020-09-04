

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

function getMaxVringSize()
{
    /* size of MSS mailbox memory - space reserved for SW queues (32B per SW queue),
     * right now assume all SW queus are in MSS mailbox memory area
     * 4 CPUs = 4 x (4-1) = 12 SW queues
     * Each SW queue size is 32 bytes
     */
    return ( (16*1024)-(32*12) );
}

function getImplementationVersion()
{
    return "v1";
}

exports = {
    getConfigurables,
    getSelfIpcCoreName,
    getSysCfgCoreName,
    getMaxVringSize,
    getImplementationVersion,
};
