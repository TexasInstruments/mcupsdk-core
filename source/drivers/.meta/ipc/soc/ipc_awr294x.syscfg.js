

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
        name: "c66ss0",
        displayName: "C66 Core 0",
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

    let mailboxConfig = {};

    mailboxConfig.name = "enableMailboxIpc";
    mailboxConfig.displayName = "Enable Mailbox IPC With RSS R4";
    mailboxConfig.readOnly = false;
    mailboxConfig.description = "IPC with RSS R4 and this CPU using the 'mailbox' driver API";
    mailboxConfig.default = true;

    configurables.push(mailboxConfig);

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
        case "c66ss0":
            return "c66ss0";
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
        case "c66ss0":
            return "c66ss0";
    }
}

function getMaxVringSize()
{
    /* size of MSS mailbox memory - space reserved for SW queues (32B per SW queue),
     * right now assume all SW queus are in MSS mailbox memory area
     * 3 CPUs = 3 x (3-1) = 6 SW queues
     * Each SW queue size is 32 bytes
     */
    return ( (8*1024)-(32*6) );
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
