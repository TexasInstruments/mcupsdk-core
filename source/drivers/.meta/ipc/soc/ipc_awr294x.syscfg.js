

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
    mailboxConfig.readOnly = true;
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

function getIpcSharedMemAvailable()
{
    /* Size of MSS mailbox memory */
    return ((8*1024) - 512 );
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
    return 0xC5000200;
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
