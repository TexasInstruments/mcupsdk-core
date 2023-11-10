

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
    {
        name: "m4fss0_0",
        displayName: "M4FSS0",
    },
    {
        name: "a53ss0_0",
        displayName: "A53SS0 Core 0"
    }
];

function getConfigurables() {

    let configurables = [];
    let cpuConfig1, cpuConfig2;

    for( let cpuConfig of config)
    {
        cpuConfig1 = _.cloneDeep(cpuConfig);
        cpuConfig2 = _.cloneDeep(cpuConfig);

        /* mark self CPU with the text 'self' and make it read only */
        if(cpuConfig1.name == getSelfIpcCoreName())
        {
            cpuConfig1.displayName += " (self)";
            cpuConfig1.readOnly = true;
            cpuConfig1.description = "CPU on which this application is running";
        }
        configurables.push(cpuConfig1);
        
        if(cpuConfig2.name != getSelfIpcCoreName())
        {
            cpuConfig2.name += "_safeipc";
            cpuConfig2.displayName += " SafeIPC";
            cpuConfig2.description = "Enable Safe IPC with remote core.";
            cpuConfig2.default = false;
            cpuConfig2.hidden = true;
            configurables.push(cpuConfig2);
        }
    }
    configurables.push (
        {
            name: "enableLinuxIpc",
            displayName: "Linux A53 IPC RP Message",
            description: `Enable IPC with Linux. Only IPC RP Message supported`,
            default: false,
        }
    );

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
        case "m4fss0-0":
            return "m4fss0_0";
        case "a53ss0-0":
            return "a53ss0_0";
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
        case "m4fss0_0":
            return "m4fss0-0";
        case "a53ss0_0":
            return "a53ss0-0";
    }
}

function getIpcSharedMemAvailable()
{
    /* The limit is determined by space set aside in OCRAM, but keep a reasonable default
     */
    return (48*1024);
}

function getImplementationVersion()
{
    return "v0";
}

function getFirewallGranularity()
{
    return 4096;
}

function getSharedMemAddress()
{
    return 0x701D4000;
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
