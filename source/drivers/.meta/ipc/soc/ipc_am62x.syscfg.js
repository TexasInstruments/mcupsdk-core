

let common = system.getScript("/common");


let config = [
    {
        name: "m4fss0_0",
        displayName: "M4FSS0",
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
    configurables.push (
        {
            name: "enableLinuxIpc",
            displayName: "Linux A53 IPC RP Message",
            description: `Enable IPC with Linux. Only IPC RP Message supported`,
            default: true,
            readOnly: true,
        }
    );

    return configurables;
}

function getSelfIpcCoreName()
{
    let cpuName = common.getSelfSysCfgCoreName();

    switch(cpuName) {
        default:
        case "m4fss0-0":
            return "m4fss0_0";
    }
}

function getSysCfgCoreName(ipcCoreName)
{
    switch(ipcCoreName) {
        default:
        case "m4fss0_0":
            return "m4fss0-0";
    }
}

function getMaxVringSize()
{
    /* The limit is determined by space set aside in OCRAM, but keep a reasonable default
     */
    return (48*1024);
}

function getImplementationVersion()
{
    return "v0";
}

exports = {
    getConfigurables,
    getSelfIpcCoreName,
    getSysCfgCoreName,
    getMaxVringSize,
    getImplementationVersion,
};
