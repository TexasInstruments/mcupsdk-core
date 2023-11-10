

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
    /* size of MSS mailbox memory - space reserved for SW queues (32B per SW queue),
     * right now assume all SW queus are in MSS mailbox memory area
     * 3 CPUs = 3 x (3-1) = 6 SW queues
     * Each SW queue size is 32 bytes
     */
    return ((8*1024)-(32*6));
}

function getImplementationVersion()
{
    return "v1";
}

function getIpcStaticMemory()
{
    var str = [
    "/* Dedicated mailbox memories address and size */",
    "#define MSS_MBOX_MEM                (CSL_MSS_MBOX_U_BASE)",
    "#define MSS_MBOX_MEM_SIZE           (8U*1024U)",
    "",
    "/*",
    "* SW queue between each pair of CPUs",
    "*",
    "* place SW queues at the bottom of the dedicated mailbox memories.",
    "* Driver assume this memory is init to zero in bootloader as it's ECC protected and",
    "* needs to be intialized only once and to ensure that only one core has done the",
    "* mailbox ram initialization before ipc_init. If SBL is not used then Gel does the initialization.",
    "* We need 4 SW Q's for the 2x R5F to send messages to C66SS0 and each other, i.e 128 B",
    "* and we need 2 SW Q's for C66SS0 to send messages to each R5F, i.e 64 B.",
    "*",
    "* Rest of the mailbox memory can be used for ipc_rpmessage or custom message passing.",
    "*/",
    "#define C66SS0_TO_R5FSS0_0_SW_QUEUE        (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*6U))",
    "#define C66SS0_TO_R5FSS0_1_SW_QUEUE        (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*5U))",
    "#define R5FSS0_1_TO_R5FSS0_0_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*4U))",
    "#define R5FSS0_1_TO_C66SS0_SW_QUEUE        (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*3U))",
    "#define R5FSS0_0_TO_R5FSS0_1_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*2U))",
    "#define R5FSS0_0_TO_C66SS0_SW_QUEUE        (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*1U))",
    ].join("\n");

    return str;
}

function getFirewallGranularity()
{
    return 1024;
}

function getSharedMemAddress()
{
    return 0xC5000000;
}

exports = {
    getConfigurables,
    getSelfIpcCoreName,
    getSysCfgCoreName,
    getIpcSharedMemAvailable,
    getImplementationVersion,
    getFirewallGranularity,
    getSharedMemAddress,
    getIpcStaticMemory,
};
