

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

function getIpcStaticMemory()
{
    var str = [
    "/* Dedicated mailbox memories address and size */",
    "#define MSS_MBOX_MEM                (CSL_MBOX_SRAM_U_BASE)",
    "#define MSS_MBOX_MEM_SIZE           (16U*1024U)",
    "",
    "/*",
    "* SW queue between each pair of CPUs",
    "*",
    "* place SW queues at the bottom of the dedicated mailbox memories.",
    "* Driver assume this memory is init to zero in bootloader as it's ECC protected and",
    "* needs to be intialized only once and to ensure that only one core has done the",
    "* mailbox ram initialization before ipc_init. If SBL is not used then Gel does the initialization.",
    "* We need 4*3 SW Q's for the 4x R5F to send messages to each other, i.e 384 B.",
    "*",
    "* Rest of the mailbox memory cna be used for ipc_rpmessage or custom message passing.",
    "*/",
    "#define R5FSS0_0_TO_R5FSS0_1_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*12U))",
    "#define R5FSS0_0_TO_R5FSS1_0_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*11U))",
    "#define R5FSS0_0_TO_R5FSS1_1_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*10U))",
    "#define R5FSS0_1_TO_R5FSS0_0_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*9U))",
    "#define R5FSS0_1_TO_R5FSS1_0_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*8U))",
    "#define R5FSS0_1_TO_R5FSS1_1_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*7U))",
    "#define R5FSS1_0_TO_R5FSS0_0_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*6U))",
    "#define R5FSS1_0_TO_R5FSS0_1_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*5U))",
    "#define R5FSS1_0_TO_R5FSS1_1_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*4U))",
    "#define R5FSS1_1_TO_R5FSS0_0_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*3U))",
    "#define R5FSS1_1_TO_R5FSS0_1_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*2U))",
    "#define R5FSS1_1_TO_R5FSS1_0_SW_QUEUE      (IpcNotify_SwQueue*)((MSS_MBOX_MEM + MSS_MBOX_MEM_SIZE) - (MAILBOX_MAX_SW_QUEUE_SIZE*1U))",
    ].join("\n");

    return str;
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
    getIpcStaticMemory,
};
