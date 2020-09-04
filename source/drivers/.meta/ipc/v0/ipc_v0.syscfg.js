
let common = system.getScript("/common");

let ipc_soc = system.getScript(`/drivers/ipc/soc/ipc_${common.getSocName()}`)

let ipc_options = [
    {
        name: "NONE",
        displayName: "All IPC DISABLED",
    },
    {
        name: "notify",
        displayName: "IPC Notify ONLY",
    },
    {
        name: "notify_rpmsg",
        displayName: "IPC Notify + IPC RP Message",
    },
];

let ipc_options_default = "notify_rpmsg";

function getConfigurables()
{
    /* get 'CPU enable' configurables */
    let config = ipc_soc.getConfigurables();

    /* add defaults and options */
    config.forEach( function (element) {
        if(element.name != "enableLinuxIpc" && element.name != "enableMailboxIpc" ) {
            element.default = ipc_options_default;
            element.options = [ ...ipc_options ];
        }
        if((element.name == "a53ss0_0") && (getSelfIpcCoreName() != "a53ss0_0")) {
            element.default = "NONE";
        }
      });

    let hideVringConfig = false;
    /*
     * AM62x currently supports only LinuxIPC (M4 <---> A53), hide vringNumBuff, vringMsgSize and  vringSize
     * This can be removed once full RP Msg IPC is supported.
     */
    if (common.getSocName().match(/am62x/))
    {
        hideVringConfig = true;
    }
    /* to this add the configurable for RP Message buffer size and number */
    config.push(
        {
            name: "vringNumBuf",
            displayName: "RP Message Number of Buffers",
            description: `Number of message buffers in one RP Message VRING`,
            default: 8,
            options: [
                { name: 1},
                { name: 2},
                { name: 4},
                { name: 6},
                { name: 8},
                { name: 12},
                { name: 16},
            ],
            hidden: hideVringConfig,
        },
        {
            name: "vringMsgSize",
            displayName: "RP Message Buffer Size (Bytes)",
            description: `Size of one message buffer in a RP Message VRING`,
            longDescription:
                `Actual size available for user payload data is 16 bytes less than the value shown here`,
            default: 128,
            options: [
                { name: 64},
                { name: 128},
                { name: 256},
                { name: 512},
                { name: 1024},
                { name: 1152},
            ],
            hidden: hideVringConfig,
        },
    );

    /* create a instance like obj, so that we can get to the defaults as we would when inside onChange */
    const instanceLikeObj = _.reduce(config, (result, configurable) => {
        result[configurable.name] = configurable.default;
        return result;
    }, {});

    /* add a read only configurable to show the shared memory size based on current config set by user */
    config.push(
            {
                name: "vringSize",
                displayName: "RP Message Shared Memory (Bytes)",
                description: `Amount of shared memory needed for current user configuration`,
                default: getRPMessageVringSize(instanceLikeObj), /* set initial value based on defaults */
                readOnly: true,
                hidden: hideVringConfig,
            },
        );

    /* add onChange to each configurable in config, except the 'vringSize' since we update this inside onChange  */
    config.forEach( function (element) {
        if(element.name != "vringSize" && !element.readOnly)
            element.onChange = onChange;
      });

    return config;
}

function onChange(instance, ui)
{
    instance.vringSize = getRPMessageVringSize(instance);
    instance[getSelfIpcCoreName()] = "NONE";
    if(getEnabledCpus(instance).length>0)
    {
        instance[getSelfIpcCoreName()] = "notify";
    }
    if(getEnabledRPMessageCpus(instance).length>0 || instance.enableLinuxIpc === true)
    {
        instance[getSelfIpcCoreName()] = "notify_rpmsg";
    }
    if(getEnabledRPMessageCpus(instance).length>0)
    {
        ui.vringNumBuf.hidden = false;
        ui.vringMsgSize.hidden = false;
        ui.vringSize.hidden = false;
    }
    else
    {
        ui.vringNumBuf.hidden = true;
        ui.vringMsgSize.hidden = true;
        ui.vringSize.hidden = true;
    }
}

function getRPMessageVringSize(instance) {
    let enabledRPMessageCpus = getEnabledRPMessageCpus(instance);

    return (enabledRPMessageCpus.length+1) * (enabledRPMessageCpus.length+1 - 1)
        * ( instance.vringNumBuf * (instance.vringMsgSize + 32) + 32);
}


function getRPMessageVringRxTxMap(instance)
{
    let enabledCpus = [];
    let config = ipc_soc.getConfigurables();
    let rxTxMap = {};
    let vringId = 0;

    /* get names of RPMessage enabled CPUs, add self CPU to this list */
    for( let cpuConfig of config) {
        if(cpuConfig.name != "enableLinuxIpc") {
            if(instance[cpuConfig.name] == "notify_rpmsg" || cpuConfig.name == ipc_soc.getSelfIpcCoreName()) {
                enabledCpus.push(cpuConfig.name);
            }
        }
    }
    /* for each name, construct a N x N object mapping SRC CPU to DST CPU VRING ID,
       Assign VRING IDs to each SRC/DST pair, skip assignment when SRC == DST */
    for( let src of enabledCpus ) {
        rxTxMap[src] = {};
        for( let dst of enabledCpus ) {
            rxTxMap[src][dst] = -1;
            if(dst != src) { /* NO VRING for a CPU to itself */
                rxTxMap[src][dst] = vringId;
                vringId++;
            }
        }
    }
    return rxTxMap;
}

function getEnabledCpus(instance) {
    let enabledCpus = [];
    let config = ipc_soc.getConfigurables();

    for( let cpuConfig of config)
    {
        if(cpuConfig.name != "enableLinuxIpc" && cpuConfig.name != "enableMailboxIpc" ) {
            if(instance[cpuConfig.name] != "NONE" && cpuConfig.name != ipc_soc.getSelfIpcCoreName())
            {
                enabledCpus.push(cpuConfig.name);
            }
        }
    }
    return enabledCpus;
}

function getEnabledRPMessageCpus(instance) {
    let enabledCpus = [];
    let config = ipc_soc.getConfigurables();

    for( let cpuConfig of config)
    {
        if(cpuConfig.name != "enableLinuxIpc") {
            if(instance[cpuConfig.name] == "notify_rpmsg" && cpuConfig.name != ipc_soc.getSelfIpcCoreName())
            {
                enabledCpus.push(cpuConfig.name);
            }
        }
    }
    return enabledCpus;
}

function getSelfIpcCoreName() {
    return ipc_soc.getSelfIpcCoreName();
}

function getImplementationVersion() {
    return ipc_soc.getImplementationVersion();
}
function validate(instance, report) {

    /* multi-script validation  */
    let configs = ipc_soc.getConfigurables();
    let selfIpcCoreName = getSelfIpcCoreName();
    let ipc_module_name = '/drivers/ipc/ipc';

    common.validate.checkConfigurableValueMatchForAllCores(ipc_module_name, instance, report, "vringNumBuf");
    common.validate.checkConfigurableValueMatchForAllCores(ipc_module_name, instance, report, "vringMsgSize");

    for ( let config of configs)
    {
        if(config.name != "enableLinuxIpc" && config.name != "enableMailboxIpc" ) {
            let remoteCoreName = config.name;
            if( remoteCoreName != selfIpcCoreName)
            {
                let sysCfgCoreName = ipc_soc.getSysCfgCoreName(remoteCoreName)
                let remote_core_instance = common.getStaticModuleForCore(ipc_module_name, sysCfgCoreName);

                /* IPC option from self core to remote core and from remote core to self core should match */
                if(remote_core_instance && instance[remoteCoreName] != remote_core_instance[selfIpcCoreName])
                {
                    report.logError(`Value MUST match with core ${sysCfgCoreName}`, instance, remoteCoreName);
                }
            }
        }
    }

    if( instance.vringSize > ipc_soc.getMaxVringSize() )
    {
        report.logError(`RP Message Shared Memory size must be <= ${ipc_soc.getMaxVringSize()} bytes. Reduce RP Message number of buffers or buffer size`,
            instance, "vringSize" );

    }
}

let ipc_module = {
    displayName: "IPC",
    longDescription: `
IPC (Inter Processor Communication) provides a fast and low latency means to exchange
messages between applications on different CPUs on a SOC. There are two means to do IPC,

1. IPC Notify, here users can exchange small 28b messages, at exterme low latencies.
   This typically uses direct HW mechanism for message exchange, hence is very fast but also constrained
   on message size.
2. IPC RP Message, here users can exchange larger messages between multiple clients or tasks
   on different CPUs. This uses IPC Notify underneath with shared memory for message exchange.

When IPC RP Message is enabled, users can optional tune the memory requirements further by
choosing the values for size of message buffer that can be exchanged and depth of
message buffer SW queue, i.e how many messages can be outstanding until the SW queue becomes full.

VRING is the terminology to refer to a shared memory SW queue that is
used to exchange messages between two CPUs.

Total shared memory needed for IPC RPMessage VRING's is,

    ( number of cpus ) x ( number of cpus  - 1)                  // number of VRINGs
    x ( number of message buffers x (message buffer size + 32 bytes) + 32 ) // size of 1x VRING

Smaller the number of message buffers, smaller the size of message buffers, and lesser the number of CPUs
enabled for IPC RP Message, lesser the shared memory that is required.
        `,
    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/ipc/templates/ipc_v0_config.c.xdt",
            driver_init: "/drivers/ipc/templates/ipc_v0_init.c.xdt",
            driver_deinit: "/drivers/ipc/templates/ipc_v0_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/ipc/templates/ipc_v0.h.xdt",
        },
    },
    defaultInstanceName: "CONFIG_IPC",

    moduleStatic : {

        config: getConfigurables(),
        validate,

        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },

    },

    getSelfIpcCoreName,
    getRPMessageVringRxTxMap,
    getRPMessageVringSize,
    getEnabledCpus,
    getEnabledRPMessageCpus,
    getImplementationVersion,
};

exports = ipc_module;
