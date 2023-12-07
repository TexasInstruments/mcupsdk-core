
let common = system.getScript("/common");
let hwi = system.getScript("/kernel/dpl/hwi.js");
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
    let hideVringConfig = false;
    let hideSafeIpcConfig = false;
    let hideCrcConfig = false;

    /*
     * AM62x currently supports only LinuxIPC (M4 <---> A53), hide vringNumBuff and vringMsgSize
     * This can be removed once full RP Msg IPC is supported.
     */
    if (common.getSocName().match(/am62x/))
    {
        hideVringConfig = true;
        hideSafeIpcConfig = true;
        hideCrcConfig = true;
    }

    /* add defaults and options */
    config.forEach( function (element) {
        if(element.name != "enableLinuxIpc" && element.name != "enableMailboxIpc" && element.name.search("_safeipc") == -1) {
            element.default = ipc_options_default;
            element.options = [ ...ipc_options ];
        }
        if((element.name == "a53ss0_0") && (getSelfIpcCoreName() != "a53ss0_0")) {
            element.default = "NONE";
        }
      });

    /* to this add the configurable for RP Message buffer size and number */
    config.push(
        {
            name: "intrPriority",
            displayName: "Interrupt Priority",
            default: hwi.getHwiDefaultPriority(),
            hidden: !hwi.getPriorityConfigSupported(),
            description: `Interrupt Priority: 0 (highest) to ${hwi.getHwiMaxPriority()} (lowest)`,
        },
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
        /* Advanced parameters */
        {
            name: "enableSafeIpc",
            displayName: "Enable SafeIPC",
            description: `This enables SafeIPC configuration. If enabled, the shared memory is organized in a way that enables firewalling to prevent unwanted access.`,
            default: false,
            hidden: hideSafeIpcConfig,
        },
        {
            name: "enableCrc",
            displayName: "Enable CRC",
            description: `This enables CRC check for IPC data.`,
            default: false,
            hidden: hideCrcConfig,
        },
        {
            name: "crcHookFxn",
            displayName: "CRC Hook Function",
            default: "NULL",
            hidden: true,
            description: "Hook function in application for CRC calculation.",
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
                name: "sharedMemAvailable",
                displayName: "IPC Shared Memory Available (Bytes)",
                description: `Amount of shared memory available for user configuration`,
                default: ipc_soc.getIpcSharedMemAvailable(), /* set initial value based on defaults */
                readOnly: true,
            },
            {
                name: "sharedMemUsed",
                displayName: "IPC Shared Memory Used (Bytes)",
                description: `Amount of shared memory needed for current user configuration`,
                default:0,
                getValue: function (inst, ui) {
                    return getIpcSharedMemData(inst).sharedMemUsed;
                },
                readOnly: true,
            },
        );

    /* add onChange to each configurable in config, except the 'sharedMemUsed' since we update this inside onChange and 'sharedMemAvailable' since it's fixed */
    config.forEach( function (element) {
        if(element.name != "sharedMemUsed" && element.name != "sharedMemAvailable" && element.name != "crcHookFxn" && !element.readOnly)
            element.onChange = onChange;
      });

    return config;
}

function onChange(instance, ui)
{
    instance[getSelfIpcCoreName()] = "NONE";
    if(getEnabledCpus(instance).length>0 || instance.enableMailboxIpc === true)
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
        if(getImplementationVersion() == "v0")
        {
            ui.sharedMemUsed.hidden = false;
            ui.sharedMemAvailable.hidden = false;
            if (!common.getSocName().match(/am62x/))
            {
                ui.enableSafeIpc.hidden = false;
            }
        }
    }
    else
    {
        ui.vringNumBuf.hidden = true;
        ui.vringMsgSize.hidden = true;
        if(getImplementationVersion() == "v0")
        {
            ui.sharedMemUsed.hidden = true;
            ui.sharedMemAvailable.hidden = true;
            ui.enableSafeIpc.hidden = true;
        }
    }
    let config = ipc_soc.getConfigurables();
    for(let element of config)
    {
        if(element.name.search("_safeipc") != -1)
        {
            if(instance.enableSafeIpc == true) {
                ui[element.name].hidden = false;
            }
            else {
                ui[element.name].hidden = true;
                instance[element.name] = false;
            }
        }
    }
    if(instance.enableCrc)
    {
        ui.crcHookFxn.hidden = false;
    }
    else
    {
        ui.crcHookFxn.hidden = true;
    }
}

function getIpcStaticMemory()
{
    return ipc_soc.getIpcStaticMemory();
}

function getIpcSharedMemData(instance) {
    let sharedMemSize = 0;
    let configs = ipc_soc.getConfigurables();
    let ipc_module_name = '/drivers/ipc/ipc';
    let vringSize = (instance.vringNumBuf * (instance.vringMsgSize + 32)) + 32;
    let ipcSize = 0, firewallSize = 0, vringCount = 0, notifyCount = 0, offset = 0, startAddrOffset = 0;
    let firewallGranularity = ipc_soc.getFirewallGranularity();
    let ipcInfo = {};
    let ipcNotifyOffsetArr = {};
    let ipcRpMsgOffsetArr = {};
    let ipcFirewallAddress = [];
    /* The core which should display the firewall configuration info */
    let firewallCore = "NULL";
    let safeIpcEnabled = false;

    if(getImplementationVersion() == "v0")
    {
        /* Create 2D array to hold the VRING addresses */
        for ( let config of configs)
        {
            if(config.name != "enableLinuxIpc" && config.name != "enableMailboxIpc" && config.name.search("_safeipc") == -1) {
                let remoteCoreName = config.name;
                let sysCfgCoreName = ipc_soc.getSysCfgCoreName(remoteCoreName)
                let remote_core_instance = common.getStaticModuleForCore(ipc_module_name, sysCfgCoreName);

                if(remote_core_instance)
                {
                    ipcRpMsgOffsetArr[remoteCoreName] = {};
                }
            }
        }
        for ( let config of configs)
        {
            if(config.name != "enableLinuxIpc" && config.name != "enableMailboxIpc" && config.name.search("_safeipc") == -1) {
                let remoteCoreName = config.name;
                let sysCfgCoreName = ipc_soc.getSysCfgCoreName(remoteCoreName)
                let remote_core_instance = common.getStaticModuleForCore(ipc_module_name, sysCfgCoreName);

                if(remote_core_instance)
                {
                    /* Firewall config info is display in the first core that has IPC enabled */
                    if(firewallCore == "NULL")
                    {
                        firewallCore = remoteCoreName;
                    }
                    for ( let config1 of configs)
                    {
                        if(config1.name != "enableLinuxIpc" && config1.name != "enableMailboxIpc" && config1.name.search("_safeipc") == -1) {
                            let remoteCoreName1 = config1.name;
                            let sysCfgCoreName1 = ipc_soc.getSysCfgCoreName(remoteCoreName1)
                            let remote_core_instance1 = common.getStaticModuleForCore(ipc_module_name, sysCfgCoreName1);
                            let safeIpcConfig = "", safeIpcConfig1 = "";

                            if(remote_core_instance1)
                            {
                                if((remote_core_instance[remoteCoreName1] == "notify_rpmsg") && (remoteCoreName1 != remoteCoreName) && (configs.indexOf(config1) > configs.indexOf(config)))
                                {
                                    ipcSize = vringSize * 2;
                                    ipcRpMsgOffsetArr[remoteCoreName][remoteCoreName1] = (vringSize * vringCount) + offset;
                                    ipcRpMsgOffsetArr[remoteCoreName1][remoteCoreName] = ipcRpMsgOffsetArr[remoteCoreName][remoteCoreName1] + vringSize ;
                                    vringCount += 2;

                                    safeIpcConfig = remoteCoreName1 + "_safeipc";
                                    safeIpcConfig1 = remoteCoreName + "_safeipc";

                                    if(remote_core_instance[safeIpcConfig] || remote_core_instance1[safeIpcConfig1])
                                    {
                                        /* Align Start address to firewall granularity */
                                        if(ipcRpMsgOffsetArr[remoteCoreName][remoteCoreName1] % firewallGranularity != 0)
                                        {
                                            startAddrOffset = firewallGranularity - (ipcRpMsgOffsetArr[remoteCoreName][remoteCoreName1] % firewallGranularity);
                                            ipcRpMsgOffsetArr[remoteCoreName][remoteCoreName1] += startAddrOffset;
                                            ipcRpMsgOffsetArr[remoteCoreName1][remoteCoreName] += startAddrOffset;
                                            sharedMemSize += startAddrOffset;
                                            offset += startAddrOffset;
                                        }

                                        /* Align IPC data size to firewall granularity */
                                        firewallSize = firewallGranularity * Math.trunc(ipcSize / firewallGranularity);
                                        if((ipcSize < firewallGranularity) || (ipcSize % firewallGranularity != 0)) {
                                            firewallSize += firewallGranularity;
                                        }
                                        offset += firewallSize - ipcSize;
                                        sharedMemSize += firewallSize;

                                        ipcFirewallAddress.push(
                                            {
                                                startAddress: ipc_soc.getSharedMemAddress() + ipcRpMsgOffsetArr[remoteCoreName][remoteCoreName1],
                                                endAddress: ipc_soc.getSharedMemAddress() + ipcRpMsgOffsetArr[remoteCoreName][remoteCoreName1] + firewallSize - 1,
                                                cores: remoteCoreName + ", " + remoteCoreName1,
                                            }
                                        );
                                    }
                                    else
                                    {
                                        sharedMemSize += ipcSize;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        for ( let config of configs)
        {
            if(config.name != "enableLinuxIpc" && config.name != "enableMailboxIpc" && config.name.search("_safeipc") == -1) {
                let remoteCoreName = config.name;
                let sysCfgCoreName = ipc_soc.getSysCfgCoreName(remoteCoreName)
                let remote_core_instance = common.getStaticModuleForCore(ipc_module_name, sysCfgCoreName);

                if(remote_core_instance)
                {
                    /* Check if SafeIPC is enabled in any core for backward compatibility */
                    if(remote_core_instance["enableSafeIpc"])
                    {
                        safeIpcEnabled = true;
                    }

                    /* Create 2D array to hold the VRING addresses */
                    ipcNotifyOffsetArr[remoteCoreName] = {};
                    ipcRpMsgOffsetArr[remoteCoreName] = {};
                }
            }
        }
        for ( let config of configs)
        {
            if(config.name != "enableLinuxIpc" && config.name != "enableMailboxIpc" && config.name.search("_safeipc") == -1) {
                let remoteCoreName = config.name;
                let sysCfgCoreName = ipc_soc.getSysCfgCoreName(remoteCoreName)
                let remote_core_instance = common.getStaticModuleForCore(ipc_module_name, sysCfgCoreName);

                if(remote_core_instance)
                {
                    /* Firewall config info is display in the first core that has IPC enabled */
                    if(firewallCore == "NULL")
                    {
                        firewallCore = remoteCoreName;
                    }
                    for ( let config1 of configs)
                    {
                        if(config1.name != "enableLinuxIpc" && config1.name != "enableMailboxIpc" && config1.name.search("_safeipc") == -1) {
                            let remoteCoreName1 = config1.name;
                            let sysCfgCoreName1 = ipc_soc.getSysCfgCoreName(remoteCoreName1)
                            let remote_core_instance1 = common.getStaticModuleForCore(ipc_module_name, sysCfgCoreName1);
                            let safeIpcConfig = "", safeIpcConfig1 = "";

                            if(remote_core_instance1)
                            {
                                if((remoteCoreName1 != remoteCoreName) && (remote_core_instance[remoteCoreName1] != "NONE"))
                                {
                                    if(safeIpcEnabled == true)
                                    {
                                        if(configs.indexOf(config1) > configs.indexOf(config))
                                        {
                                            ipcSize = 32 * 2;
                                            ipcNotifyOffsetArr[remoteCoreName][remoteCoreName1] = (32 * notifyCount) + (vringSize * vringCount) + offset;
                                            ipcNotifyOffsetArr[remoteCoreName1][remoteCoreName] = ipcNotifyOffsetArr[remoteCoreName][remoteCoreName1] + 32;
                                            notifyCount += 2;

                                            if(remote_core_instance[remoteCoreName1] == "notify_rpmsg")
                                            {
                                                ipcSize += vringSize * 2;
                                                ipcRpMsgOffsetArr[remoteCoreName][remoteCoreName1] = ipcNotifyOffsetArr[remoteCoreName][remoteCoreName1] + (32 * 2);
                                                ipcRpMsgOffsetArr[remoteCoreName1][remoteCoreName] = ipcRpMsgOffsetArr[remoteCoreName][remoteCoreName1] + vringSize;
                                                vringCount += 2;
                                            }
                                            safeIpcConfig = remoteCoreName1 + "_safeipc";
                                            safeIpcConfig1 = remoteCoreName + "_safeipc";

                                            if(remote_core_instance[safeIpcConfig] || remote_core_instance1[safeIpcConfig1])
                                            {
                                                /* Align Start address to firewall granularity */
                                                if(ipcNotifyOffsetArr[remoteCoreName][remoteCoreName1] % firewallGranularity != 0)
                                                {
                                                    startAddrOffset = firewallGranularity - (ipcNotifyOffsetArr[remoteCoreName][remoteCoreName1] % firewallGranularity);
                                                    ipcNotifyOffsetArr[remoteCoreName][remoteCoreName1] += startAddrOffset;
                                                    ipcNotifyOffsetArr[remoteCoreName1][remoteCoreName] += startAddrOffset;

                                                    if(remote_core_instance[remoteCoreName1] == "notify_rpmsg")
                                                    {
                                                        ipcRpMsgOffsetArr[remoteCoreName][remoteCoreName1] += startAddrOffset;
                                                        ipcRpMsgOffsetArr[remoteCoreName1][remoteCoreName] += startAddrOffset;
                                                    }
                                                    sharedMemSize += startAddrOffset;
                                                    offset += startAddrOffset;
                                                }

                                                /* Align IPC data size to firewall granularity */
                                                firewallSize = firewallGranularity * Math.trunc(ipcSize / firewallGranularity);
                                                if((ipcSize < firewallGranularity) || (ipcSize % firewallGranularity != 0)) {
                                                    firewallSize += firewallGranularity;
                                                }
                                                offset += firewallSize - ipcSize;
                                                sharedMemSize += firewallSize;

                                                ipcFirewallAddress.push(
                                                    {
                                                        startAddress: ipc_soc.getSharedMemAddress() + ipcNotifyOffsetArr[remoteCoreName][remoteCoreName1],
                                                        endAddress: ipc_soc.getSharedMemAddress() + ipcNotifyOffsetArr[remoteCoreName][remoteCoreName1] + firewallSize - 1,
                                                        cores: remoteCoreName + ", " + remoteCoreName1,
                                                    }
                                                );
                                            }
                                            else
                                            {
                                                sharedMemSize += ipcSize;
                                            }
                                        }
                                    }
                                    else
                                    {
                                        if(remote_core_instance[remoteCoreName1] == "notify_rpmsg")
                                        {
                                            ipcSize = vringSize;
                                            ipcRpMsgOffsetArr[remoteCoreName][remoteCoreName1] = (vringSize * vringCount);
                                            vringCount += 1;
                                            sharedMemSize += ipcSize;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    ipcInfo.sharedMemUsed = sharedMemSize;
    ipcInfo.ipcNotifyOffsetArr = ipcNotifyOffsetArr;
    ipcInfo.ipcRpMsgOffsetArr = ipcRpMsgOffsetArr;
    ipcInfo.safeIpcEnabled = safeIpcEnabled;
    ipcInfo.ipcFirewallAddress = ipcFirewallAddress;
    ipcInfo.firewallCore = firewallCore;

    return ipcInfo;
}

function getEnabledCpus(instance) {
    let enabledCpus = [];
    let config = ipc_soc.getConfigurables();

    for( let cpuConfig of config)
    {
        if(cpuConfig.name != "enableLinuxIpc" && cpuConfig.name != "enableMailboxIpc" ) {
            if(((instance[cpuConfig.name] == "notify") || (instance[cpuConfig.name] == "notify_rpmsg")) && cpuConfig.name != ipc_soc.getSelfIpcCoreName())
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

function getSysCfgCoreName()
{
    return ipc_soc.getSysCfgCoreName();
}

function getImplementationVersion() {
    return ipc_soc.getImplementationVersion();
}

function validate(instance, report) {

    /* multi-script validation  */
    let configs = ipc_soc.getConfigurables();
    let selfIpcCoreName = getSelfIpcCoreName();
    let ipc_module_name = '/drivers/ipc/ipc';
    let ipcFirewallAddress = getIpcSharedMemData(instance).ipcFirewallAddress;
    let firewallCore = getIpcSharedMemData(instance).firewallCore;
    let safeIpcEnabled = false;

    common.validate.checkConfigurableValueMatchForAllCores(ipc_module_name, instance, report, "vringNumBuf");
    common.validate.checkConfigurableValueMatchForAllCores(ipc_module_name, instance, report, "vringMsgSize");
    common.validate.checkConfigurableValueMatchForAllCores(ipc_module_name, instance, report, "enableCrc");

    common.validate.checkNumberRange(instance, report, "intrPriority", 0, hwi.getHwiMaxPriority(), "dec");

    for ( let config of configs)
    {
        if(config.name != "enableLinuxIpc" && config.name != "enableMailboxIpc" && config.name.search("_safeipc") == -1) {
            let remoteCoreName = config.name;
            if( remoteCoreName != selfIpcCoreName)
            {
                let sysCfgCoreName = ipc_soc.getSysCfgCoreName(remoteCoreName);
                let remote_core_instance = common.getStaticModuleForCore(ipc_module_name, sysCfgCoreName);

                /* IPC option from self core to remote core and from remote core to self core should match */
                if(remote_core_instance)
                {
                    if(instance[remoteCoreName] != remote_core_instance[selfIpcCoreName])
                    {
                        report.logError(`Value MUST match with core ${sysCfgCoreName}`, instance, remoteCoreName);
                    }
                    if(instance[remoteCoreName + "_safeipc"] && !remote_core_instance[selfIpcCoreName + "_safeipc"])
                    {
                        report.logError(`SafeIPC not enabled in remote Core ${sysCfgCoreName}`, instance, remoteCoreName + "_safeipc");
                    }
                }
            }
        }
    }

    /* If top level SafeIPC option has been enabled, at least one of the sub level options should be enabled */
    if(instance["enableSafeIpc"])
    {
        for ( let config of configs)
        {
            if(config.name.search("_safeipc") != -1)
            {
                if(instance[config.name])
                {
                    safeIpcEnabled = true;
                    break;
                }
            }
        }

        if(safeIpcEnabled == false)
        {
            report.logError(`SafeIPC not enabled with any remote Core`, instance, "enableSafeIpc");
        }
    }

    if((instance.sharedMemUsed) > instance.sharedMemAvailable)
    {
        report.logError(`Total Shared Memory used for IPC must be <= ${instance.sharedMemAvailable} bytes. Reduce RP Message number of buffers or buffer size`,
            instance, "sharedMemUsed" );

    }

    if((ipcFirewallAddress.length > 0) && (selfIpcCoreName == firewallCore))
    {
        for(let firewallAddress in ipcFirewallAddress)
        {
            let startAddress = ipcFirewallAddress[firewallAddress].startAddress.toString(16).toUpperCase();
            let endAddress = ipcFirewallAddress[firewallAddress].endAddress.toString(16).toUpperCase();
            let cores = ipcFirewallAddress[firewallAddress].cores.toUpperCase();

            report.logInfo(`Firewall Start Address: 0x${startAddress}, End Address: 0x${endAddress}, Cores: ${cores}`, instance, "enableSafeIpc");
        }
    }

    if(getImplementationVersion() == "v0")
    {
        if(instance.enableLinuxIpc && instance.enableSafeIpc)
        {
            report.logError(`SafeIPC cannot be enabled with Linux IPC`, instance, "enableSafeIpc");
        }

        if(instance.enableLinuxIpc && instance.enableCrc)
        {
            report.logError(`CRC Check cannot be enabled with Linux IPC`, instance, "enableCrc");
        }
    }

    if((instance.enableCrc) && ((instance.crcHookFxn == "NULL") || (instance.crcHookFxn == "")))
    {
        report.logError("Hook function MUST be provided for CRC Calculation", instance, "crcHookFxn");
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
    getEnabledCpus,
    getEnabledRPMessageCpus,
    getImplementationVersion,
    getSysCfgCoreName,
    getIpcSharedMemData,
    getIpcStaticMemory,
};

exports = ipc_module;
