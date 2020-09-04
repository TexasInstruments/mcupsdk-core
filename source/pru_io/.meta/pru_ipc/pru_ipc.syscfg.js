let common = system.getScript("/common");
let soc = system.getScript(`/drivers/pruicss/soc/icss_intc_${common.getSocName()}`);

function getInstanceConfig(moduleInstance) {
    return {
        ...moduleInstance
    }
}

function defaultRxIntcMap(icssInstance, pruCore) {
    if(icssInstance === "ICSSG0")
    {
        if(pruCore === "PRU0")
            return "16";
        else if(pruCore === "PRU1")
            return "17";
    }
    else if(icssInstance === "ICSSG1")
    {
        if(pruCore === "PRU0")
            return "18";
        else if(pruCore === "PRU1")
            return "19";
    }
    return "20";
}

function defaultTxIntcMap(icssInstance, pruCore) {
    if(icssInstance === "ICSSG0")
    {
        if(pruCore === "PRU0")
            return "31";
        else if(pruCore === "PRU1")
            return "30";
    }
    else if(icssInstance === "ICSSG1")
    {
        if(pruCore === "PRU0")
            return "29";
        else if(pruCore === "PRU1")
            return "28";
    }
    return "27";
}

function moduleInstances(instance) {
    let modInstances = new Array();

    // Interrupt Mapping:
    if (instance.interruptRx) {
        // only enable options that are used for PRU IPC
        let allEventOptions = soc.getEventConfigOptions(instance.icssInstance);
        let options = soc.getDisabledOptionsMtoN(allEventOptions, 0, 15, 'Not compatible');
        options.push(...soc.getDisabledOptionsMtoN(allEventOptions, 32, 154, 'Not compatible'));
        let eventDisabledOptions = JSON.stringify(options);

        let allHostOptions = soc.getHostConfigOptions();
        options = soc.getDisabledOptionsMtoN(allHostOptions, 0, 1, 'Not compatible');
        options.push(...soc.getDisabledOptionsMtoN(allHostOptions, 10, 19, 'Not compatible'));
        let hostDisabledOptions = JSON.stringify(options);

        let submodule = "/drivers/pruicss/icss_intc/";
        if(instance.icssInstance === "ICSSG0")
            submodule += "icss0_intc_mapping";
        else if(instance.icssInstance === "ICSSG1")
            submodule += "icss1_intc_mapping";
        modInstances.push({
            name: "rxIntcMapping",
            displayName: instance.icssInstance + " INTC For Interrupt From PRU",
            moduleName: submodule,
            collapsed: false,
            args :{
                event: defaultRxIntcMap(instance.icssInstance, instance.pruCore),
            },
            requiredArgs: {
                eventDisabledOptions,
                hostDisabledOptions,
            },
        });
    }
    if (instance.interruptTx) {
        // only enable options that are used for PRU IPC
        let allHostOptions = soc.getHostConfigOptions();
        let options = soc.getDisabledOptionsMtoN(allHostOptions, 2, 19, 'Not compatible');
        let hostDisabledOptions = JSON.stringify(options, null, 4);

        let submodule = "/drivers/pruicss/icss_intc/";
        if(instance.icssInstance === "ICSSG0")
            submodule += "icss0_intc_mapping";
        else if(instance.icssInstance === "ICSSG1")
            submodule += "icss1_intc_mapping";
        modInstances.push({
            name: "txIntcMapping",
            displayName: instance.icssInstance + " INTC For Interrupt To PRU",
            moduleName: submodule,
            collapsed: false,
            args: {
                event: defaultTxIntcMap(instance.icssInstance, instance.pruCore),
                channel: "0",
            },
            requiredArgs: {
                hostDisabledOptions,
            },
        });
    }

    return (modInstances);
}

let pru_ipc_top_module_name = "/pru_io/pru_ipc/pru_ipc";

let pru_ipc_top_module = {
    displayName: "PRU IPC",

    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/pru_io/pru_ipc/templates/pru_ipc.h.xdt",
            moduleName: pru_ipc_top_module_name,
        },
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/pru_io/pru_ipc/templates/pru_ipc_config.c.xdt",
            moduleName: pru_ipc_top_module_name,
        },
        "/pru_io/common/pru_io_config.inc.xdt": {
            pru_io_config: "/pru_io/pru_ipc/templates/pru_ipc_config.inc.xdt",
            moduleName: pru_ipc_top_module_name,
        },
    },

    defaultInstanceName: "CONFIG_PRU_IPC",
    longDescription: "PRU IPC module to configure APIs for transferring data between specified PRU core and current R5F core",
    config: [
        {
            name: "icssInstance",
            displayName: "ICSSG Instance",
            default: "ICSSG0",
            options: [
                {
                    name: "ICSSG0",
                },
                {
                    name: "ICSSG1",
                },
            ],
        },
        {
            name: "pruCore",
            displayName: "PRU Core",
            default: "PRU0",
            options: [
                {
                    name: "PRU0",
                },
                {
                    name: "PRU1",
                },
                {
                    name: "RTU_PRU0",
                },
                {
                    name: "RTU_PRU1",
                },
                {
                    name: "TX_PRU0",
                },
                {
                    name: "TX_PRU1",
                },
            ],
        },
        {
            name: "dataSize",
            displayName: "Data Packet Size",
            description: "Size of data packets in bytes",
            default: "4",
            options: [
                {
                    name: "1",
                },
                {
                    name: "2",
                },
                {
                    name: "4",
                },
            ],
        },
        {
            name: "blockSize",
            displayName: "Block Size",
            description: "Size of each Block in terms of data packets",
            default: 32,
        },
        {
            name: "noOfBlocks",
            displayName: "No Of Blocks",
            description: "Total Blocks per Buffer",
            default: 4,
        },
        {
            name: "noOfBuffers",
            displayName: "No Of Buffers",
            description: "Total Buffers to reserve for shared memory",
            default: 1,
        },
        {
            name: "interruptRx",
            displayName: "Enable Interrupt On Data Receive",
            default: true,
        },
        {
            name: "interruptTx",
            displayName: "Enable Interrupt To PRU On Data Send",
            default: false,
        },
    ],
    validate,
    getInstanceConfig,
    moduleInstances,
};

function validate(inst, report) {
    common.validate.checkSameFieldName(inst, "pruCore", report);
}

exports = pru_ipc_top_module;
