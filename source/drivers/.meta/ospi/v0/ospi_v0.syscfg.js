let common = system.getScript("/common");
let hwi = system.getScript("/kernel/dpl/hwi.js");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/ospi/soc/ospi_${common.getSocName()}`);
let phyConfigs = (common.getSocName() == "am263px");

function getConfigArr() {
	return soc.getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let solution = moduleInstance[getInterfaceName(moduleInstance)].$solution;
    let configArr = getConfigArr();
    let config = configArr.find(o => o.name === solution.peripheralName);

    config.clockFrequencies[0].clkRate = moduleInstance.inputClkFreq;

    return {
        ...config,
        ...moduleInstance,
    };
};

function getInterfaceName(inst) {

    return "OSPI";
}

function getPeripheralPinNames(inst) {
    return [ "CLK", "CSn0", "CSn1", "CSn2", "CSn3", "D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "DQS" ];
}

function getDmaRestrictedRegions() {
    return soc.getDmaRestrictedRegions();
}

function pinmuxRequirements(inst) {
   let interfaceName = getInterfaceName(inst);

    let resources = [];
    let pinResource = {};

    pinResource = pinmux.getPinRequirements(interfaceName, "CLK", "OSPI CLK Pin");
    pinmux.setConfigurableDefault( pinResource, "rx", false );
    resources.push( pinResource);

    switch(inst.chipSelect)
    {
        default:
        case "CS0":
            pinResource = pinmux.getPinRequirements(interfaceName, "CSn0", "OSPI CS0 Pin");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);
            break;
        case "CS1":
            pinResource = pinmux.getPinRequirements(interfaceName, "CSn1", "OSPI CS1 Pin");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);
            break;
        case "CS2":
            pinResource = pinmux.getPinRequirements(interfaceName, "CSn2", "OSPI CS2 Pin");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);
            break;
        case "CS3":
            pinResource = pinmux.getPinRequirements(interfaceName, "CSn3", "OSPI CS3 Pin");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);
            break;
    }

    switch(soc.getSupportedDataLines()) {
        default:
        case 8:
            pinResource = pinmux.getPinRequirements(interfaceName, "DQS", "OSPI Data Strobe Pin");
            pinmux.setConfigurableDefault( pinResource, "rx", true );
            resources.push( pinResource);
            pinResource = pinmux.getPinRequirements(interfaceName, "D7", "OSPI Data I/O Pin7");
            pinmux.setConfigurableDefault( pinResource, "rx", true );
            resources.push( pinResource);
            pinResource = pinmux.getPinRequirements(interfaceName, "D6", "OSPI Data I/O Pin6");
            pinmux.setConfigurableDefault( pinResource, "rx", true );
            resources.push( pinResource);
            pinResource = pinmux.getPinRequirements(interfaceName, "D5", "OSPI Data I/O Pin5");
            pinmux.setConfigurableDefault( pinResource, "rx", true );
            resources.push( pinResource);
            pinResource = pinmux.getPinRequirements(interfaceName, "D4", "OSPI Data I/O Pin4");
            pinmux.setConfigurableDefault( pinResource, "rx", true );
            resources.push( pinResource);
        case 4:
            pinResource = pinmux.getPinRequirements(interfaceName, "D3", "OSPI Data I/O Pin3");
            pinmux.setConfigurableDefault( pinResource, "rx", true );
            resources.push( pinResource);
            pinResource = pinmux.getPinRequirements(interfaceName, "D2", "OSPI Data I/O Pin2");
            pinmux.setConfigurableDefault( pinResource, "rx", true );
            resources.push( pinResource);
        case 2:
            pinResource = pinmux.getPinRequirements(interfaceName, "D1", "OSPI Data I/O Pin1");
            pinmux.setConfigurableDefault( pinResource, "rx", true );
            resources.push( pinResource);
        case 1:
            pinResource = pinmux.getPinRequirements(interfaceName, "D0", "OSPI Data I/O Pin0");
            pinmux.setConfigurableDefault( pinResource, "rx", true );
            resources.push( pinResource);

    }

    let peripheral = {
        name: interfaceName,
        displayName: "OSPI Instance",
        interfaceName: interfaceName,
        resources: resources,
    };

    return [peripheral];
}

function getClockEnableIds(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockIds;
}

function getClockFrequencies(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockFrequencies;
}

const ospi_supported_protocols = [
    { name : "1s_1s_1s", displayName : "1S-1S-1S" },
    { name : "1s_1s_2s", displayName : "1S-1S-2S" },
    { name : "1s_1s_4s", displayName : "1S-1S-4S" },
    { name : "1s_1s_8s", displayName : "1S-1S-8S" },
    { name : "4s_4s_4s", displayName : "4S-4S-4S" },
    { name : "4s_4d_4d", displayName : "4S-4D-4D" },
    { name : "8s_8s_8s", displayName : "8S-8S-8S" },
    { name : "8d_8d_8d", displayName : "8D-8D-8D" },
    { name : "custom",   displayName : "Custom Protocol" },
];

function getSupportedProtocols() {

    return ospi_supported_protocols;
}

let ospi_module_name = "/drivers/ospi/ospi";

let ospi_module = {
    displayName: "OSPI",
    templates: soc.getTemplates(),
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_OSPI",
    validate: validate,
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    config : getConfigurables(),
    sharedModuleInstances: soc.addModuleInstances,
    pinmuxRequirements,
    getInstanceConfig,
    getInterfaceName,
    getPeripheralPinNames,
    getClockEnableIds,
    getClockFrequencies,
    getDmaRestrictedRegions,
    getSupportedProtocols,
    onMigrate,
};


function onMigrate(newInst, oldInst, oldSystem) {
    let pins = getPeripheralPinNames(oldInst)
    let interfaceName = getInterfaceName(oldInst)
    common.onMigrate(newInst, oldInst, oldSystem, pins, interfaceName)
}

function getConfigurables()
{
    let config = [];
    config.push(
        {
            name: "inputClkFreq",
            displayName: "Input Clock Frequency (Hz)",
            default: soc.getDefaultConfig().inputClkFreq,
        },
        {
            name: "baudRateDiv",
            displayName: "Input Clock Divider",
            description: "OSPI Output Clock = Input Clock Frequency / Input Clock Divider",
            default: soc.getDefaultConfig().baudRateDiv,
        },
        {
            name: "chipSelect",
            displayName: "Chip Select",
            default: "CS0",
            options: [
                { name: "CS0" },
                { name: "CS1" },
                { name: "CS2" },
                { name: "CS3" },
            ]
        },
        {
            name: "protocol",
            displayName: "Protocol",
            description: "The NOR SPI protocol to be used",
            default: ospi_supported_protocols[0].name,
            options: ospi_supported_protocols,
            onChange: function(inst, ui) {
                let hideLines = true;
                if(inst.protocol == "custom") {
                    hideLines = false;
                }
                /* Add manual config stuff */
                ui.cmdLines.hidden = hideLines;
                ui.addrLines.hidden = hideLines;
                ui.dataLines.hidden = hideLines;
            }
        },
        {
            name: "cmdLines",
            displayName: "CMD Lines",
            description: "Number of transfer lines to be used for sending CMD",
            default: "1",
            options: [
                { name: "1" },
                { name: "2" },
                { name: "4" },
                { name: "8" },
            ],
            hidden: true,
        },
        {
            name: "addrLines",
            displayName: "ADDR Lines",
            description: "Number of transfer lines to be used for sending ADDR",
            default: "1",
            options: [
                { name: "1" },
                { name: "2" },
                { name: "4" },
                { name: "8" },
            ],
            hidden: true,
        },
        {
            name: "dataLines",
            displayName: "DATA Lines",
            description: "Number of transfer lines to be used for sending DATA",
            default: "1",
            options: [
                { name: "1" },
                { name: "2" },
                { name: "4" },
                { name: "8" },
            ],
            hidden: true,
        },
        {
            name: "dmaEnable",
            displayName: "Enable DMA",
            default: false,
            description: `Enable data transfer using DMA`,
        },
        {
            name: "phyEnable",
            displayName: "Enable PHY Mode",
            default: false,
            description: `PHY mode MUST be enabled when using higher clocks (> 50 Mhz)`,
            onChange: function(inst,ui)
            {
                if(phyConfigs)
                {
                    ui.phaseDetectDelayElement.hidden = !inst.phyEnable;
                    ui.phyControlMode.hidden = !inst.phyEnable;
                    ui.dllLockMode.hidden = !inst.phyEnable;
                    ui.txDllLowWindowStart.hidden = !inst.phyEnable;
                    ui.txDllLowWindowEnd.hidden = !inst.phyEnable;
                    ui.txDllHighWindowStart.hidden = !inst.phyEnable;
                    ui.txDllHighWindowEnd.hidden = !inst.phyEnable;
                    ui.rxLowSearchStart.hidden = !inst.phyEnable;
                    ui.rxLowSearchEnd.hidden = !inst.phyEnable;
                    ui.rxHighSearchStart.hidden = !inst.phyEnable;
                    ui.rxHighSearchEnd.hidden = !inst.phyEnable;
                    ui.txLowSearchStart.hidden = !inst.phyEnable;
                    ui.txLowSearchEnd.hidden = !inst.phyEnable;
                    ui.txHighSearchStart.hidden = !inst.phyEnable;
                    ui.txHighSearchEnd.hidden = !inst.phyEnable;
                    ui.txDLLSearchOffset.hidden = !inst.phyEnable;
                    ui.rdDelayMin.hidden = !inst.phyEnable;
                    ui.rdDelayMax.hidden = !inst.phyEnable;
                    ui.rxTxDLLSearchStep.hidden = !inst.phyEnable;
                }
            }
        },
        /* Advanced parameters */
        {
            name: "advanced",
            displayName: "Show Advanced Config",
            default: false,
            onChange: function (inst, ui) {
                let hideConfigs = true;
                if(inst.advanced == true) {
                    hideConfigs = false;
                }
                ui.intrEnable.hidden = hideConfigs;
                ui.intrPriority.hidden = hideConfigs;
                ui.frmFmt.hidden = hideConfigs;
                ui.decChipSelect.hidden = hideConfigs;
            },
        },
        {
            name: "intrEnable",
            displayName: "Interrupt Mode Enable",
            description: "NOT tested, DO NOT USE",
            default: false,
            hidden: true,
        },
        {
            name: "intrPriority",
            displayName: "Interrupt Priority",
            description: "NOT tested, DO NOT USE",
            default: 4,
            hidden: true,
            description: `Interrupt Priority: 0 (highest) to ${hwi.getHwiMaxPriority()} (lowest)`,
        },
        /* Advance Open attributes */
        {
            name: "frmFmt",
            displayName: "Frame Format",
            default: "POL0_PHA0",
            hidden: true,
            options: [
                { name: "POL0_PHA0" },
                { name: "POL0_PHA1" },
                { name: "POL1_PHA0" },
                { name: "POL1_PHA1" },
            ]
        },
        {
            name: "decChipSelect",
            displayName: "Decoder Chip Select",
            default: "OSPI_DECODER_SELECT4",
            hidden: true,
            options: [
                { name: "OSPI_DECODER_SELECT4", displayName: "DECODER_SELECT4" },
                { name: "OSPI_DECODER_SELECT16", displayName: "DECODER_SELECT16" },
            ]
        },

    )

    if(phyConfigs)
    {
        config.push(
            {
                name: "phyConfig",
                displayName : "PHY Configuration",
                collapsed : true,
                config : [
                    {
                        name: "phaseDetectDelayElement",
                        displayName: "Phase Detect Delay Element",
                        description: "Number of delay elements to be inserted between phase detect flip-flops ",
                        default: soc.getPhyTuningParams().phaseDelayElement,
                        hidden: !phyConfigs,
                    },
                    {
                        name : "phyControlMode",
                        displayName: "PHY Control Mode",
                        longDescription :"\nControls the bypass mode of the master and slave DLLs. \
                        \nIf this bit is set, the bypass mode is intended to be used only for debug. \
                        \n0h = Master operational mode \
                        \nDLL works in normal mode of operation where the slave delay line \
                        \nsettings are used as fractional delay of the master delay line encoder \
                        \nreading of the number of delays in one cycle. \
                        \n1h = Bypass mode \
                        \nMaster DLL is disabled with only 1 delay element in its delay line. \
                        \nThe slave delay lines decode delays in absolute delay elements \
                        \nrather than as fractional delays.",
                        default: soc.getPhyTuningParams().phyControlMode,
                        options: [
                            { name : "PHY_MASTER_MODE", displayName : "Master Mode"},
                            { name : "PHY_BYPASS_MODE", displayName : "Bypass Mode"}
                        ],
                        hidden: !phyConfigs,
                    },
                    {
                        name : "dllLockMode",
                        displayName : "DLL Lock Mode",
                        longDescription : "Determines if the master delay line locks on a full cycle or half cycle \
                        of delay. This bit need not be written by software. Force DLL lock mode with this setting.",
                        default: soc.getPhyTuningParams().dllLockMode,
                        options: [
                            { name : "FULL_CYCLE_LOCK", displayName : "Full Cycle Lock"},
                            { name : "HALF_CYCLE_LOCK", displayName : "Half Cycle Lock"}
                        ],
                        hidden: !phyConfigs,
                    },
                    {
                        name : "windowParams",
                        displayName : "Tuning Window Parameters",
                        longDescription : "Shown below is an approximate txDLL vs rxDLL graph of a typical PHY. \
                        \nRegions P1-P2-BL and Q1-Q2-R2-TR-R1 are the passing regions. \
                        \nEach region corresponds to a different value of read data capture delay. \
                        \nThe gap between the regions can move away or towards origin depending on various factors (like temperature).  \
                        \nThere can be just one region also. Or the orientation of the gap will be opposite.  \
                        \n\nThe tuning/calibration algorithm can work correctly only if we have a general idea of this graph.\n" + "\n" +
                        `                       RX
                           |\n\
                           |     R1\n\
                           |     _______________________________ TR\n\
                           |     |                               |\n\
                           |     | Q1                            |\n\
                           |   P1 \\                              |\n\
                           |       \\                             |\n\
                           |     |\\ \\                            |\n\
                           |     | \\ \\                           |\n\
                           |     |  \\ \\                          |\n\
                           |     |   \\ \\                         |\n\
                           |     |    \\ \\                        |\n\
                           |     |     \\ \\                       |\n\
                           |     |      \\ \\                      |\n\
                           |     |       \\ \\ Q2                  |\n\
                           |     |________\\ \\____________________|R2\n\
                           |     BL     P2\n\
                           |_________________________________________ TX\n`+
                        "\n\nTo find the RxDLL boundaries, we fix a valid TxDLL and search through RxDLL range, rdDelay values \
                        \nAs we are not sure of a valid TxDLL we use a window of TxDLL values to find the RxDLL boundaries.\n " + "\n" +
                        `               Rx_DLL\n\
                        ▲\n\
                        │   ++++++++++++++++\n\
                    127 │     ++++++++++++++\n\
                        │   x   ++++++++++++\n\
                        │   xx   +++++++++++\n\
                        │   xxx   ++++++++++\n\
                        │   xxxx   +++++++++\n\
                        │   xxxxx   ++++++++\n\
                        │ │ xxx│xx   +++++++\n\
                        │ │ xxx│xxx   ++++++\n\
                        │ │ xxx│xxxx   +++++\n\
                        │ │ xxx│xxxxx   ++++\n\
                        │ │ xxx│xxxxxx   +++\n\
               Search   │ │ xxx│xxxxxxx   ++\n\
               Rx_Low ──┼─┤►xxx│xxxxxxxx   +\n\
                        │ │    │\n\
                       ─┼─┼────┼────────────►  Tx_DLL\n\
                       0│ │    │           127\n\
                          │    │\n\
                          │    │\n\
          \n\
                      Tx_Low   Tx_Low\n\
                      Start    End\n` +
                        "\n\nFind the rxDLL boundaries using the TxDLL window at the higher end .            \
                        \nwe start the window_end and decrement the TxDLL value until we find the valid point.\n" +"\n" +
                        `               Rx_DLL\n
                        ▲\n\
                        │   ++++++++++++++++\n\
                    127 │   ++++++++++++++++\n\
                        │   ++++++++++++++++\n\
                        │    +++++++++++++++\n\
                        │     +++++++++│++++│\n\
                        │      ++++++++│++++│\n\
                        │   x   +++++++│++++│\n\
                        │   xx   ++++++│++++│\n\
                        │   xxx   +++++│++++│\n\
                        │   xxxx   ++++│++++│\n\
                        │   xxxxx   +++│++++│\n\
                        │   xxxxxx   ++│++++│\n\
                        │   xxxxxxx   +│++++│         Search\n\
                        │   xxxxxxxx   │++++◄───────  Rx_Low\n\
                        │              │    │\n\
                        ─┼──────────────┼────┤► Tx_DLL\n\
                        0│              │    │   127\n\
                                        │    │\n\
                                Tx_High        Tx_High\n\
                                Start          End\n`,
                        collapsed: true,
                        config: [
                            {
                                name: "rdDelayMin",
                                displayName : "Read Delay Min",
                                description : "Minimum value of Read delay for Read Delay Capture Register for tuning search.",
                                default: soc.getPhyTuningParams().rdDelayMin,
                                displayFormat: "dec",
                                hidden: !phyConfigs,
                            },
                            {
                                name: "rdDelayMax",
                                displayName : "Read Delay Max",
                                description : "Maximum value of Read delay for Read Delay Capture Register for tuning search.",
                                default: soc.getPhyTuningParams().rdDelayMax,
                                displayFormat: "dec",
                                hidden: !phyConfigs,
                            },
                            {
                                name: "txDllLowWindowStart",
                                displayName : "RxDLL Search - TxDLL Low Start",
                                description : "Tx Dll window lower value to search RxDLL low and high. \
                                This corresponds to the bottom left point serach.",
                                default: soc.getPhyTuningParams().txDllLowWindowStart,
                                displayFormat: "dec",
                                hidden: !phyConfigs,
                            },
                            {
                                name: "txDllLowWindowEnd",
                                displayName : "RxDLL Search - TxDLL Low End",
                                description : "Tx Dll window higher value to search RxDLL low and high. \
                                This corresponds to the bottom left point search.",
                                default: soc.getPhyTuningParams().txDllLowWindowEnd,
                                displayFormat: "dec",
                                hidden: !phyConfigs,
                            },
                            {
                                name: "txDllHighWindowStart",
                                displayName : "RxDLL Search - TxDLL High Start",
                                description : "Tx Dll window lower value to search RxDLL low and high. \
                                This corresponds to the top right point search.",
                                default: soc.getPhyTuningParams().txDllHighWindowStart,
                                displayFormat: "dec",
                                hidden: !phyConfigs,
                            },
                            {
                                name: "txDllHighWindowEnd",
                                displayName : "RxDLL Search - TxDLL High End",
                                description : "Tx Dll window higher value to search RxDLL low and high. \
                                This corresponds to the top right point search.",
                                default: soc.getPhyTuningParams().txDllHighWindowEnd,
                                displayFormat: "dec",
                                hidden: !phyConfigs,
                            },
                            {
                                name: "rxLowSearchStart",
                                displayName : "RxDLL Low Search Start",
                                description : "Rx Dll lower value for Rx Dll low search. \
                                The value of Rx dll will lie in this window bottom left point search.",
                                default: soc.getPhyTuningParams().rxLowSearchStart,
                                displayFormat: "dec",
                                hidden: !phyConfigs,
                            },
                            {
                                name: "rxLowSearchEnd",
                                displayName : "RxDLL Low Search End",
                                description : "Rx Dll higher value for Rx Dll low search. \
                                The value of Rx dll will lie in this window bottom left point search.",
                                default: soc.getPhyTuningParams().rxLowSearchEnd,
                                displayFormat: "dec",
                                hidden: !phyConfigs,
                            },
                            {
                                name: "rxHighSearchStart",
                                displayName : "RxDLL High Search Start",
                                description : "Rx Dll lower value for Rx Dll high search. \
                                The value of Rx dll will lie in this window top right point search.",
                                default: soc.getPhyTuningParams().rxHighSearchStart,
                                displayFormat: "dec",
                                hidden: !phyConfigs,
                            },
                            {
                                name: "rxHighSearchEnd",
                                displayName : "RxDLL High Search End",
                                description : "Rx Dll higher value for Rx Dll high search. \
                                The value of Rx dll will lie in this window for top right point search.",
                                default: soc.getPhyTuningParams().rxHighSearchEnd,
                                displayFormat: "dec",
                                hidden: !phyConfigs,
                            },
                            {
                                name: "txLowSearchStart",
                                displayName : "TxDLL Low Search Start",
                                description : "Tx Dll lower value for Tx Dll low search. \
                                The value of Tx dll will lie in this window.",
                                default: soc.getPhyTuningParams().txLowSearchStart,
                                displayFormat: "dec",
                                hidden: !phyConfigs,
                            },
                            {
                                name: "txLowSearchEnd",
                                displayName : "TxDLL Low Search End",
                                description : "Tx Dll higher value for Tx Dll low search. \
                                The value of Tx dll will lie in this window.",
                                default: soc.getPhyTuningParams().txLowSearchEnd,
                                displayFormat: "dec",
                                hidden: !phyConfigs,
                            },
                            {
                                name: "txHighSearchStart",
                                displayName : "TxDLL High Search Start",
                                description : "Tx Dll lower value for Tx Dll high search. \
                                The value of Tx dll will lie in this window.",
                                default: soc.getPhyTuningParams().txHighSearchStart,
                                displayFormat: "dec",
                                hidden: !phyConfigs,
                            },
                            {
                                name: "txHighSearchEnd",
                                displayName : "TxDLL High Search End",
                                description : "Tx Dll higher value for Tx Dll high search. \
                                The value of Tx dll will lie in this window.",
                                default: soc.getPhyTuningParams().txHighSearchEnd,
                                displayFormat: "dec",
                                hidden: !phyConfigs,
                            },
                            {
                                name: "txDLLSearchOffset",
                                displayName : "TxDLL Search Offset",
                                description : "Tx Dll step increase for backup Rx Dll low and high search.",
                                default: soc.getPhyTuningParams().txDLLSearchOffset,
                                displayFormat: "dec",
                                hidden: !phyConfigs,
                            },
                            {
                                name: "rxTxDLLSearchStep",
                                displayName : "RxDL & TxDLL Search Step",
                                description : "Rx Dll and Tx DLL step increase for Rx Dll and Tx Dll low and high search.",
                                default: soc.getPhyTuningParams().rxTxDLLSearchStep,
                                displayFormat: "dec",
                                hidden: !phyConfigs,
                            },
                        ]
                    }
                ]
            },
        )
    }

    return config;
}
function validate(inst, report) {

    common.validate.checkNumberRange(inst, report, "intrPriority", 0, hwi.getHwiMaxPriority(), "dec");
    common.validate.checkNumberRange(inst, report, "baudRateDiv", 2, 32, "dec");
    if(inst.baudRateDiv % 2)
    {
        report.logError("Value MUST be EVEN number", inst, "baudRateDiv");
    }
}

exports = ospi_module;