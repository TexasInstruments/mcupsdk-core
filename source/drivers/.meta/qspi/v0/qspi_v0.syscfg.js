
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/qspi/soc/qspi_${common.getSocName()}`);

function getConfigArr() {
    return soc.getConfigArr();}

function getInstanceConfig(moduleInstance) {
    let solution = moduleInstance[getInterfaceName(moduleInstance)].$solution;
    let configArr = getConfigArr();
    let config = configArr.find( o => o.name === solution.peripheralName);

    config.clockFrequencies[0].clkRate = moduleInstance.inputClkFreq;

    return {
        ...config,
        ...moduleInstance,
    };
};

function getClockEnableIds(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockIds;
}

function getClockFrequencies(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockFrequencies;
}

function getPeripheralPinNames(inst) {
    let pinNameD0 = getQspiPinName(inst, "D0");
    let pinNameD1 = getQspiPinName(inst, "D1");
    let pinNameD2 = getQspiPinName(inst, "D2");
    let pinNameD3 = getQspiPinName(inst, "D3");
    let pinNameCLK = getQspiPinName(inst, "CLK");
    let pinNameCS = getQspiPinName(inst, inst.chipSelect);
    return [ pinNameD0, pinNameD1, pinNameD2, pinNameD3, pinNameCLK, pinNameCS];
}

function pinmuxRequirements(inst) {


    let resources = [];
    let interfaceName = getInterfaceName(inst);

    resources.push( pinmux.getPinRequirements(interfaceName, getQspiPinName(inst, "D0"), "QSPI D0 Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, getQspiPinName(inst, "D1"), "QSPI D1 Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, getQspiPinName(inst, "D2"), "QSPI D2 Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, getQspiPinName(inst, "D3"), "QSPI D3 Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, getQspiPinName(inst, "CLK"), "QSPI CLK Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, getQspiPinName(inst, inst.chipSelect), "QSPI CS Pin"));

    let qspi = {
        name: interfaceName,
        displayName: "QSPI Instance",
        interfaceName: interfaceName,
        resources: resources,
        canShareWith: "/drivers/qspi/qspi"
    };

   return [qspi];
}

function getQspiPinName(inst, qspiPinName) {
    return soc.getQspiPinName(inst, qspiPinName);
}

function getInterfaceName(inst) {
    return soc.getInterfaceName(inst);
}

const qspi_supported_protocols = [
    { name : "1s_1s_1s", displayName : "1S-1S-1S" },
    { name : "1s_1s_2s", displayName : "1S-1S-2S" },
    { name : "1s_1s_4s", displayName : "1S-1S-4S" },
    { name : "custom",   displayName : "Custom Protocol" },
]

function getSupportedProtocols() {

    return qspi_supported_protocols;
}

let qspi_module_name = "/drivers/qspi/qspi";

let qspi_module = {
    displayName: "QSPI",
    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: qspi_module_name,
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: qspi_module_name,
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_QSPI",
    config: [
        {
            name: "inputClkFreq",
            displayName: "Input Clock Frequency (Hz)",
            default: soc.getDefaultConfig().inputClkFreq,
        },
        {
            name: "baudRateDiv",
            displayName: "Input Clock Divider",
            description: "QSPI Output Clock = Input Clock Frequency / Input Clock Divider",
            default: soc.getDefaultConfig().baudRateDiv,
        },
        {
            name: "wrdLen",
            displayName: "Word Length",
            description: "Word Length for QSPI transfer",
            default: soc.getDefaultConfig().wrdLen,
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
            default: qspi_supported_protocols[0].name,
            options: qspi_supported_protocols,
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
            ],
            hidden: true,
        },
        {
            name: "dmaEnable",
            displayName: "Enable DMA",
            default: true,
            description: `Enable data transfer using DMA`,
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
                ui.intrNum.hidden = hideConfigs;
                ui.frmFmt.hidden = hideConfigs;
                ui.csPol.hidden = hideConfigs;
                ui.dataDelay.hidden = hideConfigs;
            },
        },
        {
            name: "intrEnable",
            displayName: "Interrupt Mode Enable",
            description: "NOT tested, DO NOT USE",
            default: false,
            hidden: true,
            onChange: function (inst, ui) {
                let hideConfigs = true;
                if(inst.intrEnable == true) {
                    hideConfigs = false;
                }
                ui.wordIntr.hidden = hideConfigs;
                ui.frameIntr.hidden = hideConfigs;
            },
        },
        {
            name: "wordIntr",
            displayName: "Word Interrupt",
            description: "NOT tested, DO NOT USE",
            default: true,
            hidden: true,
        },
        {
            name: "frameIntr",
            displayName: "Frame Interrupt",
            description: "NOT tested, DO NOT USE",
            default: true,
            hidden: true,
        },
        {
            name: "intrNum",
            displayName: "Interrupt Number",
            description: "NOT tested, DO NOT USE",
            default: soc.getDefaultConfig().intrNum,
            hidden: true,
        },
        {
            name: "intrPriority",
            displayName: "Interrupt Priority",
            description: "NOT tested, DO NOT USE",
            default: 4,
            hidden: true,
            description: `Interrupt Priority: 0 (highest) to x (lowest)`,
        },
        {
            name: "sdkInfra",
            displayName: "SDK Infra",
            default: "HLD",
            options: [
                {
                    name: "HLD",
                    displayName: "HLD"
                },
                {
                    name: "LLD",
                    displayName: "LLD"
                },
            ],
            description: "SDK Infra",
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
            name: "csPol",
            displayName: "Chip Select Polarity",
            default: "ACTIVE_LOW",
            hidden: true,
            options: [
                { name: "ACTIVE_LOW" },
                { name: "ACTIVE_HIGH" },

            ]
        },
        {
            name: "dataDelay",
            displayName: "Data Delay",
            default: "DELAY_0",
            hidden: true,
            options: [
                { name: "DELAY_0" },
                { name: "DELAY_1" },
                { name: "DELAY_2" },
                { name: "DELAY_3" },
            ]
        },
    ],
    validate: validate,
    moduleStatic: {
        modules: function(instance) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    sharedModuleInstances: addModuleInstances,
    moduleInstances: moduleInstances,
    pinmuxRequirements,
    getInstanceConfig,
    getClockEnableIds,
    getClockFrequencies,
    getPeripheralPinNames,
    getInterfaceName,
    getQspiPinName,
    getSupportedProtocols,
};

function addModuleInstances(instance) {
    let modInstances = new Array();

    modInstances.push({
        name: "edmaConfig",
        displayName: "EDMA",
        moduleName: '/drivers/edma/edma',
    });

    return modInstances;
}

/*
 *  ======== moduleInstances ========
 */
function moduleInstances(inst) {
    let modInstances = new Array();

    if( inst.sdkInfra == "HLD")
    {
        modInstances.push({
            name: "child",
            moduleName: '/drivers/qspi/v0/qspi_v0_template',
            },
        );
    }
    else
    {
        modInstances.push({
            name: "child",
            moduleName: '/drivers/qspi/v0/qspi_v0_lld_template',
            },
        );
    }


    return (modInstances);
}

function validate(instance, report) {
    common.validate.checkSameInstanceName(instance, report);
}

exports = qspi_module;
