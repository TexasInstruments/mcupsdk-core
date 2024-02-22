let common = system.getScript("/common");
let hwi = system.getScript("/kernel/dpl/hwi.js");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/ospi/soc/ospi_${common.getSocName()}`);

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
    config : [

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
    ],
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


function validate(inst, report) {

    common.validate.checkNumberRange(inst, report, "intrPriority", 0, hwi.getHwiMaxPriority(), "dec");
    common.validate.checkNumberRange(inst, report, "baudRateDiv", 2, 32, "dec");
    if(inst.baudRateDiv % 2)
    {
        report.logError("Value MUST be EVEN number", inst, "baudRateDiv");
    }
}

exports = ospi_module;