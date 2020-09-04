
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");

function getInstanceConfig(moduleInstance) {

    let peripheralPinName = null;
    if (moduleInstance[getPinName(moduleInstance)] != null)
        peripheralPinName = moduleInstance[getPinName(moduleInstance)].$solution.peripheralPinName;
    let csNum;
    if (peripheralPinName != null) {
        /* last char is 0 or 1 or 2 or 3 i.e xyz_CSn */
        csNum = peripheralPinName[peripheralPinName.length-1];
    } else {
        /* If CS pin is unchecked/in 3 pin mode, use channel 0 for reigster access. */
        csNum = 0;
    }

    return {
        ...moduleInstance,
        cs: csNum,
    };
};

function pinmuxRequirements(inst) {
    let parent = inst.$ownedBy;
    let interfaceName = inst.interfaceName;
    let config = pinmux.getPinConfigurables(interfaceName, "CS0");

    if( parent[interfaceName] == undefined)
        return [];
    if(inst.pinMode == 3)
        return [];

    return [{
        extend: parent[interfaceName],
        name: getPinName(inst),
        displayName: "CS Pin",
        interfaceNames: ["CS0", "CS1", "CS2", "CS3"],
        config: config,
    }];
}

function getPinName(inst) {
    if(inst.pinMode == 3)
        return "";

    return "CSn";
}

let mcspi_ch_module_name = "/drivers/mcspi/v1/mcspi_v1_channel";

let mcspi_ch_module = {
    displayName: "MCSPI Channel Configuration",
    defaultInstanceName: "CONFIG_MCSPI_CH",
    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: mcspi_ch_module_name,
        },
    },
    config: [
        /* Channel attributes */
        {
            name: "frameFormat",
            displayName: "Frame Format",
            default: "POL0_PHA0",
            options: [
                {
                    name: "POL0_PHA0",
                    displayName: "Mode 0 (POL0 PHA0)"
                },
                {
                    name: "POL0_PHA1",
                    displayName: "Mode 1 (POL0 PHA1)"
                },
                {
                    name: "POL1_PHA0",
                    displayName: "Mode 2 (POL1 PHA0)"
                },
                {
                    name: "POL1_PHA1",
                    displayName: "Mode 3 (POL1 PHA1)"
                },
            ],
            description: "Data frame formats",
            longDescription:
`POL0 = SPICLK is held low during the INACTIVE state\n
POL1 = SPICLK is held high during the INACTIVE state\n
PHA0 = Data are latched on odd-numbered edges of SPICLK\n
PHA1 = Data are latched on even-numbered edges of SPICLK`,
        },
        {
            name: "bitRate",
            displayName: "Clock Frequency (Hz)",
            default: 1000000,
            description: "MCSPI bit rate in Hz",
        },
        {
            name: "csPolarity",
            displayName: "Chip-select Polarity",
            default: "LOW",
            options: [
                {
                    name: "LOW",
                    displayName: "Low"
                },
                {
                    name: "HIGH",
                    displayName: "High"
                },
            ],
            description: "Polarity of the chip select signal",
        },
        /* Advanced parameters */
        {
            name: "advanced",
            displayName: "Show Advanced Channel Config",
            default: false,
            onChange: function(inst, ui) {
                if (inst.advanced == true) {
                    ui.slvCsSelect.hidden = false;
                    ui.startBitEnable.hidden = false;
                    ui.startBitPolarity.hidden = false;
                    ui.csIdleTime.hidden = false;
                    ui.defaultTxData.hidden = false;
                }
                else {
                    ui.slvCsSelect.hidden = true;
                    ui.startBitEnable.hidden = true;
                    ui.startBitPolarity.hidden = true;
                    ui.csIdleTime.hidden = true;
                    ui.defaultTxData.hidden = true;
                }
                /* startBitPolarity applicable only when start bit is enabled */
                if ((inst.advanced == true) &&
                    (inst.startBitEnable == true)) {
                    ui.startBitPolarity.hidden = false;
                }
                else {
                    ui.startBitPolarity.hidden = true;
                }
            },
        },
        {
            name: "slvCsSelect",
            displayName: "Slave Chip-select",
            default: 0,
            hidden: true,
            description: "Slave select signal detection. Applicable for Channel 0 and in slave mode only",
            //TODO: Hide for master mode
        },
        {
            name: "startBitEnable",
            displayName: "Start Bit Enable",
            default: false,
            hidden: true,
            description: "Start bit D/CX added before SPI transfer. Polarity is defined by start bit level",
            onChange: function(inst, ui) {
                /* startBitPolarity applicable only when start bit is enabled */
                if ((inst.advanced == true) &&
                    (inst.startBitEnable == true)) {
                    ui.startBitPolarity.hidden = false;
                }
                else {
                    ui.startBitPolarity.hidden = true;
                }
            },
        },
        {
            name: "startBitPolarity",
            displayName: "Start Bit Polarity",
            default: "LOW",
            hidden: true,
            options: [
                {
                    name: "LOW",
                    displayName: "Low"
                },
                {
                    name: "HIGH",
                    displayName: "High"
                },
            ],
            description: "Start-bit polarity used when startBitEnable is TRUE",
        },
        {
            name: "csIdleTime",
            displayName: "Chip-select TCS",
            default: 0,
            hidden: true,
            description: "Chip select time control in clock cycles",
            longDescription:
`0 = 0.5 clock cycles delay\n
1 = 1.5 clock cycles delay\n
2 = 2.5 clock cycles delay\n
3 = 3.5 clock cycles delay`,
        },
        {
            name: "defaultTxData",
            displayName: "Default TX Data",
            default: 0x00000000,
            hidden: true,
            description: "TX data to transmit when TX buffer pointer is NULL. The actual data that is transmitted depends on the Data Frame Size field",
            displayFormat: "hex",
        },
        {
            name: "interfaceName",
            default: "SPI",
            hidden: true,
        },
        {
            name: "dmaEnable",
            default: "INTERRUPT",
            hidden: true,
        },
        {
            name: "pinMode",
            default: 4,
            hidden: true,
        },
    ],
    validate : validate,
    moduleInstances: moduleInstances,
    getInstanceConfig,
    pinmuxRequirements,
    getPinName,
};

function moduleInstances(inst) {
    let modInstances = new Array();

    if(inst.dmaEnable == "DMA")
    {
        modInstances.push({
            name: "mcspiRxConfigXbar",
            displayName: "MCSPI DMA RX Trigger Configuration",
            moduleName: '/xbar/dma_trig_xbar/dma_trig_xbar',
            requiredArgs: {
                parentName: "MCSPI_RX",
            },
        });
        modInstances.push({
            name: "mcspiTxConfigXbar",
            displayName: "MCSPI DMA TX Trigger Configuration",
            moduleName: '/xbar/dma_trig_xbar/dma_trig_xbar',
            requiredArgs: {
                parentName: "MCSPI_TX",
            },
        });
    }

    return (modInstances);
}

/*
 *  ======== validate ========
 */
function validate(inst, report) {

    /* Divider can go up to 4096. 12207 = 50MHz(functional clk)/4096 */
    common.validate.checkNumberRange(inst, report, "bitRate", 12207, 50000000, "dec");
    common.validate.checkNumberRange(inst, report, "slvCsSelect", 0, 3, "dec");
    common.validate.checkNumberRange(inst, report, "csIdleTime", 0, 3, "dec");
    common.validate.checkNumberRange(inst, report, "defaultTxData", 0, 0xFFFFFFFF, "hex");
}

exports = mcspi_ch_module;
