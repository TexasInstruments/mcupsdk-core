let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/mibspi/soc/mibspi_${common.getSocName()}`);

const MAXCHANNELS = 3; /* max number of channels per MIBSPI can support, This will vary depending upon instance name  */

function getConfigArr() {
    return soc.getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let configArr = getConfigArr();
    let config = configArr.find( o => o.name === moduleInstance.instance);

     return {
        ...config,
        ...moduleInstance,
     };
};

function getMaxChannels(inst) {

    if(inst.mode == "PERIPHERAL") {
        return 1;
    }
    else {
        return MAXCHANNELS;
    }
}

function getPeripheralPinNames(inst) {

    return [ "CLK", "MOSI", "MISO", "CS0", "CS1", "CS2"];
}

function pinmuxRequirements(inst) {

    let resources = [];
    let interfaceName = getInterfaceName(inst);

	const pullUpDownOption = {
                   name: "pu_pd",
                   displayName: "Pull Up/Down",
                   options: [
				   {
                        name: "pu",
                        displayName: "Pull Up",
                   },
				   {
                         name: "pd",
                         displayName: "Pull Down",
                   },
				   {
                            name: "nopull",
                            displayName: "No Pull",
                    }
				],
                default: "pu",

                };

	const clkPin =  {
                    name: "CLK",
                    displayName: "MIBSPI CLK Pin",
                    interfaceNames: ["CLK"],
                    config: [pullUpDownOption],
					};

	const misoPin = {
                     name: "MISO",
                     displayName: "MIBSPI MISO Pin",
                     interfaceNames: ["MISO"],
                     config: [pullUpDownOption],
					};

	const mosiPin = {
                    name: "MOSI",
                    displayName: "MIBSPI MOSI Pin",
                    interfaceNames: ["MOSI"],
                    config: [pullUpDownOption],
					};

    const spiPeripheral = {
                    name: interfaceName,
                    displayName: "SPI Instance",
                    interfaceName: "SPI",
					interfaceName: interfaceName,
                    resources: [clkPin,misoPin,mosiPin],
					};

     return [spiPeripheral];
}

function getInterfaceName(inst) {
    return soc.getInterfaceName(inst);
}

function getClockEnableIds(instance) {
    let instConfig = getInstanceConfig(instance);
    return instConfig.clockIds;
}

function getClockFrequencies(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockFrequencies;
}

let mibspi_module_name = "/drivers/mibspi/mibspi";
let mibspi_module = {
    displayName: "MIBSPI",

    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/mibspi/templates/mibspi_config.c.xdt",
            driver_init: "/drivers/mibspi/templates/mibspi_init.c.xdt",
            driver_deinit: "/drivers/mibspi/templates/mibspi_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/mibspi/templates/mibspi.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/mibspi/templates/mibspi_open_close_config.c.xdt",
            driver_open: "/drivers/mibspi/templates/mibspi_open.c.xdt",
            driver_close: "/drivers/mibspi/templates/mibspi_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/mibspi/templates/mibspi_open_close.h.xdt",
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: mibspi_module_name,
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: mibspi_module_name,
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_MIBSPI",
    config: [
        {
            name: "mode",
            displayName: "Mode of Operation",
            default: "CONTROLLER",
            options: [
                {
                    name: "CONTROLLER",
                    displayName: "Controller"
                },
                {
                    name: "PERIPHERAL",
                    displayName: "Peripheral"
                },
            ],
            description: "Controller/Peripheral mode of operation",
            onChange: function (inst, ui) {
                if(inst.mode == "CONTROLLER") {
                    ui.bitRate.hidden = false;
                    ui.t2cDelay.hidden = false;
                    ui.c2tDelay.hidden = false;
                    ui.wDelay.hidden = false;
                }
                else {
                    ui.bitRate.hidden = true;
                    ui.t2cDelay.hidden = true;
                    ui.c2tDelay.hidden = true;
                    ui.wDelay.hidden = true;
                }
            },
        },
        /* Controller mode params */
        {
            name: "bitRate",
            displayName: "Clock Frequency (Hz)",
            default: 5000000,
            description: "MIBSPI bit rate in Hz",
        },
        {
            name: "t2cDelay",
            displayName: "Transmit End Delay",
            default: 0,
            description: "Transmit end to chip select inactive delay",
            displayFormat: "hex",
        },
        {
            name: "c2tDelay",
            displayName: "Chip Select Active Delay",
            default: 0,
            description: "Chip select active to transmit start delay",
            displayFormat: "hex",
        },
        {
            name: "wDelay",
            displayName: "Delay Between Transmissions",
            default: 0,
            description: "Delay in between transmissions",
            displayFormat: "hex",
        },
        {
            name: "dataSize",
            displayName: "Data Size",
            default: 16,
            options: [
                {
                    name: 8,
                },
                {
                    name: 16,
                },
            ],
            description: "Data frame size : Must be 8 or 16 bits",
        },
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
            name: "transferMode",
            displayName: "Transfer Mode",
            default: "BLOCKING",
            hidden: false,
            options: [
                {
                    name: "BLOCKING",
                    displayName: "Blocking"
                },
                {
                    name: "CALLBACK",
                    displayName: "Callback"
                },
            ],
            onChange: function (inst, ui) {
                if(inst.transferMode == "CALLBACK") {
                    ui.transferCallbackFxn.hidden = false;
                    if(inst.transferCallbackFxn == "NULL") {
                        /* Clear NULL entry as user need to provide a fxn */
                        inst.transferCallbackFxn = "";
                    }
                }
                else {
                        ui.transferCallbackFxn.hidden = true;
                        inst.transferCallbackFxn = "NULL";
                }

                if(inst.transferMode == "CALLBACK") {
                    /* multi iCount supported only in blocking mode */
                    ui.iCountSupport.hidden = true;
                }
                else
                {
                    ui.iCountSupport.hidden = false;
                }

            },
            description: "This determines whether the driver operates synchronously or asynchronously",
        },
        {
            name: "transferCallbackFxn",
            displayName: "Transfer Callback",
            default: "NULL",
            hidden: true,
            description: "Transfer callback function when callback mode is selected",
        },
        {
            name: "iCountSupport",
            displayName: "Enable Multi ICount Support",
            default: false,
            hidden: false,
            description: "Support for multi icount in one transfer to achieve high throughput, this \
                          is only supported in blocking mode ",
        },
        {
            name: "transferTimeout",
            displayName: "Transfer Timeout",
            default: 0xFFFFFFFF,
            description: "Transfer timeout in system ticks. Provide 0xFFFFFFFF to wait forever",
            displayFormat: "hex",
        },
        {
            name: "pinMode",
            displayName: "Pin Mode",
            default: "4PIN_CS",
            options: [
                {
                    name: "3PIN",
                    displayName: "3 Pin Mode"
                },
                {
                    name: "4PIN_CS",
                    displayName: "4 Pin Mode with CS"
                },
            ],
            description: "3 pin mode: Chip-select (CS) is not used and all related options to CS have no meaning. 4 pin mode: CS is used.",
        },
        {
            name: "shiftFormat",
            displayName: "Shift Format",
            default: "MSB_FIRST",
            options: [
                {
                    name: "MSB_FIRST",
                    displayName: "MSB shift out first"
                },
                {
                    name: "LSB_FIRST",
                    displayName: "LSB shift out first"
                },
            ],
            description: "SPI Data shift format.",
        },
        {
            name: "dmaEnable",
            displayName: "Enable DMA",
            default: true,
            description: `Enable data transfer using DMA`,
            onChange: function (inst, ui) {
                if(inst.dmaEnable == true) {
                    ui.numDmaReqLines.hidden = false;
                }
                else
                {
                    ui.numDmaReqLines.hidden = true;
                }
            },
        },
        {
            name: "numDmaReqLines",
            displayName: "DMA REQ Lines",
            default: 1,
            hidden: false,
            description: `For each instance Maximum 3 DMA REQ lines are supported`,
            displayFormat: "dec",
        },
        {
            name: "numTransferGroups",
            displayName: "Number of Transfer Groups",
            default: 1,
            description: `Number of transfer groups: 1 to 8`,
            displayFormat: "dec",
        },
        {
            name: "advanced",
            displayName: "Show Advanced Config",
            default: false,
            onChange: function (inst, ui) {
                let hideConfigs = true;
                if(inst.advanced == true) {
                    hideConfigs = false;
                }
                ui.numParallelModePins.hidden = hideConfigs;
                ui.featureBitMap.hidden = hideConfigs;
                ui.txDummyValue.hidden = hideConfigs;
                ui.eccEnable.hidden = hideConfigs;
                ui.csHold.hidden = hideConfigs;
                ui.compatibilityMode.hidden = hideConfigs;
            },
        },
        {
            name: "numParallelModePins",
            displayName: "Parallel Mode Operation",
            default: "DIS",
            hidden: true,
            options: [
                {
                    name: "ENA",
                    displayName: "Enable"
                },
                {
                    name: "DIS",
                    displayName: "Disable"
                },
            ],
            description: `Paralled mode feature: value 0 to disable, 1 to enable this feature`,
        },
        {
            name: "featureBitMap",
            displayName: "Bit Maps",
            default: "DIS",
            hidden: true,
            options: [
                {
                    name: "ENA",
                    displayName: "Enable"
                },
                {
                    name: "DIS",
                    displayName: "Disable"
                },
            ],
            description: `Bit Map Feature: value 0 to disable, 4 to enable this feature`,
            hidden: true,
        },
        {
            name: "txDummyValue",
            displayName: "Transmit Dummy Value",
            default: 0xFFFF,
            hidden: true,
            description: `Transmit Dummy Value`,
            displayFormat: "hex",
        },
        {
            name: "eccEnable",
            displayName: "Enable ECC",
            default: false,
            hidden: true,
            description: `Enable ECC mode`,
        },
        {
            name: "csHold",
            displayName: "CS Hold Enable",
            default: false,
            hidden: true,
            description: `Enable CS Hold`,
        },
        {
            name: "compatibilityMode",
            displayName: "Enable Compatibilty Mode",
            default: false,
            hidden: true,
            description: `Enable Compatibility mode operation of MIBSPI. MIBSPI RAM will be disabled`,
        },
        common.ui.makeInstanceConfig(getConfigArr()),
    ],
    validate: validate,
    validatePinmux: validatePinmux,
    sharedModuleInstances: addModuleInstances,
    getInstanceConfig,
    moduleInstances: moduleInstances,
    pinmuxRequirements,
    getMaxChannels,
    getInterfaceName,
    getPeripheralPinNames,
    getClockEnableIds,
    getClockFrequencies,
};

function addModuleInstances(inst) {
    let instConfig = getInstanceConfig(inst);
    let edmaInst = instConfig.edmaInst;
    let modInstances = new Array();

        modInstances.push({
            name: "edmaConfig",
            displayName: "EDMA",
            moduleName: "/drivers/edma/edma",
            requiredArgs: {
                instance: edmaInst,
            },
        });

    return modInstances;
}

/*
 *  ======== validate ========
 */
function validate(inst, report) {

    common.validate.checkValidCName(inst, report, "transferCallbackFxn");


    if((inst.transferMode == "CALLBACK") &&
        ((inst.transferCallbackFxn == "NULL") ||
            (inst.transferCallbackFxn == ""))) {
        report.logError("Callback function MUST be provided for callback transfer mode", inst, "transferCallbackFxn");
    }
    common.validate.checkNumberRange(inst, report, "transferTimeout", 0x0, 0xFFFFFFFF, "hex");
    common.validate.checkNumberRange(inst, report, "txDummyValue", 0x0, 0xFFFF, "hex");
    common.validate.checkNumberRange(inst, report, "t2cDelay", 0, 0xFF, "hex");
    common.validate.checkNumberRange(inst, report, "c2tDelay", 0, 0xFF, "hex");
    common.validate.checkNumberRange(inst, report, "wDelay", 0, 0xFF, "hex");
    common.validate.checkNumberRange(inst, report, "numDmaReqLines", 1, 3, "dec");
    common.validate.checkNumberRange(inst, report, "numTransferGroups", 1, 8, "dec");
}

/*
 *  ======== validatePinmux ========
 */
function validatePinmux(inst, report) {
    if (inst.dmaEnable == true) {
        /* Validate if the dma channels required for mibspi are allocated and reserved. */
        let instConfig = getInstanceConfig(inst);
        for(let j = 0; j < inst.numDmaReqLines; j++) {
            let rxEvtFound = false;
            let txEvtFound = false;
            for(let i = 0; i < inst.edmaConfig.edmaRmDmaCh.length; i++) {
                if((inst.edmaConfig.edmaRmDmaCh[i].startIndex <= instConfig.edmaEvtRx[j]) &&
                (inst.edmaConfig.edmaRmDmaCh[i].endIndex >= instConfig.edmaEvtRx[j])) {
                    rxEvtFound = true;
                }
                if((inst.edmaConfig.edmaRmDmaCh[i].startIndex <= instConfig.edmaEvtTx[j]) &&
                (inst.edmaConfig.edmaRmDmaCh[i].endIndex >= instConfig.edmaEvtTx[j])) {
                    txEvtFound = true;
                }
            }
            if (rxEvtFound == false) {
                report.logError(`Allocate dma channel no: ${instConfig.edmaEvtRx[j]} in "Own Dma Channel Resource Manager" in "EDMA" Configuration`, inst, "dmaEnable");
            }
            if (txEvtFound == false) {
                report.logError(`Allocate dma channel no: ${instConfig.edmaEvtTx[j]} in "Own Dma Channel Resource Manager" in "EDMA" Configuration`, inst, "dmaEnable");
            }
        }
        for(let j = 0; j < inst.numDmaReqLines; j++) {
            let rxEvtRsvd = false;
            let txEvtRsvd = false;
            for(let i = 0; i < inst.edmaConfig.edmaRmReservedDmaCh.length; i++) {
                if((inst.edmaConfig.edmaRmReservedDmaCh[i].startIndex <= instConfig.edmaEvtRx[j]) &&
                (inst.edmaConfig.edmaRmReservedDmaCh[i].endIndex >= instConfig.edmaEvtRx[j])) {
                    rxEvtRsvd = true;
                }
                if((inst.edmaConfig.edmaRmReservedDmaCh[i].startIndex <= instConfig.edmaEvtTx[j]) &&
                (inst.edmaConfig.edmaRmReservedDmaCh[i].endIndex >= instConfig.edmaEvtTx[j])) {
                    txEvtRsvd = true;
                }
            }
            if (rxEvtRsvd == false) {
                report.logInfo(`Allocate dma channel no: ${instConfig.edmaEvtRx[j]} in "Reserved Dma Channel Resource Manager" in "EDMA" Configuration for event triggered transfers`, inst, "dmaEnable");
            }
            if (txEvtRsvd == false) {
                report.logInfo(`Allocate dma channel no: ${instConfig.edmaEvtTx[j]} in "Reserved Dma Channel Resource Manager" in "EDMA" Configuration for event triggered transfers`, inst, "dmaEnable");
            }
        }
    }
}

/*
 *  ======== moduleInstances ========
 */
function moduleInstances(inst) {
    let modInstances = new Array();

    let maxCh = getMaxChannels(inst);
    modInstances.push({
        name: "mibspiChannel",
        displayName: "MIBSPI Channel Configuration",
        moduleName: '/drivers/mibspi/v0/mibspi_v0_channel',
        useArray: true,
        maxInstanceCount: maxCh,
        minInstanceCount: 1,
        defaultInstanceCount: 1,
        args: {
        interfaceName: getInterfaceName(inst),
        pinMode: inst.pinMode,
        },

    });

    return (modInstances);
}

exports = mibspi_module;


