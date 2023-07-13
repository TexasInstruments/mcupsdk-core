let common = system.getScript("/common");
let hwi = system.getScript("/kernel/dpl/hwi.js");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/gpmc/soc/gpmc_${common.getSocName()}`);

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

    return "GPMC";
}

function getPeripheralPinNames(inst) {
    return [ "CLK", "CSn0", "CSn1", "CSn2", "CSn3", "DIR", "OEn_REn", "WAIT0", "WAIT1", "WEn", "WPn", "BE0n_CLE", "ADVn_ALE",
                "AD0", "AD1", "AD2", "AD3", "AD4", "AD5", "AD6", "AD7", "AD8", "AD9", "AD10", "AD11", "AD12", "AD13", "AD14", "AD15" ];
}

function getDmaRestrictedRegions() {
    return soc.getDmaRestrictedRegions();
}

function getGpmcDeviceConfig(moduleInstance) {

    let deviceConfig = {};

    switch(moduleInstance.deviceType)
    {
        default:
        case "NANDLIKE":
            return soc.getNandlikeGpmcConfig();
            break;
    }

    return deviceConfig;
}


function pinmuxRequirements(inst) {
   let interfaceName = getInterfaceName(inst);

    let resources = [];
    let pinResource = {};

    pinResource = pinmux.getPinRequirements(interfaceName, "OEn_REn", "GPMC OEn_REn Pin");
    pinmux.setConfigurableDefault( pinResource, "rx", false );
    resources.push( pinResource);

    pinResource = pinmux.getPinRequirements(interfaceName, "ADVn_ALE", "GPMC ADVn_ALE Pin");
    pinmux.setConfigurableDefault( pinResource, "rx", false );
    resources.push( pinResource);

    pinResource = pinmux.getPinRequirements(interfaceName, "WEn", "GPMC WEn Pin");
    pinmux.setConfigurableDefault( pinResource, "rx", false );
    resources.push( pinResource);

    pinResource = pinmux.getPinRequirements(interfaceName, "BE0n_CLE", "GPMC BE0n_CLE Pin");
    pinmux.setConfigurableDefault( pinResource, "rx", false );
    resources.push( pinResource);



    switch(inst.chipSelect)
    {
        default:
        case "CS0":
            pinResource = pinmux.getPinRequirements(interfaceName, "CSn0", "GPMC CS0 Pin");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);
            break;
        case "CS1":
            pinResource = pinmux.getPinRequirements(interfaceName, "CSn1", "GPMC CS1 Pin");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);
            break;
        case "CS2":
            pinResource = pinmux.getPinRequirements(interfaceName, "CSn2", "GPMC CS2 Pin");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);
            break;
        case "CS3":
            pinResource = pinmux.getPinRequirements(interfaceName, "CSn3", "GPMC CS3 Pin");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);
            break;
    }

    switch(inst.deviceType)
    {
        default:
        case "NAND":
            switch(inst.deviceWidth)
            {
                default:
                case "8 bit":
                    pinResource = pinmux.getPinRequirements(interfaceName, "AD0", "GPMC Data I/O Pin0");
                    pinmux.setConfigurableDefault( pinResource, "rx", true );
                    resources.push( pinResource);
                    pinResource = pinmux.getPinRequirements(interfaceName, "AD1", "GPMC Data I/O Pin1");
                    pinmux.setConfigurableDefault( pinResource, "rx", true );
                    resources.push( pinResource);
                    pinResource = pinmux.getPinRequirements(interfaceName, "AD2", "GPMC Data I/O Pin2");
                    pinmux.setConfigurableDefault( pinResource, "rx", true );
                    resources.push( pinResource);
                    pinResource = pinmux.getPinRequirements(interfaceName, "AD3", "GPMC Data I/O Pin3");
                    pinmux.setConfigurableDefault( pinResource, "rx", true );
                    resources.push( pinResource);
                    pinResource = pinmux.getPinRequirements(interfaceName, "AD4", "GPMC Data I/O Pin4");
                    pinmux.setConfigurableDefault( pinResource, "rx", true );
                    resources.push( pinResource);
                    pinResource = pinmux.getPinRequirements(interfaceName, "AD5", "GPMC Data I/O Pin5");
                    pinmux.setConfigurableDefault( pinResource, "rx", true );
                    resources.push( pinResource);
                    pinResource = pinmux.getPinRequirements(interfaceName, "AD6", "GPMC Data I/O Pin6");
                    pinmux.setConfigurableDefault( pinResource, "rx", true );
                    resources.push( pinResource);
                    pinResource = pinmux.getPinRequirements(interfaceName, "AD7", "GPMC Data I/O Pin7");
                    pinmux.setConfigurableDefault( pinResource, "rx", true );
                    resources.push( pinResource);
                    break;
            }


    }

    switch(inst.waitPinselect)
    {
        default:
        case "WAIT0 PIN":
            pinResource = pinmux.getPinRequirements(interfaceName, "WAIT0", "GPMC WAIT0 Pin");
            pinmux.setConfigurableDefault( pinResource, "rx", true );
            resources.push( pinResource);
            break;
        case "WAIT1 PIN":
            pinResource = pinmux.getPinRequirements(interfaceName, "WAIT1", "GPMC WAIT1 Pin");
            pinmux.setConfigurableDefault( pinResource, "rx", true );
            resources.push( pinResource);
            break;

    }

    let peripheral = {
        name: interfaceName,
        displayName: "GPMC Instance",
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

let gpmc_module_name = "/drivers/gpmc/gpmc";

let gpmc_module = {
    displayName: "GPMC",

    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/gpmc/templates/gpmc_config.c.xdt",
            driver_init: "/drivers/gpmc/templates/gpmc_init.c.xdt",
            driver_deinit: "/drivers/gpmc/templates/gpmc_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/gpmc/templates/gpmc.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/gpmc/templates/gpmc_open_close_config.c.xdt",
            driver_open: "/drivers/gpmc/templates/gpmc_open.c.xdt",
            driver_close: "/drivers/gpmc/templates/gpmc_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/gpmc/templates/gpmc_open_close.h.xdt",
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: gpmc_module_name,
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: gpmc_module_name,
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_GPMC",
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
            name: "clockRateDiv",
            displayName: "Input Clock Divider",
            description: "GPMC Output Clock = Input Clock Frequency / Input Clock Divider",
            default: soc.getDefaultConfig().clockRateDiv,
        },
        {
            name: "deviceWidth",
            displayName: "Device Size",
            default: "8 bit",
            options: [
                { name: "8 bit" },
            ]
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
            name: "dmaEnable",
            displayName: "Enable DMA",
            default: true,
            description: `Enable data transfer using DMA`,
        },
        {
            name: "chipSelBaseAddr",
            displayName: "CS Base Address",
            description: "Chip select base address",
            default: "0x50000000",
        },
        {
            name: "chipSelAddrSize",
            displayName: "CS Address Size",
            description: "Chip select address size",
            default: "16MB",
            options: [
                { name: "16MB" },
                { name: "32MB" },
                { name: "64MB" },
                { name: "128MB" },
                { name: "256MB" },
            ]
        },
        /* Advanced parameters */
        {
            name: "advanced",
            displayName: "Advanced Config",
            default: false,
            onChange: function (inst, ui) {
                let hideConfigs = true;
                if(inst.advanced == true) {
                    hideConfigs = false;
                }
                ui.intrEnable.hidden = hideConfigs;
                ui.intrPriority.hidden = hideConfigs;
                ui.waitPinselect.hidden = hideConfigs;
                ui.waitPinPol.hidden = hideConfigs;
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
        {
            name: "waitPinselect",
            displayName: "Wait Pin",
            default: "WAIT0 PIN",
            options: [
                { name: "WAIT0 PIN" },
                { name: "WAIT1 PIN" },
            ],
            hidden: true,
        },
        {
            name: "waitPinPol",
            displayName: "Wait Pin Polarity",
            default: "ACTIVE LOW POLARITY",
            options: [
                { name: "ACTIVE LOW POLARITY" },
                { name: "ACTIVE HIGH POLARITY" },
            ],
            hidden: true,
        },
        /* Timing parameters */
        {
            name: "timingParams",
            displayName: "Timing Config",
            description: "Refer section 7.10.5.8.3 in the datasheet",
            default: false,
            onChange: function (inst, ui) {
                let hideConfigs = true;
                if(inst.timingParams == true) {
                    hideConfigs = false;
                }
                ui.csOnTime.hidden = hideConfigs;
                ui.csRdOffTime.hidden = hideConfigs;
                ui.csWrOffTime.hidden = hideConfigs;
                ui.advOnTime.hidden = hideConfigs;
                ui.advRdOffTime.hidden = hideConfigs;
                ui.advWrOffTime.hidden = hideConfigs;
                ui.advAadMuxOnTime.hidden = hideConfigs;
                ui.advAadMuxRdOffTime.hidden = hideConfigs;
                ui.advAadMuxWrOffTime.hidden = hideConfigs;
                ui.weOnTtime.hidden = hideConfigs;
                ui.weOffTime.hidden = hideConfigs;
                ui.oeOnTime.hidden = hideConfigs;
                ui.oeOffTime.hidden = hideConfigs;
                ui.oeAadMuxOnTime.hidden = hideConfigs;
                ui.oeAadMuxOffTime.hidden = hideConfigs;
                ui.pageBurstAccess.hidden = hideConfigs;
                ui.rdAccessTime.hidden = hideConfigs;
                ui.wrAcessTime.hidden = hideConfigs;
                ui.rdCycleTime.hidden = hideConfigs;
                ui.wrCycleTime.hidden = hideConfigs;
                ui.wrDataOnMuxBusTime.hidden = hideConfigs;
                ui.busTurnAroundTime.hidden = hideConfigs;
                ui.cycle2CycleDelay.hidden = hideConfigs;
                ui.cycleDelaySameChipSel.hidden = hideConfigs;
                ui.cycleDelayDiffChipSel.hidden = hideConfigs;
                ui.addrDataMux.hidden = hideConfigs;
                ui.timeLatency.hidden = hideConfigs;
                ui.eccAlgo.hidden = hideConfigs;
                ui.readType.hidden = hideConfigs;
                ui.csExDelay.hidden = hideConfigs;
                ui.accessType.hidden = hideConfigs;
            },
        },
        {
            name: "csOnTime",
            displayName: "Chip Select Assertion Time",
            description: "Assertion time from start cycle time in GPMC_FCLK cycles",
            default: soc.getNandlikeGpmcConfig().timingParams.csOnTime,
            hidden: true,
        },
        {
            name: "csRdOffTime",
            displayName: "Chip Select Read Deassertion Time",
            description: "De-assertion time from start cycle time for read accesses in GPMC_FCLK cycles",
            default: soc.getNandlikeGpmcConfig().timingParams.csRdOffTime,
            hidden: true,
        },
        {
            name: "csWrOffTime",
            displayName: "Chip Select Write Deassertion Time",
            description: "De-assertion time from start cycle time for write accesses in GPMC_FCLK cycles",
            default: soc.getNandlikeGpmcConfig().timingParams.csWrOffTime,
            hidden: true,
        },
        {
            name: "advOnTime",
            displayName: "ADV Assertion Time",
            description: "Assertion time from start cycle time in GPMC_FCLK cycles",
            default: soc.getNandlikeGpmcConfig().timingParams.advOnTime,
            hidden: true,
        },
        {
            name: "advRdOffTime",
            displayName: "ADV Read Deassertion Time",
            description: "De-assertion time in GPMC_FCLK cycles from start cycle time for read accesses",
            default: soc.getNandlikeGpmcConfig().timingParams.advRdOffTime,
            hidden: true,
        },
        {
            name: "advWrOffTime",
            displayName: "ADV Write Deassertion Time",
            description: "De-assertion time in GPMC_FCLK cycles from start cycle time for write accesses",
            default: soc.getNandlikeGpmcConfig().timingParams.advWrOffTime,
            hidden: true,
        },
        {
            name: "advAadMuxOnTime",
            displayName: "AADMux ADV Assertion Time",
            description: "Assertion time in GPMC_FCLK cycles for first address phase when using the AAD-Multiplexed protocol",
            default: soc.getNandlikeGpmcConfig().timingParams.advAadMuxOnTime,
            hidden: true,
        },
        {
            name: "advAadMuxRdOffTime",
            displayName: "AADMux ADV Read Deassertion Time",
            description: "De-assertion time in GPMC_FCLK cycles for first address phase when using the AAD-Multiplexed read prorocol",
            default: soc.getNandlikeGpmcConfig().timingParams.advAadMuxRdOffTime,
            hidden: true,
        },
        {
            name: "advAadMuxWrOffTime",
            displayName: "AADMux ADV Write Deassertion Time",
            description: "De-assertion time in GPMC_FCLK cycles for first address phase when using the AAD-Multiplexed write prorocol",
            default: soc.getNandlikeGpmcConfig().timingParams.advAadMuxWrOffTime,
            hidden: true,
        },
        {
            name: "weOnTtime",
            displayName: "WE Assertion Time",
            description: "Assertion time in GPMC_FCLK cycles from start cycle time",
            default: soc.getNandlikeGpmcConfig().timingParams.weOnTtime,
            hidden: true,
        },
        {
            name: "weOffTime",
            displayName: "WE Deassertion Time",
            description: "De-assertion time in GPMC_FCLK cycles from start cycle time",
            default: soc.getNandlikeGpmcConfig().timingParams.weOffTime,
            hidden: true,
        },
        {
            name: "oeOnTime",
            displayName: "OE Assertion Time",
            description: "Assertion time in GPMC_FCLK cycles from start cycle time",
            default: soc.getNandlikeGpmcConfig().timingParams.oeOnTime,
            hidden: true,
        },
        {
            name: "oeOffTime",
            displayName: "OE Deassertion Time",
            description: "De-assertion time in GPMC_FCLK cycles from start cycle time",
            default: soc.getNandlikeGpmcConfig().timingParams.oeOffTime,
            hidden: true,
        },
        {
            name: "oeAadMuxOnTime",
            displayName: "AADMux OE Assertion Time",
            description: "Assertion time in GPMC_FCLK cycles for first address phase when using the AAD-Mux prorocol",
            default: soc.getNandlikeGpmcConfig().timingParams.oeAadMuxOnTime,
            hidden: true,
        },
        {
            name: "oeAadMuxOffTime",
            displayName: "AADMux OE Deassertion Time",
            description: "De-assertion time in GPMC_FCLK cycles for first address phase when using the AAD-Mux prorocol",
            default: soc.getNandlikeGpmcConfig().timingParams.oeAadMuxOffTime,
            hidden: true,
        },
        {
            name: "pageBurstAccess",
            displayName: "Page Burst Access Delay",
            description: "Multiple access word delay",
            default: soc.getNandlikeGpmcConfig().timingParams.pageBurstAccess,
            hidden: true,
        },
        {
            name: "rdAccessTime",
            displayName: "Read Access Time",
            description: "Start-cycle to first valid delay",
            default: soc.getNandlikeGpmcConfig().timingParams.rdAccessTime,
            hidden: true,
        },
        {
            name: "wrAcessTime",
            displayName: "Write Access Time",
            description: "Delay from StartAccessTime to the CLK rising edge",
            default: soc.getNandlikeGpmcConfig().timingParams.wrAcessTime,
            hidden: true,
        },
        {
            name: "rdCycleTime",
            displayName: "Read Cycle Time",
            description: "Total read cycle time",
            default: soc.getNandlikeGpmcConfig().timingParams.rdCycleTime,
            hidden: true,
        },
        {
            name: "wrCycleTime",
            displayName: "Write Cycle Time",
            description: "Total write cycle time",
            default: soc.getNandlikeGpmcConfig().timingParams.wrCycleTime,
            hidden: true,
        },
        {
            name: "wrDataOnMuxBusTime",
            displayName: "Write Data On Mux Bus Time",
            description: "Time taken to write Data on Mux Bus",
            default: soc.getNandlikeGpmcConfig().timingParams.wrDataOnMuxBusTime,
            hidden: true,
        },
        {
            name: "busTurnAroundTime",
            displayName: "Bus Turn Around Time",
            description: "Time between two successive accesses to the same chip-select (read to write) or to a different chip-select(read to read and read to write) in GPMC_FCLK cycles",
            default: soc.getNandlikeGpmcConfig().timingParams.busTurnAroundTime,
            hidden: true,
        },
        {
            name: "cycle2CycleDelay",
            displayName:"Cycle To Cycle Delay",
            description: "Chip Select high pulse delay between two successive accesses",
            default: soc.getNandlikeGpmcConfig().timingParams.cycle2CycleDelay,
            hidden: true,
        },
        {
            name: "cycleDelaySameChipSel",
            displayName: "Same Chip Cycle Delay",
            description: "Value to control adding of cycle to cycle delay between two successive accesses to the same chip select",
            default: soc.getNandlikeGpmcConfig().timingParams.cycleDelaySameChipSel,
            hidden: true,
        },
        {
            name: "cycleDelayDiffChipSel",
            displayName: "Different Chip Cycle Delay",
            description: "Value to control adding of cycle to cycle delay between two successive accesses to the different chip select",
            default: soc.getNandlikeGpmcConfig().timingParams.cycleDelayDiffChipSel,
            hidden: true,
        },
        {
            name: "addrDataMux",
            displayName: "Data Address Multiplex",
            description: "Address and data multiplexed protocol",
            default: soc.getNandlikeGpmcConfig().addrDataMux,
            hidden: true,
        },
        {
            name: "timeLatency",
            displayName: "Latency Time",
            description: "Latency time for current Active chip select in use by the memory controller",
            default: soc.getNandlikeGpmcConfig().timeLatency,
            hidden: true,
        },
        {
            name: "eccAlgo",
            displayName: "ECC Algorithm",
            description: "Algorithm supported by the controller",
            default: soc.getNandlikeGpmcConfig().eccAlgo,
            hidden: true,
        },
        {
            name: "readType",
            displayName: "Read Operation Mode",
            description: "Use read operation",
            default: soc.getNandlikeGpmcConfig().readType,
            hidden: true,
        },
        {
            name: "csExDelay",
            displayName: "Chip Select Extra Delay Flag",
            description: "Extra delay support",
            default: soc.getNandlikeGpmcConfig().csExDelay,
            hidden: true,
        },
        {
            name: "accessType",
            displayName: "Access Type",
            description: "Memory access type",
            default: soc.getNandlikeGpmcConfig().accessType,
            hidden: true,
        },
    ],
    sharedModuleInstances: addModuleInstances,
    pinmuxRequirements,
    getInstanceConfig,
    getInterfaceName,
    getPeripheralPinNames,
    getClockEnableIds,
    getClockFrequencies,
    getGpmcDeviceConfig,
    getDmaRestrictedRegions,
};

function addModuleInstances(instance) {
    let modInstances = new Array();

    if(instance.dmaEnable == true) {
        modInstances.push({
            name: "udmaDriver",
            displayName: "UDMA Configuration",
            moduleName: "/drivers/udma/udma",
        });
    }

    return modInstances;
}

function validate(inst, report) {

    common.validate.checkNumberRange(inst, report, "intrPriority", 0, hwi.getHwiMaxPriority(), "dec");
    common.validate.checkNumberRange(inst, report, "clockRateDiv", 1, 4, "dec");
}


exports = gpmc_module;