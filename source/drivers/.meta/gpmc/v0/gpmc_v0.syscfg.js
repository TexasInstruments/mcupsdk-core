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
    return [ "CLK", "CSn0", "CSn1", "CSn2", "CSn3", "DIR", "OEn_REn", "WAIT0", "WAIT1", "WEn", "WPn", "BE0n_CLE", "ADVn_ALE", "BE1n",
                "AD0", "AD1", "AD2", "AD3", "AD4", "AD5", "AD6", "AD7", "AD8", "AD9", "AD10", "AD11", "AD12", "AD13", "AD14", "AD15", "A0", "A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8", "A9", "A10", "A11", "A12", "A13", "A14", "A15", "A16", "A17", "A18", "A19", "A20", "A21"];
}

function getDmaRestrictedRegions() {
    return soc.getDmaRestrictedRegions();
}

function getGpmcDeviceConfig(moduleInstance) {

    let deviceConfig = {};

    switch(moduleInstance.deviceType)
    {
        case "PSRAM":
            return soc.getPsramGpmcConfig();
            break;
        default:
        case "PARALLEL_NAND":
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
        case "PSRAM":
            pinResource = pinmux.getPinRequirements(interfaceName, "BE1n", "GPMC_BE1n");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

            pinResource = pinmux.getPinRequirements(interfaceName, "BE0n_CLE", "GPMC BE0n_CLE Pin");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);
        case "PARALLEL_NOR":        //TODO: BE pins to NOR also??
            /* Address Lines */
            pinResource = pinmux.getPinRequirements(interfaceName, "A1", "GPMC_A1");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

            pinResource = pinmux.getPinRequirements(interfaceName, "A2", "GPMC_A2");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

            pinResource = pinmux.getPinRequirements(interfaceName, "A3", "GPMC_A3");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

            pinResource = pinmux.getPinRequirements(interfaceName, "A4", "GPMC_A4");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

            pinResource = pinmux.getPinRequirements(interfaceName, "A5", "GPMC_A5");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

            pinResource = pinmux.getPinRequirements(interfaceName, "A6", "GPMC_A6");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

            pinResource = pinmux.getPinRequirements(interfaceName, "A7", "GPMC_A7");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

            pinResource = pinmux.getPinRequirements(interfaceName, "A8", "GPMC_A8");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

            pinResource = pinmux.getPinRequirements(interfaceName, "A9", "GPMC_A9");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

            pinResource = pinmux.getPinRequirements(interfaceName, "A10", "GPMC_A10");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

            pinResource = pinmux.getPinRequirements(interfaceName, "A11", "GPMC_A11");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

            pinResource = pinmux.getPinRequirements(interfaceName, "A12", "GPMC_A12");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

            pinResource = pinmux.getPinRequirements(interfaceName, "A13", "GPMC_A13");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

            pinResource = pinmux.getPinRequirements(interfaceName, "A14", "GPMC_A14");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

            pinResource = pinmux.getPinRequirements(interfaceName, "A15", "GPMC_A15");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

            pinResource = pinmux.getPinRequirements(interfaceName, "A16", "GPMC_A16");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

            pinResource = pinmux.getPinRequirements(interfaceName, "A17", "GPMC_A17");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

            pinResource = pinmux.getPinRequirements(interfaceName, "A18", "GPMC_A18");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

            pinResource = pinmux.getPinRequirements(interfaceName, "A19", "GPMC_A19");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

            pinResource = pinmux.getPinRequirements(interfaceName, "A20", "GPMC_A20");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

            pinResource = pinmux.getPinRequirements(interfaceName, "A21", "GPMC_A21");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);


            switch(inst.deviceWidth)
            {
                case "8 bit":
                    /* Address Lines */
                    pinResource = pinmux.getPinRequirements(interfaceName, "A0", "GPMC_A0");
                    pinmux.setConfigurableDefault( pinResource, "rx", false );
                    resources.push( pinResource);

                    /* Data Lines */
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
                default:
                case "16 bit":
                    /* Data Lines */
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
                    pinResource = pinmux.getPinRequirements(interfaceName, "AD8", "GPMC Data I/O Pin8");
                    pinmux.setConfigurableDefault( pinResource, "rx", true );
                    resources.push( pinResource);
                    pinResource = pinmux.getPinRequirements(interfaceName, "AD9", "GPMC Data I/O Pin9");
                    pinmux.setConfigurableDefault( pinResource, "rx", true );
                    resources.push( pinResource);
                    pinResource = pinmux.getPinRequirements(interfaceName, "AD10", "GPMC Data I/O Pin10");
                    pinmux.setConfigurableDefault( pinResource, "rx", true );
                    resources.push( pinResource);
                    pinResource = pinmux.getPinRequirements(interfaceName, "AD11", "GPMC Data I/O Pin11");
                    pinmux.setConfigurableDefault( pinResource, "rx", true );
                    resources.push( pinResource);
                    pinResource = pinmux.getPinRequirements(interfaceName, "AD12", "GPMC Data I/O Pin12");
                    pinmux.setConfigurableDefault( pinResource, "rx", true );
                    resources.push( pinResource);
                    pinResource = pinmux.getPinRequirements(interfaceName, "AD13", "GPMC Data I/O Pin13");
                    pinmux.setConfigurableDefault( pinResource, "rx", true );
                    resources.push( pinResource);
                    pinResource = pinmux.getPinRequirements(interfaceName, "AD14", "GPMC Data I/O Pin14");
                    pinmux.setConfigurableDefault( pinResource, "rx", true );
                    resources.push( pinResource);
                    pinResource = pinmux.getPinRequirements(interfaceName, "AD15", "GPMC Data I/O Pin15");
                    pinmux.setConfigurableDefault( pinResource, "rx", true );
                    resources.push( pinResource);
                    break;
            }
            break;
        default:
        case "PARALLEL_NAND":
            pinResource = pinmux.getPinRequirements(interfaceName, "BE0n_CLE", "GPMC BE0n_CLE Pin");
            pinmux.setConfigurableDefault( pinResource, "rx", false );
            resources.push( pinResource);

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
        case "WAIT0":
            pinResource = pinmux.getPinRequirements(interfaceName, "WAIT0", "GPMC WAIT0 Pin");
            pinmux.setConfigurableDefault( pinResource, "rx", true );
            resources.push( pinResource);
            break;
        case "WAIT1":
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
            name: "deviceType",
            displayName: "Device Type",
            description: "Type of device connected to GPMC",
            default: "PARALLEL_NAND",
            options: [
                { name: "PARALLEL_NAND", displayName: "Parallel Nand Flash" },
                { name: "PSRAM", displayName: "PSRAM" },
            ],
            onChange: function(inst,ui)
            {
                let hideConfigs = false;
                if(inst.deviceType != "PARALLEL_NAND") {
                    hideConfigs = true;
                }

                /* Timing parameters */

                if(inst.deviceType == "PARALLEL_NAND") {
                    inst.dmaEnable = true;
                    ui.dmaEnable.readOnly = false;
                    inst.csOnTime = soc.getNandlikeGpmcConfig().timingParams.csOnTime;
                    inst.csRdOffTime = soc.getNandlikeGpmcConfig().timingParams.csRdOffTime;
                    inst.csWrOffTime = soc.getNandlikeGpmcConfig().timingParams.csWrOffTime;
                    inst.advOnTime = soc.getNandlikeGpmcConfig().timingParams.advOnTime;
                    inst.advRdOffTime = soc.getNandlikeGpmcConfig().timingParams.advRdOffTime;
                    inst.advWrOffTime = soc.getNandlikeGpmcConfig().timingParams.advWrOffTime;
                    inst.advAadMuxOnTime = soc.getNandlikeGpmcConfig().timingParams.advAadMuxOnTime;
                    inst.advAadMuxRdOffTime = soc.getNandlikeGpmcConfig().timingParams.advAadMuxRdOffTime;
                    inst.advAadMuxWrOffTime = soc.getNandlikeGpmcConfig().timingParams.advAadMuxWrOffTime;
                    inst.weOnTtime = soc.getNandlikeGpmcConfig().timingParams.weOnTtime;
                    inst.weOffTime = soc.getNandlikeGpmcConfig().timingParams.weOffTime;
                    inst.oeOnTime = soc.getNandlikeGpmcConfig().timingParams.oeOnTime;
                    inst.oeOffTime = soc.getNandlikeGpmcConfig().timingParams.oeOffTime;
                    inst.oeAadMuxOnTime = soc.getNandlikeGpmcConfig().timingParams.oeAadMuxOnTime;
                    inst.oeAadMuxOffTime = soc.getNandlikeGpmcConfig().timingParams.oeAadMuxOffTime;
                    inst.pageBurstAccess = soc.getNandlikeGpmcConfig().timingParams.pageBurstAccess;
                    inst.rdAccessTime = soc.getNandlikeGpmcConfig().timingParams.rdAccessTime;
                    inst.wrAcessTime = soc.getNandlikeGpmcConfig().timingParams.wrAcessTime;
                    inst.rdCycleTime = soc.getNandlikeGpmcConfig().timingParams.rdCycleTime;
                    inst.wrCycleTime = soc.getNandlikeGpmcConfig().timingParams.wrCycleTime;
                    inst.wrDataOnMuxBusTime = soc.getNandlikeGpmcConfig().timingParams.wrDataOnMuxBusTime;
                    inst.busTurnAroundTime = soc.getNandlikeGpmcConfig().timingParams.busTurnAroundTime;
                    inst.cycle2CycleDelay = soc.getNandlikeGpmcConfig().timingParams.cycle2CycleDelay;
                    inst.cycleDelaySameChipSel = soc.getNandlikeGpmcConfig().timingParams.cycleDelaySameChipSel;
                    inst.cycleDelayDiffChipSel = soc.getNandlikeGpmcConfig().timingParams.cycleDelayDiffChipSel;
                }else if(inst.deviceType == "PSRAM") {
                    inst.dmaEnable = false;
                    ui.dmaEnable.readOnly = true;
                    inst.csOnTime = soc.getPsramGpmcConfig().timingParams.csOnTime;
                    inst.csRdOffTime = soc.getPsramGpmcConfig().timingParams.csRdOffTime;
                    inst.csWrOffTime = soc.getPsramGpmcConfig().timingParams.csWrOffTime;
                    inst.advOnTime = soc.getPsramGpmcConfig().timingParams.advOnTime;
                    inst.advRdOffTime = soc.getPsramGpmcConfig().timingParams.advRdOffTime;
                    inst.advWrOffTime = soc.getPsramGpmcConfig().timingParams.advWrOffTime;
                    inst.advAadMuxOnTime = soc.getPsramGpmcConfig().timingParams.advAadMuxOnTime;
                    inst.advAadMuxRdOffTime = soc.getPsramGpmcConfig().timingParams.advAadMuxRdOffTime;
                    inst.advAadMuxWrOffTime = soc.getPsramGpmcConfig().timingParams.advAadMuxWrOffTime;
                    inst.weOnTtime = soc.getPsramGpmcConfig().timingParams.weOnTtime;
                    inst.weOffTime = soc.getPsramGpmcConfig().timingParams.weOffTime;
                    inst.oeOnTime = soc.getPsramGpmcConfig().timingParams.oeOnTime;
                    inst.oeOffTime = soc.getPsramGpmcConfig().timingParams.oeOffTime;
                    inst.oeAadMuxOnTime = soc.getPsramGpmcConfig().timingParams.oeAadMuxOnTime;
                    inst.oeAadMuxOffTime = soc.getPsramGpmcConfig().timingParams.oeAadMuxOffTime;
                    inst.pageBurstAccess = soc.getPsramGpmcConfig().timingParams.pageBurstAccess;
                    inst.rdAccessTime = soc.getPsramGpmcConfig().timingParams.rdAccessTime;
                    inst.wrAcessTime = soc.getPsramGpmcConfig().timingParams.wrAcessTime;
                    inst.rdCycleTime = soc.getPsramGpmcConfig().timingParams.rdCycleTime;
                    inst.wrCycleTime = soc.getPsramGpmcConfig().timingParams.wrCycleTime;
                    inst.wrDataOnMuxBusTime = soc.getPsramGpmcConfig().timingParams.wrDataOnMuxBusTime;
                    inst.busTurnAroundTime = soc.getPsramGpmcConfig().timingParams.busTurnAroundTime;
                    inst.cycle2CycleDelay = soc.getPsramGpmcConfig().timingParams.cycle2CycleDelay;
                    inst.cycleDelaySameChipSel = soc.getPsramGpmcConfig().timingParams.cycleDelaySameChipSel;
                    inst.cycleDelayDiffChipSel = soc.getPsramGpmcConfig().timingParams.cycleDelayDiffChipSel;
                }
            }
        },
        {
            name: "deviceWidth",
            displayName: "Device Size",
            default: "8 bit",
            options: [
                { name: "8 bit" },
                { name: "16 bit" },
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
            readOnly: false,
        },
        {
            name: "chipSelBaseAddr",
            displayName: "CS Base Address",
            description: "Chip select base address",
            default: soc.getDefaultConfig().dataBaseAddrVal,
            displayFormat: "hex",
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
            displayName: "Show Advanced Config",
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
                ui.addrDataMux.hidden = hideConfigs;
                ui.timeLatency.hidden = hideConfigs;
                if(inst.deviceType == "PSRAM")
                {
                    ui.enableOptimisedAccess.hidden = true;
                    ui.cycleOptimisation.hidden = true;

                }
                else if(inst.deviceType == "PARALLEL_NAND")
                {
                    ui.enableOptimisedAccess.hidden = hideConfigs;

                    if(inst.enableOptimisedAccess == "ENABLED")
                    {
                        ui.cycleOptimisation.hidden = hideConfigs;
                    }
                    else
                    {
                        ui.cycleOptimisation.hidden = true;
                    }
                }

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
            default: "WAIT0",
            options: [
                { name : "WAIT0", displayName: "WAIT0 PIN" },
                { name : "WAIT1", displayName: "WAIT1 PIN" },
            ],
            hidden: true,
        },
        {
            name: "waitPinPol",
            displayName: "Wait Pin Polarity",
            default: "ACTIVEL",
            options: [
                { name : "ACTIVEL", displayName : "ACTIVE LOW POLARITY" },
                { name : "ACTIVEH", displayName : "ACTIVE HIGH POLARITY" },
            ],
            hidden: true,
        },
        {
            name: "addrDataMux",
            displayName: "Data Address Multiplex",
            longDescription: "Enables the address and data multiplexed protocol",
            default: soc.getNandlikeGpmcConfig().addrDataMux,
            options: [
                { name : "NONMUX" , displayName : "Nonmultiplexed Attached Device"},
                { name : "AADMUX" , displayName : "AAD-Multiplexed Protocol Device"},
                { name : "MUX" , displayName : "Address And Data Multiplexed Device"},
            ],
            hidden : true,
        },
        {
            name: "timeLatency",
            displayName: "Signals Timing Latency",
            longDescription: "Signals timing latencies scalar factor\n" + "\n" +
            "(RD/WRCYCLETIME, RD/WRACCESSTIME, PAGEBURSTACCESSTIME, CSONTIME,\n" + "\n" +
            "CSRD/WROFFTIME, ADVONTIME, ADVRD/WROFFTIME, OEONTIME, OEOFFTIME,\n" + "\n" +
            "WEONTIME, WEOFFTIME, CYCLE2CYCLEDELAY, BUSTURNAROUND,\n" + "\n" +
            "TIMEOUTSTARTVALUE, WRDATAONADMUXBUS)",
            default: soc.getNandlikeGpmcConfig().timeLatency,
            options: [
                { name: "X1", displayName : "1x Latency"},
                { name: "X2", displayName : "2x Latency"},
            ],
            hidden : true,
        },
        {
            name: "cycleOptimisation",
            displayName: "PPE Optimisation Cycles",
            longDescription: "For Prefetch/Postwrite engine, define the number of GPMC_FCLK cycles to be subtracted\n" + "\n" +
            "from RDCYCLETIME, WRCYCLETIME, RDACCESSTIME,\n" + "\n" +
            "CSRDOFFTIME, CSWROFFTIME, ADVRDOFFTIME,\n" + "\n" +
            "ADVWROFFTIME, OEOFFTIME, WEOFFTIME\n" + "\n" +
            "0h = 0 GPMC_FCLK cycle\n" + "\n" +
            "1h = 1 GPMC_FCLK cycle\n" + "\n" +
            "...\n" + "\n" +
            "7h = 7 GPMC_FCLK cycles",
            default: soc.getNandlikeGpmcConfig().cycleOptimisation,
            options: [
                { name: 0, displayName : "0 GPMC FCLK cycles"},
                { name: 1, displayName : "1 GPMC FCLK cycles"},
                { name: 2, displayName : "2 GPMC FCLK cycles"},
                { name: 3, displayName : "3 GPMC FCLK cycles"},
                { name: 4, displayName : "4 GPMC FCLK cycles"},
                { name: 5, displayName : "5 GPMC FCLK cycles"},
                { name: 6, displayName : "6 GPMC FCLK cycles"},
                { name: 7, displayName : "7 GPMC FCLK cycles"},
            ],
            hidden : true,
        },
        {
            name: "enableOptimisedAccess",
            displayName: "PPE Cycle Optimisation",
            longDescription: "Enables access cycle optimization for prefetch and post write engine.",
            default: soc.getNandlikeGpmcConfig().optimisedAccess,
            options: [
                { name: "ENABLED", displayName : "Enable"},
                { name: "DISABLED", displayName : "Disable"},
            ],
            hidden : true,
            onChange: function(inst,ui)
            {
                if(inst.enableOptimisedAccess == "ENABLED")
                {
                    ui.cycleOptimisation.hidden = false;
                }
                else
                {
                    ui.cycleOptimisation.hidden = true;
                    inst.cycleOptimisation = soc.getNandlikeGpmcConfig().cycleOptimisation;
                }
            }
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
        /* Timing parameters */
        {
            name: "timingParams",
            displayName: "GPMC Timing Parameters Config",
            collapsed : true,
            config: [
                {
                    name: "csOnTime",
                    displayName: "Chip Select On Time",
                    longDescription: "Chip Select Assertion time from start cycle time in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.csOnTime,
                },
                {
                    name: "csRdOffTime",
                    displayName: "Chip Select Read Off Time",
                    longDescription: "Chip Select De-assertion time from start cycle time for read accesses in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.csRdOffTime,
                },
                {
                    name: "csWrOffTime",
                    displayName: "Chip Select Write Off Time",
                    longDescription: "Chip Select De-assertion time from start cycle time for write accesses in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.csWrOffTime,
                },
                {
                    name: "advOnTime",
                    displayName: "ADV On Time",
                    longDescription: "Address valid assertion time from start cycle time in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.advOnTime,
                },
                {
                    name: "advRdOffTime",
                    displayName: "ADV Read Off Time",
                    longDescription: "Address valid de-assertion time from start cycle time for read accesses in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.advRdOffTime,
                },
                {
                    name: "advWrOffTime",
                    displayName: "ADV Write Off Time",
                    longDescription: "Address valid de-assertion time from start cycle time for write accesses in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.advWrOffTime,
                },
                {
                    name: "advAadMuxOnTime",
                    displayName: "ADV AADMux On Time",
                    longDescription: "Address valid assertion time for first address phase when using the AAD-Multiplexed protocol in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.advAadMuxOnTime,
                },
                {
                    name: "advAadMuxRdOffTime",
                    displayName: "ADV AADMux Read Off Time",
                    longDescription: "Address valid de-assertion time for first address phase when using the AAD-Multiplexed read prorocol in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.advAadMuxRdOffTime,
                },
                {
                    name: "advAadMuxWrOffTime",
                    displayName: "ADV AADMux Write Off Time",
                    longDescription: "Address valid de-assertion time for first address phase when using the AAD-Multiplexed write prorocol in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.advAadMuxWrOffTime,
                },
                {
                    name: "weOnTtime",
                    displayName: "WE On Time",
                    longDescription: "Write enable assertion time from start cycle time in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.weOnTtime,
                },
                {
                    name: "weOffTime",
                    displayName: "WE Off Time",
                    longDescription: "Write enable de-assertion time from start cycle time in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.weOffTime,
                },
                {
                    name: "oeOnTime",
                    displayName: "OE On Time",
                    longDescription: "Output enable assertion time from start cycle time in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.oeOnTime,
                },
                {
                    name: "oeOffTime",
                    displayName: "OE Off Time",
                    longDescription: "Output enable de-assertion time from start cycle time in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.oeOffTime,
                },
                {
                    name: "oeAadMuxOnTime",
                    displayName: "OE AADMux On Time",
                    longDescription: "Output enable assertion time for first address phase when using the AAD-Mux prorocol in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.oeAadMuxOnTime,
                },
                {
                    name: "oeAadMuxOffTime",
                    displayName: "OE AADMux Off Time",
                    longDescription: "Output enable de-assertion time for first address phase when using the AAD-Mux prorocol in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.oeAadMuxOffTime,
                },
                {
                    name: "pageBurstAccess",
                    displayName: "Page Burst Access Delay",
                    longDescription: "Delay between successive words in a multiple access in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.pageBurstAccess,
                },
                {
                    name: "rdAccessTime",
                    displayName: "Read Access Time",
                    longDescription: "Delay between start cycle time and first data valid in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.rdAccessTime,
                },
                {
                    name: "wrAcessTime",
                    displayName: "Write Access Time",
                    longDescription: "Delay from start access time to the GPMC_FCLK rising edge corresponding the GPMC CLK rising edge used by the attached memory for the first data capture in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.wrAcessTime,
                },
                {
                    name: "rdCycleTime",
                    displayName: "Read Cycle Time",
                    longDescription: "Total read cycle time in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.rdCycleTime,
                },
                {
                    name: "wrCycleTime",
                    displayName: "Write Cycle Time",
                    longDescription: "Total write cycle time in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.wrCycleTime,
                },
                {
                    name: "wrDataOnMuxBusTime",
                    displayName: "Write Data AAD Mux Bus Time",
                    longDescription: "Specifies on which GPMC_FCLK rising edge the first data of the write is driven in the add/data mux bus in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.wrDataOnMuxBusTime,
                },
                {
                    name: "busTurnAroundTime",
                    displayName: "Bus Turn Around Time",
                    longDescription: "Bus turnaround latency between successive accesses to the same chip-select (read to write) or to a different chip-select (read to read and read to write) in GPMC_FCLK cycles",
                    default: soc.getNandlikeGpmcConfig().timingParams.busTurnAroundTime,
                },
                {
                    name: "cycle2CycleDelay",
                    displayName:"Cycle To Cycle Delay",
                    longDescription: "Chip Select high pulse delay between two successive accesses in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.cycle2CycleDelay,
                },
                {
                    name: "cycleDelaySameChipSel",
                    displayName: "Cycle Delay Same Chip Select",
                    longDescription: "Add cycle delay between successive accesses to the same chip-select (any access type) in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.cycleDelaySameChipSel,
                    options : [
                        { name :"NOC2CDELAY", displayName : "No Delay"},
                        { name : "C2CDELAY" , displayName : "Add Cycle To Cycle Delay"}
                    ]
                },
                {
                    name: "cycleDelayDiffChipSel",
                    displayName: "Cycle Delay Different Chip Select",
                    longDescription: "Add cycle delay between successive accesses to a different chip-select (any access type) in GPMC_FCLK cycles.",
                    default: soc.getNandlikeGpmcConfig().timingParams.cycleDelayDiffChipSel,
                    options : [
                        { name :"NOC2CDELAY", displayName : "No Delay"},
                        { name : "C2CDELAY" , displayName : "Add Cycle To Cycle Delay"}
                    ]
                },
            ],

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
        modInstances.push({
            name: "udmaBlkCopyChannel",
            displayName: "UDMA Block Copy Channel Configuration",
            moduleName: '/drivers/udma/udma_blkcopy_channel',
        });
    }

    return modInstances;
}

function validate(inst, report) {

    common.validate.checkNumberRange(inst, report, "intrPriority", 0, hwi.getHwiMaxPriority(), "dec");
    common.validate.checkNumberRange(inst, report, "clockRateDiv", 1, 4, "dec");
}


exports = gpmc_module;
