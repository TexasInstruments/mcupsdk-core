let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc    = system.getScript(`/drivers/mmcsd/soc/mmcsd_${common.getSocName()}`);

let gInputClkFreq = soc.getDefaultConfig().inputClkFreq;

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
}

function pinmuxRequirements(instance) {
	let interfaceName = getInterfaceName(instance);
	let resources = [];
    let pinResource = {};
    if(interfaceName == "MMC")
    {
        pinResource = pinmux.getPinRequirements(interfaceName, "MMC_CLK", "MMC1 CLK Pin");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        pinmux.setConfigurableDefault( pinResource, "pu_pd", "nopull" );
        resources.push( pinResource);

        pinResource = pinmux.getPinRequirements(interfaceName, "MMC_SDWP", "MMC1 SDWP Pin");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        pinmux.setConfigurableDefault( pinResource, "pu_pd", "nopull" );
        resources.push( pinResource);

        pinResource = pinmux.getPinRequirements(interfaceName, "MMC_CMD", "MMC1 CMD Pin");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        pinmux.setConfigurableDefault( pinResource, "pu_pd", "pu" );
        resources.push( pinResource);

        pinResource = pinmux.getPinRequirements(interfaceName, "MMC_DAT0", "MMC1 DAT0 Pin");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        pinmux.setConfigurableDefault( pinResource, "pu_pd", "pu" );
        resources.push( pinResource);

        pinResource = pinmux.getPinRequirements(interfaceName, "MMC_DAT1", "MMC1 DAT1 Pin");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        pinmux.setConfigurableDefault( pinResource, "pu_pd", "pu" );
        resources.push( pinResource);

        pinResource = pinmux.getPinRequirements(interfaceName, "MMC_DAT2", "MMC1 DAT2 Pin");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        pinmux.setConfigurableDefault( pinResource, "pu_pd", "pu" );
        resources.push( pinResource);

        pinResource = pinmux.getPinRequirements(interfaceName, "MMC_DAT3", "MMC1 DAT3 Pin");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        pinmux.setConfigurableDefault( pinResource, "pu_pd", "pu" );
        resources.push( pinResource);

        pinResource = pinmux.getPinRequirements(interfaceName, "MMC_SDCD", "MMC1 SDCD Pin");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        pinmux.setConfigurableDefault( pinResource, "pu_pd", "nopull" );
        resources.push( pinResource);
    }
	let peripheral = {
		name          : interfaceName,
        displayName   : "MMCSD Instance",
        interfaceName : interfaceName,
        resources     : resources,
	}

	return [peripheral];
}

function getPeripheralPinNames(inst) {
    return ["MMC_CLK", "MMC_CMD", "MMC_DAT0", "MMC_DAT1", "MMC_DAT2", "MMC_DAT3", "MMC_SDWP", "MMC_SDCD"];
}

function getInterfaceName(inst) {
	return inst.moduleSelect;
}

function getClockEnableIds(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockIds;
}

function getClockFrequencies(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockFrequencies;
}

let mmcsd_module_name = "/drivers/mmcsd/mmcsd";

let mmcsd_module = {
	displayName: "MMCSD",

	templates: {

        "/drivers/system/system_config.c.xdt": {
			driver_config: "/drivers/mmcsd/templates/mmcsd_v1_config.c.xdt",
			driver_init: "/drivers/mmcsd/templates/mmcsd_init.c.xdt",
			driver_deinit: "/drivers/mmcsd/templates/mmcsd_deinit.c.xdt",
		},

		"/drivers/system/system_config.h.xdt": {
			driver_config: "/drivers/mmcsd/templates/mmcsd.h.xdt",
		},
		"/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/mmcsd/templates/mmcsd_open_close_config.c.xdt",
            driver_open: "/drivers/mmcsd/templates/mmcsd_open.c.xdt",
            driver_close: "/drivers/mmcsd/templates/mmcsd_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/mmcsd/templates/mmcsd_open_close.h.xdt",
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: mmcsd_module_name,
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: mmcsd_module_name,
        },
	},
	maxInstances: getConfigArr().length,
	defaultInstanceName: "CONFIG_MMCSD",
	validate: validate,
	config: [
        {
            name: "moduleSelect",
            displayName: "Select MMCSD Module",
            description: "The MMC is usually connected to the SD card slot",
            default: "MMC",
            options: [
                { name: "MMC" },
            ],
            onChange: function (inst, ui) {
                if(inst.moduleSelect == "MMC") {
                    inst.cardType = "SD";
                }
            },
        },
		{
			name: "inputClkFreq",
			displayName: "Input Clock Frequency (Hz)",
			default: gInputClkFreq,
		},
		{
			name: "cardType",
			displayName: "Card Type",
			default: "SD",
            options: [
                { name: "SD" },
                { name: "EMMC" },
                { name: "NO_DEVICE" },
            ],
		},
		{
            name: "intrEnable",
            displayName: "Interrupt Mode Enable",
            description: "NOT tested, DO NOT USE",
            default: false,
            hidden: true,
        },
        {
            name: "dmaEnable",
            displayName: "DMA Enable",
            default: false,
            hidden: true,
        },
        {
            name: "supportedBusVoltages",
            displayName: "Voltage Value",
            options: [
                { name: "VOLTAGE_3_3V" },
                { name: "VOLTAGE_3_0V" },
            ],
            default: "VOLTAGE_3_3V",
            hidden: true,
        },
        {
            name: "supportedBusWidth",
            displayName: "Data Width",
            options: [
                { name: "MMCSD_BUS_WIDTH_4BIT" },
                { name: "MMCSD_BUS_WIDTH_1BIT" },
            ],
            default: "MMCSD_BUS_WIDTH_4BIT",
            hidden: false,
        },
	],
	getInstanceConfig,
	pinmuxRequirements,
	getInterfaceName,
	getPeripheralPinNames,
	getClockEnableIds,
	getClockFrequencies,
};

function validate(inst, report) {

}

exports = mmcsd_module;
