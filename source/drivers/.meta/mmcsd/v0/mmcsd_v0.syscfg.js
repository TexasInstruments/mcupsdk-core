let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc    = system.getScript(`/drivers/mmcsd/soc/mmcsd_${common.getSocName()}`);

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

function getOperatingMode(inst) {

    if((inst.cardType == "EMMC") || (inst.cardType == "NO_DEVICE"))
    {
        switch(inst.modeSelectEMMC)
        {
            default:
            case "HS200":
                return "MMCSD_SUPPORT_MMC_DS | MMCSD_SUPPORT_MMC_HS200";
                break;
            case "DDR50":
                return "MMCSD_SUPPORT_MMC_DS | MMCSD_SUPPORT_MMC_HS_DDR";
                break;
            case "SDR50":
                return "MMCSD_SUPPORT_MMC_DS | MMCSD_SUPPORT_MMC_HS_SDR";
                break;
        }
    }else if(inst.cardType == "SD")
    {
        switch(inst.modeSelectSD)
        {
            default:
                return "MMCSD_SUPPORT_SD_DS | MMCSD_SUPPORT_SD_HS";
                break;
        }
    }

}

function pinmuxRequirements(instance) {
	let interfaceName = getInterfaceName(instance);

	let resources = [];
    let pinResource = {};

    if(interfaceName == "MMC1")
    {
    	pinResource = pinmux.getPinRequirements(interfaceName, "CLK", "MMC1 CLK Pin");
    	pinmux.setConfigurableDefault( pinResource, "rx", true );
        pinmux.setConfigurableDefault( pinResource, "pu_pd", "nopull" );
    	resources.push( pinResource);

        pinResource = pinmux.getPinRequirements(interfaceName, "CLKLB", "MMC1 CLKLB Pin");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        pinmux.setConfigurableDefault( pinResource, "pu_pd", "pu" );
        resources.push( pinResource);

    	pinResource = pinmux.getPinRequirements(interfaceName, "CMD", "MMC1 CMD Pin");
    	pinmux.setConfigurableDefault( pinResource, "rx", true );
    	resources.push( pinResource);

    	pinResource = pinmux.getPinRequirements(interfaceName, "DAT0", "MMC1 DAT0 Pin");
    	pinmux.setConfigurableDefault( pinResource, "rx", true );
    	resources.push( pinResource);

    	pinResource = pinmux.getPinRequirements(interfaceName, "DAT1", "MMC1 DAT1 Pin");
    	pinmux.setConfigurableDefault( pinResource, "rx", true );
    	resources.push( pinResource);

    	pinResource = pinmux.getPinRequirements(interfaceName, "DAT2", "MMC1 DAT2 Pin");
    	pinmux.setConfigurableDefault( pinResource, "rx", true );
    	resources.push( pinResource);

    	pinResource = pinmux.getPinRequirements(interfaceName, "DAT3", "MMC1 DAT3 Pin");
    	pinmux.setConfigurableDefault( pinResource, "rx", true );
    	resources.push( pinResource);

    	pinResource = pinmux.getPinRequirements(interfaceName, "SDCD", "MMC1 SDCD Pin");
    	pinmux.setConfigurableDefault( pinResource, "rx", true );
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

	if(getInterfaceName(inst) == "MMC1") {
		return ["CLK", "CLKLB", "CMD", "DAT0", "DAT1", "DAT2", "DAT3", "SDCD"];
	}

    return [ ];
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
			driver_config: "/drivers/mmcsd/templates/mmcsd_config.c.xdt",
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
            description: "The MMC0 is usually connected to the eMMC device and MMC1 is usually connected to the SD card slot",
            default: "MMC1",
            options: [
                { name: "MMC0" },
                { name: "MMC1" },
            ],
            onChange: function (inst, ui) {
                if(inst.moduleSelect == "MMC0") {
                    inst.cardType = "EMMC";
                    inst.phyType = "HW_PHY";
                    ui.modeSelectEMMC.hidden = false;
                    ui.modeSelectSD.hidden = true;
                } else {
                    inst.cardType = "SD";
                    inst.phyType = "SW_PHY";
                    ui.modeSelectSD.hidden = false;
                    ui.modeSelectEMMC.hidden = true;
                }
            },
        },
        {
            name: "modeSelectEMMC",
            displayName: "EMMC Operating Mode",
            description: "Select the operating mode for EMMC",
            default: soc.getDefaultOperatingModeEMMC().name,
            options: soc.getOperatingModesEMMC(),
            hidden : true,
        },
        {
            name: "modeSelectSD",
            displayName: "SD Operating Mode",
            description: "Select the operating mode for SD",
            default: soc.getDefaultOperatingModeSD().name,
            options: soc.getOperatingModesSD(),
            hidden: false,
        },
		{
			name: "inputClkFreq",
			displayName: "Input Clock Frequency (Hz)",
			default: soc.getDefaultConfig().inputClkFreq,
		},
		{
			name: "cardType",
			displayName: "Card Type",
			default: "EMMC",
            options: [
                { name: "EMMC" },
                { name: "SD" },
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
            default: true,
            hidden: true,
        },
        {
			name: "phyType",
			displayName: "PHY Type",
			options: [
                { name: "HW_PHY" },
                { name: "SW_PHY" },
                { name: "NO_PHY" },
            ],
			default: "HW_PHY",
			hidden: false,
		},
	],
	getInstanceConfig,
	pinmuxRequirements,
	getInterfaceName,
	getPeripheralPinNames,
	getClockEnableIds,
	getClockFrequencies,
    getOperatingMode,
};

function validate(inst, report) {

}

exports = mmcsd_module;
