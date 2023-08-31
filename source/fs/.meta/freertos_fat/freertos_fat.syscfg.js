let common = system.getScript("/common");

let freertos_fat_module_name = "/fs/freertos_fat/freertos_fat";

let freertos_fat_module = {
	displayName: "FreeRTOS FAT",
	templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/fs/freertos_fat/templates/freertos_fat.h.xdt",
        },
	    "/drivers/system/drivers_open_close.c.xdt": {
	        driver_open_close_config: "/fs/freertos_fat/templates/freertos_fat_open_close_config.c.xdt",
	        driver_open: "/fs/freertos_fat/templates/freertos_fat_open.c.xdt",
	        driver_close: "/fs/freertos_fat/templates/freertos_fat_close.c.xdt",
	    },
	    "/drivers/system/drivers_open_close.h.xdt": {
	        driver_open_close_config: "/fs/freertos_fat/templates/freertos_fat_open_close.h.xdt",
	    },
	},
	defaultInstanceName: "CONFIG_FREERTOS_FAT",
	config: [
		{
			name: "media",
			displayName: "Select Media",
			description: "Select the media which is to be used underneath the virtual file system provided by FreeRTOS FAT",
			default: "SD",
			options: [
				{ name: "SD" },
                { name: "EMMC" },
			]
		},
	],
	moduleInstances: moduleInstances,
};

function moduleInstances(inst) {

    let modInstances = new Array();
    let moduleSelectName = "";

    switch(inst.media) {
    	case "SD":
            moduleSelectName = "MMC1";
            if ((common.getSocName() == "am263x") || (common.getSocName() == "am263px"))
            {
                moduleSelectName = "MMC";
            }
    		modInstances.push({
    		    name: "peripheralDriver",
    		    displayName: "MMCSD Configuration",
    		    moduleName: '/drivers/mmcsd/mmcsd',
    		    useArray: false,
    		    requiredArgs: {
    		        moduleSelect: moduleSelectName,
                    cardType : "SD",
    		    },
    		});
    		break;
    	case "EMMC":
            moduleSelectName = "MMC0";
            if ((common.getSocName() == "am263x") || (common.getSocName() == "am263px"))
            {
                moduleSelectName = "MMC";
            }
    		modInstances.push({
    		    name: "peripheralDriver",
    		    displayName: "MMCSD Configuration",
    		    moduleName: '/drivers/mmcsd/mmcsd',
    		    useArray: false,
    		    requiredArgs: {
    		        moduleSelect: moduleSelectName,
                    cardType : "EMMC",
    		    },
    		});
    		break;
    }

    return (modInstances);
}

exports = freertos_fat_module;