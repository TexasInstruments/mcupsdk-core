let common = system.getScript("/common");

let filex_module_name = "/eclipse_threadx/filex/filex";

let filex_module = {
	displayName: "FileX",
    templates: {
        "/eclipse_threadx/eclipse_threadx/eclipse_threadx_config.h.xdt": {
            config: "/eclipse_threadx/filex/templates/filex.h.xdt",
        },
        "/eclipse_threadx/eclipse_threadx/eclipse_threadx_open_close.c.xdt": {
            open_close_config: "/eclipse_threadx/filex/templates/filex_open_close_config.c.xdt",
            open: "/eclipse_threadx/filex/templates/filex_open.c.xdt",
            close: "/eclipse_threadx/filex/templates/filex_close.c.xdt",
        },
        "/eclipse_threadx/eclipse_threadx/eclipse_threadx_open_close.h.xdt": {
            open_close_config: "/eclipse_threadx/filex/templates/filex_open_close.h.xdt",
        },
    },
	defaultInstanceName: "CONFIG_FILEX",
	config: [
		{
			name: "media",
			displayName: "Select Media",
			description: "Select the media which is to be used underneath the virtual file system provided by FileX",
			default: "SD",
			options: [
				{ name: "SD" },
                { name: "EMMC" },
			]
		},
        {
            name: "auto_format",
            displayName: "Auto-Format",
            description: "Automatically format the media if no file system is found. Be careful: will overwrite content!",
            default: false
        },
        {
            name: "sector_size",
            displayName: "Sector Size",
            description: "Sector size.",
            default: 512,
            options: [
                { name: 512},
                { name: 1024},
                { name: 2048},
                { name: 4096},
            ]
        },
        {
            name: "sectors_per_cluster",
            displayName: "Sectors Per Cluster",
            description: "Number of sectors per FAT clusters.",
            default: 1,
            options: [
                { name: 1},
                { name: 2},
                { name: 3},
                { name: 4},
            ]
        }
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
    }

    return (modInstances);
}

exports = filex_module;