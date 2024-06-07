let common = system.getScript("/common");

const topModules = [
    "/eclipse_threadx/filex/filex",
    "/eclipse_threadx/threadx/threadx",
];

const topModulesNull = [
];

function getTopModules() {
    const fsSocList = ["am243x"];
	if(fsSocList.includes(common.getSocName())) {
		return topModules;
	} else {
		return topModulesNull;
	}
}

exports = {
    displayName: "Eclipse ThreadX",
    templates: [
        {
            name: "/eclipse_threadx/eclipse_threadx/eclipse_threadx_config.h.xdt",
            outputPath: "ti_eclipse_threadx_config.h",
            alwaysRun: false,
        },
        {
            name: "/eclipse_threadx/eclipse_threadx/eclipse_threadx_open_close.c.xdt",
            outputPath: "ti_eclipse_threadx_open_close.c",
            alwaysRun: false,
        },
        {
            name: "/eclipse_threadx/eclipse_threadx/eclipse_threadx_open_close.h.xdt",
            outputPath: "ti_eclipse_threadx_open_close.h",
            alwaysRun: false,
        },
    ],
    topModules: getTopModules(),
};
