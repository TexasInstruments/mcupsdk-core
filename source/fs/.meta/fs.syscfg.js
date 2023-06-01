let common = system.getScript("/common");

const topModules = [
    "/fs/freertos_fat/freertos_fat",
];

const topModulesNull = [
];

function getTopModules() {
    const fsSocList = ["am64x", "am243x", "am263x", "am263px"];
	if(fsSocList.includes(common.getSocName())) {
		return topModules;
	} else {
		return topModulesNull;
	}
}

exports = {
    displayName: "File System",
    topModules: getTopModules(),
};
