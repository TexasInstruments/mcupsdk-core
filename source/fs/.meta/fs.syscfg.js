let common = system.getScript("/common");

const topModules = [
    "/fs/freertos_fat/freertos_fat",
];

const topModulesNull = [
];

function getTopModules() {
	if((common.getSocName() == "am64x") || (common.getSocName() == "am243x")) {
		return topModules;
	} else {
		return topModulesNull;
	}
}

exports = {
    displayName: "File System",
    topModules: getTopModules(),
};
