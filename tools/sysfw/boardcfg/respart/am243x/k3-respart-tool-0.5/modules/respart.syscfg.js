const deviceSelected = system.deviceData.device;
const devData = _.keyBy(system.getScript("/data/SOC.json"), (r) => r.soc);
const socName = devData[deviceSelected].shortName;
const pdkUsage = devData[deviceSelected].pdkUsage;
const mcusdkUsage = devData[deviceSelected].mcusdkUsage;
var hosts = system.getScript("/data/" + socName + "/Hosts.json");

exports = {
	displayName: "Resource Partitioning",
	templates: [
		{
			name: "/templates/rm-cfg.syscfg.xdt",
			outputPath: "rm-cfg.c",
			alwaysRun: true,
		},
		{
			name: "/templates/sciclient_defaultBoardcfg_rm.syscfg.xdt",
			outputPath: "sciclient_defaultBoardcfg_rm.c",
			alwaysRun: pdkUsage,
		},
		{
			name: "/templates/sciclient_defaultBoardcfg_rm_mcusdk.syscfg.xdt",
			outputPath: "sciclient_defaultBoardcfg_rm_mcusdk.c",
			alwaysRun: mcusdkUsage,
		},
		{
			name: "/templates/sciclient_tifsBoardcfg_rm.syscfg.xdt",
			outputPath: "sciclient_tifsBoardcfg_rm.c",
			alwaysRun: false,
		},
		{
			name: "/templates/tifs-rm-cfg.syscfg.xdt",
			outputPath: "tifs-rm-cfg.c",
			alwaysRun: true,
		},
		{
			name: "/templates/qos_config.syscfg.xdt",
			outputPath: socName + "_qos_data.c",
			alwaysRun: true,
		},
		{
			name: "/templates/firewall_config.syscfg.xdt",
			outputPath: socName + "_firewall_data.c",
			alwaysRun: true,
		},
		{
			name: "/templates/sciclient_defaultBoardcfg.syscfg.xdt",
			outputPath: "sciclient_defaultBoardcfg.c",
			alwaysRun: true,
		},
		{
			name: "/templates/sysfw_img_cfg.syscfg.xdt",
			outputPath: "sysfw_img_cfg.h",
			alwaysRun: true,
		},
	],
	topModules: [
		{
			displayName: "SYSFW Board config",
			modules: ["/modules/boardConfig"],
		},
		{
			displayName: "SYSFW Resource Partitioning",
			modules: get_host_modules(),
		},
		{
			displayName: "Resource Sharing",
			modules: ["/modules/resourceSharing"],
		},
		{
			displayName: "Peripheral Resource Partitioning",
			modules: ["/modules/qosConfig", "/modules/firewallConfig"],
		}
	],
	views: [
		{
			name: "/templates/resAllocMarkdown.xdt",
			displayName: "Resource Allocation Markwodn",
			viewType: "markdown",
			ignoreErrors: true,
		},
		{
			name: "/templates/resAllocTable.xdt",
			displayName: "Resource Allocation Table",
			viewType: "markdown",
			ignoreErrors: true,
		},
		{
			name: "/templates/hwaTable.xdt",
			displayName: "HWA Channels Table",
			viewType: "markdown",
			ignoreErrors: true,
		},
	],
};

function get_host_modules() {
	var modules = [];
	for (var idx = 0; idx < hosts.length; idx++) {
		modules.push("/modules/" + socName + "/" + hosts[idx].hostName);
	}
	return modules;
}
