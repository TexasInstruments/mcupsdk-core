exports = {
	topModules: [{
		displayName: "Memory",
		modules: [
			"memory_configurator/general",
            "memory_configurator/region",
            "memory_configurator/section",
		],
	}],
    templates: [
        {
            name: "/memory_configurator/templates/linker.cmd.xdt",
            outputPath: "linker.cmd",
            alwaysRun: false,
        },
    ],
	views: [{
		name: "memory_configurator/Summary",
		displayName: "Memory Summary",
		viewType: "markdown",
		icon: "table-large",
	}],
}