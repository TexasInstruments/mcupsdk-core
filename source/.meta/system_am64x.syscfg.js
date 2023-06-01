
let common = system.getScript("/common");

let system_module = {
    displayName: "System",

	moduleStatic: {
		modules: function(inst) {
			return [
                {
				    name: "clock",
				    moduleName: "/kernel/dpl/clock",
                },
                {
				    name: "debug_log",
				    moduleName: "/kernel/dpl/debug_log",
                },
            ]
		},
	},
};

exports = system_module;

