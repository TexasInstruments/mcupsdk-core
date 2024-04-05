let common = system.getScript("/common");
let selfCoreName = common.getSelfSysCfgCoreName();

let memory_configurator_module = {}

if ( (!selfCoreName.includes ("a53")) && (common.getSocName() != "awr294x") && (system.compiler != "gcc"))
{
    memory_configurator_module = {
        displayName: "TI Memory Configurator",
        topModules: [{
            displayName: "Memory Configurator",
            modules: [
                "memory_configurator/default_linker",
                "memory_configurator/general",
                "memory_configurator/region",
                "memory_configurator/section",
                "memory_configurator/shared_region_references",
            ],
        }],
        templates: [
            {
                name: "/memory_configurator/templates/linker.cmd.xdt",
                outputPath: "linker.cmd",
                alwaysRun: false,
            },
            {
                name: "/memory_configurator/templates/linker_defines.h.xdt",
                outputPath: "linker_defines.h",
                alwaysRun: true,
            },
        ],
        views: [{
            name: "memory_configurator/Summary",
            displayName: "Memory Summary",
            viewType: "markdown",
            icon: "table-large",
            ignoreErrors: true,
        }],
    }
}

exports = memory_configurator_module