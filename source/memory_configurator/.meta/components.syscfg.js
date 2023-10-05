let common = system.getScript("/common");
let selfCoreName = common.getSelfSysCfgCoreName();

let memory_configurator_module = {}

if ( !selfCoreName.includes ("a53") )
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
        ],
        views: [{
            name: "memory_configurator/Summary",
            displayName: "Memory Summary",
            viewType: "markdown",
            icon: "table-large",
        }],
    }
}

exports = memory_configurator_module