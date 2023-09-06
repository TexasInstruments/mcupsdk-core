
exports = {
    defaultInstanceName: "GCC",
	displayName: "Gcc",
    config: [],
    templates:
        {
            "/memory_configurator/templates/linker.cmd.xdt": {
                linker_config: "/memory_configurator/templates/linker_gcc.cmd.xdt"
            }
        },
}