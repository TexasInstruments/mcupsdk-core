let common = system.getScript("/common");
let soc = system.getScript(`/pru_io/soc/pru_io_${common.getSocName()}`);

exports = {
    displayName: "PRU-IO",
    templates: [
        {
            name: "/pru_io/common/pru_io_config.inc.xdt",
            outputPath: "ti_pru_io_config.inc",
            alwaysRun: true,
        },
    ],
    topModules: soc.getTopModules(),
};
