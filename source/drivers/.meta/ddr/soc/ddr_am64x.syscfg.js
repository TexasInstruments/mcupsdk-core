
const staticConfig =
{
    clockIds: [ "TISCI_DEV_EMIF_DATA_0_VD", "TISCI_DEV_DDR16SS0" ],
};

let defaultDdrConfigFile = "drivers/ddr/v0/soc/am64x_am243x/board_ddrReginit.h"

function getDefaultDdrConfigFileName() {
    return defaultDdrConfigFile;
}

function getStaticConfig()
{
    return staticConfig;
}

exports = {
    getDefaultDdrConfigFileName,
    getStaticConfig,
};
