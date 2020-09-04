let common = system.getScript("/common");

const gpio_config_r5fss = [
    {
        name            : "GPIO",
        baseAddr        : "CSL_MSS_GIO_U_BASE",
        intrNumLow      : "CSL_MSS_INTR_MSS_GIO_INT1",
        intrNumHigh     : "CSL_MSS_INTR_MSS_GIO_INT0",
    },
];

function getConfigArr() {
    let gpio_config = [];

    if(common.getSelfSysCfgCoreName().includes("r5f")) {
        gpio_config = gpio_config_r5fss;
    }

    return gpio_config;
}

exports = {
    getConfigArr,
};
