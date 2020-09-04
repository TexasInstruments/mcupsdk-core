let common = system.getScript("/common");

const gpio_config_r5fss = [
    {
        name            : "MSS_GPIO",
        baseAddr        : "CSL_MSS_GIO_U_BASE",
        intrNumLow      : "CSL_MSS_INTR_MSS_GIO_INT1",
        intrNumHigh     : "CSL_MSS_INTR_MSS_GIO_INT0",
    },
    {
        name            : "RCSS_GPIO",
        baseAddr        : "CSL_RCSS_GIO_U_BASE",
        intrNumLow      : "CSL_MSS_INTR_RCSS_GIO_INT1",
        intrNumHigh     : "CSL_MSS_INTR_RCSS_GIO_INT0",
    },
];

const gpio_config_c66ss = [
    {
        name            : "RCSS_GPIO",
        baseAddr        : "CSL_RCSS_GIO_U_BASE",
        intrNumLow      : "CSL_DSS_INTR_RCSS_GIO_INT1",
        intrNumHigh     : "CSL_DSS_INTR_RCSS_GIO_INT0",
    },
];

function getConfigArr() {
    let gpio_config;

    if(common.getSelfSysCfgCoreName().includes("c66")) {
        gpio_config = gpio_config_c66ss;
    }
    else {
        gpio_config = gpio_config_r5fss;

    }

    return gpio_config;
}

exports = {
    getConfigArr,
};
