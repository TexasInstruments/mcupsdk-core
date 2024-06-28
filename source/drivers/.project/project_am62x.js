let path = require('path');

let device = "am62x";

const files_m4f = {
    common: [
        "csl_sec_proxy.c",
        "i2c_v0.c",
        "i2c_v0_lld.c",
        "gpio.c",
        "pinmux.c",
        "mcspi_v0.c",
        "mcspi_dma.c",
        "mcspi_dma_udma.c",
        "ipc_notify_soc.c",
        "ipc_notify_v0.c",
        "ipc_notify_v0_cfg.c",
        "ipc_rpmsg.c",
        "ipc_rpmsg_vring.c",
        "mcan.c",
        "sciclient.c",
        "sciclient_pm.c",
        "sciclient_rm.c",
        "sciclient_rm_irq.c",
        "sciclient_procboot.c",
        "sciclient_firewall.c",
        "sciclient_irq_rm.c",
        "sciclient_fmwSecureProxyMap.c",
        "sciclient_boardcfg.c",
        "sciclient_soc_priv.c",
        "soc.c",
        "uart_v0.c",
        "uart_dma.c",
    ],
};

const filedirs = {
    common: [
        "i2c/v0",
        "i2c/v0/lld",
        `pinmux/${device}`,
        "mcan/v0",
        "ipc_notify/v0",
        "gpio/v0",
        "mcspi/v0",
        "mcspi/v0/dma",
        "mcspi/v0/dma/udma",
        "ipc_notify/v0/soc",
        `ipc_notify/v0/soc/${device}`,
        "ipc_rpmsg/",
        "sciclient",
        `sciclient/soc/${device}`,
        `soc/${device}`,
        "uart/v0",
        "uart/v0/dma",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "m4f", cgt: "ti-arm-clang"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "drivers";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.filedirs = filedirs;
    if(buildOption.cpu.match(/m4f*/)) {
        build_property.files = files_m4f;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
