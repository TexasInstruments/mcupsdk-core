let path = require('path');

let device = "am65x"

const files_r5f = {
    common: [
        "csl_intaggr.c",
        "csl_proxy.c",
        "csl_ringacc.c",
        "csl_sec_proxy.c",
        "csl_udmap.c",
        "gpio.c",
        "i2c_v0.c",
        "i2c_v0_lld.c",
        "pinmux.c",
        "sciclient.c",
        "sciclient_boardcfg.c",
        "sciclient_fmwSecureProxyMap.c",
        "sciclient_irq_rm.c",
        "sciclient_pm.c",
        "sciclient_rm.c",
        "sciclient_rm_irq.c",
        "sciclient_soc_priv.c",
        "soc.c",
        "udma.c",
        "udma_ch.c",
        "udma_event.c",
        "udma_flow.c",
        "udma_ring_common.c",
        "udma_ring_normal.c",
        "udma_rm.c",
        "udma_rmcfg.c",
        "udma_rmcfg_common.c",
        "udma_soc.c",
        "udma_utils.c",
    ],
};

const filedirs = {
    common: [
        "gpio/v0",
        "i2c/v0",
        "i2c/v0/lld",
        "hw_include/ringacc/V0/priv",
        "pinmux/am65x",
        "sciclient",
        "sciclient/soc/am65x",
        "soc/am65x",
        "udma/v1",
        "udma/hw_include",
        "udma/soc",
        `udma/soc/am65x`,
    ],
};

const filedirs_r5f =  {
    common: [

    ]
};

const asmfiles_r5f = {
    common: [

    ]
};


const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},

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
    if(buildOption.cpu.match(/r5f*/)) {
        build_property.filedirs = {common: [...filedirs.common, ...filedirs_r5f.common]};
        build_property.files = files_r5f;
        build_property.asmfiles = asmfiles_r5f;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
