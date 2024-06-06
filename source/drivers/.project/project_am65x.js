let path = require('path');

let device = "am65x"

const files_r5f = {
    common: [
        "csl_sec_proxy.c",
        "pinmux.c",
        "sciclient.c",
        "sciclient_boardcfg.c",
        "sciclient_fmwSecureProxyMap.c",
        "sciclient_pm.c",
        "sciclient_soc_priv.c",
        "soc.c",
    ],
};

const filedirs = {
    common: [
        "pinmux/am65x",
        "sciclient",
        "sciclient/soc/am65x",
        "soc/am65x",
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
