let path = require('path');

let device = "am263px";

const files_r5f = {
    common: [
        "pmic_tps653860xx.c",
        "pmic.c",
        "pmic_common.c",
        "pmic_core.c",
        "pmic_io.c",
        "pmic_power.c",
        "pmic_wdg.c",
        "pmic_lld.c"
    ],
};

const defines = {
    common: [
        "BLACKBIRD",
    ],
};

const filedirs = {
    common: [
        "pmic_lld/blackbird/src",
    ],
};

const includes = {
    common: [
        "../",
        "pmic_lld/blackbird/src",
        "pmic_lld/blackbird/include",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "../..");
    property.type = "library";
    property.name = "pmic_blackbird";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.filedirs = filedirs;
    build_property.includes = includes;
    build_property.defines = defines;
    if(buildOption.cpu.match(/r5f*/)) {
        build_property.files = files_r5f;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
