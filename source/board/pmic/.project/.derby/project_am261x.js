let path = require('path');

let device = "am261x";

const files_r5f = {
    common: [
        "pmic_core.c",
        "pmic_esm.c",
        "pmic_io.c",
        "pmic_irq.c",
        "pmic_power.c",
        "pmic_wdg.c",
        "pmic.c",
        "pmic_lld.c",
        "pmic_tps65036xx.c"
    ],
};

const defines = {
    common: [
        "DERBY",
    ],
};

const filedirs = {
    common: [
        "pmic_lld/derby/src",
    ],
};

const includes = {
    common: [
        "../",
        "pmic_lld/derby/include",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "../..");
    property.type = "library";
    property.name = "pmic_derby";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.defines = defines;
    build_property.filedirs = filedirs;
    build_property.includes = includes;
    if(buildOption.cpu.match(/r5f*/)) {
        build_property.files = files_r5f;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
