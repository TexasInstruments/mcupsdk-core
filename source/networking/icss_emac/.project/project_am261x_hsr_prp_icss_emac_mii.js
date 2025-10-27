let path = require('path');

let device = "am261x";

const files = {
    common: [
        "icss_emac_learning.c",
        "icss_emac_local.c",
        "icss_emac_statistics.c",
        "icss_emac_stormControl.c",
        "icss_emac.c",
    ],
};

const filedirs = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/source",
    ],
};

const cflags = {
    common: [
        "-mno-unaligned-access",
    ],
};

const defines_r5f = {
    common: [
        "BUILD_HSR_PRP_MII",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "hsr_prp_icss_emac";
    property.isInternal = false;
    property.tag = "hsr_prp_icss_emac";
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.cflags = cflags;
    build_property.defines = defines_r5f;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
