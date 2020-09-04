let path = require('path');

let device = "am64x";

const files = {
    common: [
        "hsrPrp_red_config.c",
        "hsrPrp_red_multicastTable.c",
        "hsrPrp_red_nodeTable.c",
        "hsrPrp_red_hsr.c",
        "hsrPrp_red_snmp.c",
        "hsrPrp_red_statistics.c",
        "hsrPrp_red_tx.c",
        "hsrPrp_red_vlanTable.c",
        "hsrPrp_red.c",
    ],
};

const filedirs = {
    common: [
    ],
};

const defines = {
    common: [
        "BUILD_HSR_H",
    ],
};

const cflags = {
    common: [
        "-mno-unaligned-access",
        "-mthumb",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "hsr_mii_icss_fwhal";
    property.tag = "hsr_mii";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.defines = defines;
    build_property.cflags = cflags;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
