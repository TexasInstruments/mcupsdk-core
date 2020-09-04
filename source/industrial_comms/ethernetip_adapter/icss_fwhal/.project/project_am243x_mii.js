let path = require('path');

let device = "am243x";

const files = {
    common: [
        "icss_dlr.c",
        "icss_eip_driver.c",
        "icss_eip_mcFltr.c",
    ],
};

const defines = {
    common: [
        "ETHERNETIP_MII_MODE",
    ],
};

const cflags = {
    common: [
        "-mno-unaligned-access",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "ethernetip_adapter_mii_icss_fwhal";
    property.tag = "mii";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.defines = defines;
    build_property.cflags = cflags;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
