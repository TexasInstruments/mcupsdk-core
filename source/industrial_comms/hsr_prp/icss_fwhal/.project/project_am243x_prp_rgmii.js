let path = require('path');

let device = "am243x";

const files = {
    common: [
        "hsrPrp_red_config.c",
        "hsrPrp_red_multicastTable.c",
        "hsrPrp_red_nodeTable.c",
        "hsrPrp_red_prp.c",
        "hsrPrp_red_snmp.c",
        "hsrPrp_red_statistics.c",
        "hsrPrp_red_tx.c",
        "hsrPrp_red_vlanTable.c",
        "hsrPrp_red.c",
    ],
};

const defines = {
    common: [
        "BUILD_PRP",
        "HSR_PRP_RGMII",
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
    property.name = "prp_rgmii_icss_fwhal";
    property.tag = "prp_rgmii";
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
