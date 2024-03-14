let path = require('path');

let device = "am263px";

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
        "${INDUSTRIAL_COMMUNICATIONS_SDK_PATH}/source/networking/icss_emac/source",
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
    property.name = "icss_emac";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.cflags = cflags;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
