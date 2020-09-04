let path = require('path');

let device = "am64x";

const files = {
    common: [
        "iPNDrv.c",
        "iPNIsoMDrv.c",
        "iPNLegacy.c",
        "iPnOs.c",
        "iPtcpDrv.c",
        "iRtcDrv.c",
        "iRtcDrv2.c",
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/industrial_comms/profinet_device/icss_fwhal/IRT",
    ],
};

const defines = {
    common: [
        "PROFINET_MII_MODE",
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
    property.name = "profinet_device_irt_mii_icss_fwhal";
    property.tag = "irt_mii";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.includes = includes;
    build_property.defines = defines;
    build_property.cflags = cflags;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
