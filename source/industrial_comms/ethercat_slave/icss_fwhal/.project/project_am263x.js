let path = require('path');

let device = "am263x";

const files = {
    common: [
        "tiescbsp.c",
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/industrial_comms/ethercat_slave/icss_fwhal/firmware/m_v2.3",
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
    property.name = "ethercat_slave_icss_fwhal";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.cflags = cflags;
    build_property.includes = includes;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
