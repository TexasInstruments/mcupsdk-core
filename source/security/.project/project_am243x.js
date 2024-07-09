let path = require('path');

let device = "am243x";

const files = {
    common: [
        //Taken from crypto library
        "sa2ul.c",
        "crypto.c",
        "pka.c",
        "eip29t2_firmware.c",
        "crypto_util.c",
        "rng.c",
    ],
};

const filedirs = {
    common: [
        "security_common/drivers/crypto",
        "security_common/drivers/crypto/sa2ul",
        "security_common/drivers/crypto/pka",
        "security_common/drivers/crypto/rng",
    ],
};

const cflags = {
    common: [
        "-mno-unaligned-access",
        "-Wno-extra",
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/security",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
    { device: device, cpu: "r5f", cgt: "gcc-armv7"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "security";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.filedirs = filedirs;
    build_property.files = files;
    if(buildOption.cgt.match(/ti-arm-clang*/)) {
        build_property.cflags = cflags;
    }
    build_property.includes = includes;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
