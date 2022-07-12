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
        "crypto",
        "crypto/sa2ul",
        "crypto/pka",
        "crypto/rng",
    ],
};

const cflags = {
    common: [
        "-mno-unaligned-access",
        "-Wno-extra",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
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
    build_property.cflags = cflags;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
