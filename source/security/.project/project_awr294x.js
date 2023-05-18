let path = require('path');

let device = "awr294x";

const files = {
    common: [
        //Taken from crypto library
        "dthe.c",
        "dthe_aes.c",
        "dthe_sha.c",
        "crypto_util.c",
    ],
};

const filedirs = {
    common: [
        "crypto",
        "crypto/dthe",
    ],
};

const cflags_r5 = {
    common: [
        "-mno-unaligned-access",
        "-Wno-extra",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
    { device: device, cpu: "c66", cgt: "ti-c6000"},
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

    if(buildOption.cpu.match(/r5f*/)) {
        build_property.files = files;
        build_property.filedirs = filedirs;
        build_property.cflags = cflags_r5;
    }
    if(buildOption.cpu.match(/c66*/)) {
        build_property.files = files;
        build_property.filedirs = filedirs;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};