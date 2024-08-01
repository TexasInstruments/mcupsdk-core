let path = require('path');

let device = "am243x";

const files_r5f = {
    common: [
        "tielfup32.c",
    ],
};

const files_m4f = {
    common: [

    ],
};

const filedirs = {
    common: [
        "tiELFuParser",
    ],
};

const includes = {
    common: [
        "tiELFuParser",
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
    property.name = "middleware";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.filedirs = filedirs;
    build_property.includes = includes;
    if(buildOption.cpu.match(/r5f*/)) {
        build_property.files = files_r5f;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
