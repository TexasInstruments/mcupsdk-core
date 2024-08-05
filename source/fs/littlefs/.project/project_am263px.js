let path = require('path');

let device = "am263px";

const files = {
    common: [
        "lfs.c",
        "lfs_util.c",
        "lfs_flash.c",
    ],
};

const filedirs = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/fs/littlefs/LittleFS",
        "${MCU_PLUS_SDK_PATH}/source/fs/littlefs/portable",
    ],
};

const includes_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/fs/littlefs/LittleFS",
        "${MCU_PLUS_SDK_PATH}/source/fs/littlefs/portable",
    ],
};


const cflags = {
    common: [
        "-Wno-extra",
        "-Wno-uninitialized",
        "-Wno-unused-but-set-variable",
    ],
    release: [
        "-DLFS_NO_DEBUG",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "littlefs";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.filedirs = filedirs;
    build_property.files = files;
    build_property.cflags = cflags;
    if(buildOption.cpu.match(/r5f*/)) {
        build_property.includes = includes_r5f;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
