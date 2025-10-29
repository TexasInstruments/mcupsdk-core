let path = require('path');

let device = "am261x";

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


const cflags_ti_arm_clang = {
    common: [
        "-Wno-extra",
        "-Wno-uninitialized",
        "-Wno-unused-but-set-variable",
    ],
    release: [
        "-DLFS_NO_DEBUG",
    ],
};

const cflags_iar_arm = {
    release: [
        "-Ohz",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
    { device: device, cpu: "r5f", cgt: "iar-arm"},
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
    if(buildOption.cpu.match(/r5f*/)) {
        build_property.includes = includes_r5f;
    }

    if(buildOption.cgt.match(/ti-arm-clang*/)) {
        build_property.cflags = cflags_ti_arm_clang;
    }
    else if(buildOption.cgt.match(/iar-arm*/)) {
        build_property.cflags = cflags_iar_arm;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
