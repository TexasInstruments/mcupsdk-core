let path = require('path');

let nx_core = require("./nx_core.js");
let nx_addons = require("./nx_addons.js");
let nx_secure = require("./nx_secure.js");
let nx_crypto = require("./nx_crypto.js");

let device = "am243x";

const files = {
    common: [

        // Netx enet driver integration
        "netxduo_enet.c",
       
        // core,
        ...nx_core.files,

        // secure
        ...nx_secure.files,

        // addons
        ...nx_addons.files,

        // crypto
        ...nx_crypto.files
    ],
};

const filedirs = {
    common: [

        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_enet",

        // core
        ...nx_core.file_dirs,

        // addons
        ...nx_addons.file_dirs,

        // secure
        ...nx_secure.file_dirs,

        // crypto
        ...nx_crypto.file_dirs,
    ],
};

const includes = {
    common: [
        
        "${MCU_PLUS_SDK_PATH}/source/kernel/threadx/ports/ti_arm_gcc_clang_cortex_r5/inc",
        "${MCU_PLUS_SDK_PATH}/source/kernel/threadx/threadx_src/common/inc",

        "${MCU_PLUS_SDK_PATH}/source/fs/filex/filex_src/common/inc",
        "${MCU_PLUS_SDK_PATH}/source/fs/filex/filex_src/ports/generic/inc",

        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_enet",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/ports/cortex_r5/gnu/inc/",

        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/enet/rtos_drivers/include",
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/port",

        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/k3/am64x_am243x",

        // core
        ...nx_core.includes,

        // secure
        ...nx_secure.includes,

        // crypto
        ...nx_crypto.includes,

        // addons
        ...nx_addons.includes,
    ],
};

const defines_r5f = {
    common: [
        "NX_INCLUDE_USER_DEFINE_FILE",
    ],
};

const cflags = {
    common: [
        "-Wno-extra",
        "-Wno-uninitialized",
        "-Wno-unused-but-set-variable",
        "-Wno-parentheses-equality",
        "-Wno-self-assign"
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
    property.name = "netxduo";
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
    if(buildOption.cpu.match(/r5f*/)) {
        build_property.includes = includes;
    }
    build_property.defines = defines_r5f;
    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
