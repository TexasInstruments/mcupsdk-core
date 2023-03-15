let path = require('path');

let device = "am64x";

const files = {
    common: [
        /* lwip if */
        "lwip2icss_emac.c",
        "lwip2lwipif.c",
        "default_netif.c",
    ],
};

const filedirs = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/lwipif/src",
        "${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/source",
    ],
};

const includes = {
    common: [

        "${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/lwipif/inc",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am64x/r5f",

        "${MCU_PLUS_SDK_PATH}/source/industrial_comms/hsr_prp/stack/lwip/lwip-config/am64x",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/freertos/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/include",
    ],
};

const cflags = {
    common: [
        "-mno-unaligned-access",
    ],
};

const defines_r5f = {
    common: [
        "BUILD_PRP",
    ],
};
const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "hsr_prp_icss_emac_lwip_if";
    property.tag = "icss_emac_lwip_if";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.cflags = cflags;
    build_property.includes = includes;
    build_property.defines = defines_r5f;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
