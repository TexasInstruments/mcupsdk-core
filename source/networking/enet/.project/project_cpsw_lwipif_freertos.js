let path = require('path');
const _ = require('lodash');

const files = {
    common: [
        /* lwip if */
        "lwip2enet.c",
        "lwip2lwipif.c",
        "pbufQ.c",
        "custom_pbuf.c",
    ],
};

const filedirs = {
    common: [
       "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/lwipif/src",
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/networking/enet",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/utils/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/phy",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/src/phy",

       "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/lwipif/inc",

       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/include",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/freertos/include",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/include",
       "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/include",
    ],
};

const socIncludes = {
    am243x : [
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/k3/am64x_am243x",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am243x",
    ],
    am64x : [
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/k3/am64x_am243x",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am64x",
    ],
    am263x : [
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/am263x",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am263x/enet",
    ],
    am263px : [
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/am263px",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am263px/enet",
    ],
    am261x : [
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/am261x",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am261x/enet",
    ],
    am273x : [
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/am273x",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am273x",

    ],
    awr294x : [
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/awr294x",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/awr294x",
    ],

};

const cflags = {
    common: [
        "-Wno-extra",
    ],
    release: [
        "-Oz",
        "-flto",
    ],
};


const cflags_a53 = {
    common: [
        "-Wno-extra",
        "-Wno-unused-function",
    ],
    release: [
        "-flto",
    ],
};

const soc_cflags = {
    am243x : [
    ],
    am64x : [
    ],
    am263x : [
        "-Wno-ti-macros",
    ],
    am263px : [
        "-Wno-ti-macros",
    ],
    am261x : [
        "-Wno-ti-macros",
    ],
    am273x : [
        "-Wno-ti-macros",
        "-fno-strict-aliasing",

    ],
    awr294x : [
        "-Wno-ti-macros",
        "-fno-strict-aliasing",
    ],
};

const defines_r5f = {
    common: [
        "MAKEFILE_BUILD",
        "ENET_CFG_ASSERT=1",
        "ENET_CFG_PRINT_ENABLE",
        "ENET_CFG_TRACE_LEVEL=3",
        "ENET_ENABLE_PER_CPSW=1",
        "ENABLE_ENET_LOG",
    ],
    debug: [
        "ENET_CFG_DEV_ERROR=1",
        "LWIPIF_INSTRUMENTATION_ENABLED=1",
        "ENETDMA_INSTRUMENTATION_ENABLED=1",
    ],
};


const defines_a53 = {
    common: [
        "MAKEFILE_BUILD",
        "ENET_CFG_ASSERT=1",
        "ENET_CFG_PRINT_ENABLE",
        "ENET_CFG_TRACE_LEVEL=3",
        "ENET_ENABLE_PER_CPSW=1",
        "ENABLE_ENET_LOG",
    ],
    debug: [
        "ENET_CFG_DEV_ERROR=1",
        "LWIPIF_INSTRUMENTATION_ENABLED=1",
        "ENETDMA_INSTRUMENTATION_ENABLED=1",
    ],
};



const soc_defines = {
    am64x : [
    ],
};

const buildOptionCombos = [
    { device: "am263x", cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "am263px", cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "am261x", cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "am243x", cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "am243x", cpu: "r5f", cgt: "gcc-armv7"},
    { device: "am273x", cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "am64x",  cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "am64x",  cpu: "a53", cgt: "gcc-aarch64"},
    { device: "awr294x", cpu: "r5f", cgt: "ti-arm-clang"},
];

function getComponentProperty(device) {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "lwipif-cpsw-freertos";
    property.tag = "lwipif-cpsw-freertos";
    property.isInternal = false;

    deviceBuildCombos = []
    for (buildCombo of buildOptionCombos)
    {
        if (buildCombo.device === device)
        {
            deviceBuildCombos.push(buildCombo)
        }
    }
    property.buildOptionCombos = deviceBuildCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {

    let build_property = {};

    build_property.filedirs = filedirs;
    build_property.files = files;
    includes.common = _.union(includes.common, socIncludes[device]);
    build_property.includes = includes;

    if(buildOption.cpu.match(/a53*/))
    {
        defines_a53.common = _.union(defines_a53.common, soc_defines[device])
        build_property.defines = defines_a53;
        cflags_a53.common = _.union(cflags_a53.common, soc_cflags[device])
        build_property.cflags = cflags_a53;
        return build_property;
    }

    cflags.common = _.union(cflags.common, soc_cflags[device])
    if(buildOption.cgt.match(/ti-arm-clang*/)){
        build_property.cflags = cflags;
    }
    
    if(buildOption.cpu.match(/r5f*/))
    {
        build_property.defines = defines_r5f;
    }
    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};