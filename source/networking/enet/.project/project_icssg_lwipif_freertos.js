let path = require('path');
const _ = require('lodash');

const files = {
    common: [
        /* lwip if */
        "lwip2enet.c",
        "lwip2lwipif.c",
        "pbufQ.c",
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
};

const cflags = {
    common: [
        "-mno-unaligned-access",
        "-Wno-extra",
        "-fno-strict-aliasing",
    ],
    release: [
        "-Oz",
        "-flto",
    ],

};

const defines_r5f = {
    common: [
        "MAKEFILE_BUILD",
        "ENET_CFG_ASSERT=1",
        "ENET_CFG_PRINT_ENABLE",
        "ENET_CFG_TRACE_LEVEL=3",
        "ENET_ENABLE_PER_ICSSG=1",
        "ENABLE_ENET_LOG",
    ],
    debug: [
        "ENET_CFG_DEV_ERROR=1",
        "LWIPIF_INSTRUMENTATION_ENABLED=1",
        "ENETDMA_INSTRUMENTATION_ENABLED=1",
    ],
};

const buildOptionCombos = [
    { device: "am64x", cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "am243x", cpu: "r5f", cgt: "ti-arm-clang"},
];

function getComponentProperty(device) {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "lwipif-icssg-freertos";
    property.tag = "lwipif-icssg-freertos";
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
    build_property.cflags = cflags;

    includes.common = _.union(includes.common, socIncludes[device]);
    build_property.includes = includes;

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
