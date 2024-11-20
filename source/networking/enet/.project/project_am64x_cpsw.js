let path = require('path');

let device = "am64x";

const files = {
    common: [
        /* Enet Core */
        "enet.c",
        "enet_trace.c",
        "enet_utils.c",
        "enet_osal.c",
        "enet_queue.c",
        "enet_rm.c",
        "enet_rm_ioctl.c",
        "enet_rm_ioctl_register.c",
        "enet_osal_dflt.c",
        "enet_utils_dflt.c",
        "enet_phymdio_dflt.c",
        "enet_phymdio_dflt_ioctl.c",
        "enet_phymdio_dflt_ioctl_register.c",
        "mod_null.c",
        "mdio.c",
        "mdio_manual_ioctl.c",
        "mdio_ioctl.c",
        "mdio_ioctl_register.c",
        "cpsw_macport.c",
        "cpsw_macport_ioctl.c",
        "cpsw_macport_ioctl_register.c",
        "cpsw_macport_intervlan.c",
        "cpsw_macport_intervlan_ioctl.c",
        "cpsw_macport_intervlan_ioctl_register.c",
        "cpsw_macport_est.c",
        "cpsw_macport_est_ioctl.c",
        "cpsw_hostport.c",
        "cpsw_hostport_ioctl.c",
        "cpsw_hostport_ioctl_register.c",
        "cpsw_ale.c",
        "cpsw_ale_ioctl.c",
        "cpsw_ale_ioctl_register.c",
        "cpsw_stats.c",
        "cpsw_stats_ioctl.c",
        "cpsw_stats_ioctl_register.c",
        "cpsw_cpts.c",
        "cpsw_cpts_ioctl.c",
        "cpsw_cpts_ioctl_register.c",
        "per_null.c",
        "cpsw_intervlan.c",
        "cpsw.c",
        "cpsw_ioctl.c",
        "cpsw_ioctl_register.c",
        "cpsw_est.c",
        "cpsw_est_ioctl.c",
        "cpsw_est_ioctl_register.c",
        "enet_hostport.c",
        "enet_udma.c",
        "enet_udma_priv.c",

        /* CSL_FL for CPSW */
        "csl_cpgmac_sl.c",
        "csl_cpsw.c",
        "csl_cpsw_ss.c",
        "csl_cpsw_ale_9g_tblcfg.c",
        "csl_cpts.c",
        "csl_mdio.c",

        /* SOC */
        "k3_soc.c",

        /* Enet utils */
        "enetphy.c",
        "generic_phy.c",

        /* Enet utils */
        "enet_apputils.c",
        "enet_appmemutils.c",
        "enet_appethpatterns.c",
        "enet_udmautils.c",
        "enet_ioctlutils.c",
        "enet_apputils_k3.c",
    ],
};

const filedirs = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/src/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/src/mod",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/src/per",
		"${MCU_PLUS_SDK_PATH}/source/networking/enet/core/src/per/V1",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/src/common",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/src/dma/udma",

        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/emac/V5/priv",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/cpsw/V5/V5_0/priv",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/cpsw/V5/priv",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/cpts",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/mdio",

        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/k3",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/k3/am64x_am243x",

        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/src/phy",

        "${MCU_PLUS_SDK_PATH}/source/networking/enet/utils",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/utils/V3",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/utils/defcfg",

        "${MCU_PLUS_SDK_PATH}/source/networking/enet/board/src",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/board/src/am64x_am243x_evm",
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/networking/enet",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/utils/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/utils/V3",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/phy",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/src/phy",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include",

        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/emac/V5",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/cpsw/V5",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/cpsw/V5/V5_0",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/cpts",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/mdio",

        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/k3/am64x_am243x",

        "${MCU_PLUS_SDK_PATH}/source/networking/enet/board",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/board/src/am64x_am243x_evm/include",
        
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/enet/rtos_drivers/include",
    ],
};


const cflags = {
    common: [
        "-Wno-extra",
        "-Wno-error=unused-but-set-variable",
        "-Wno-unused-but-set-variable",
    ],
    release: [
        "-Oz",
        "-flto",
    ],
};

const cflags_a53 = {
    common: [
        "-Wno-extra",
        "-Wno-error=unused-but-set-variable",
        "-Wno-unused-but-set-variable",
        "-Wno-unused-function",
    ],
    release: [
        "-flto",
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

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
    { device: device, cpu: "a53", cgt: "gcc-aarch64"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "enet-cpsw";
    property.tag = "cpsw";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.filedirs = filedirs;
    build_property.files = files;
    build_property.includes = includes;
    if(buildOption.cpu.match(/r5f*/)) {
        build_property.defines = defines_r5f;
        build_property.cflags = cflags;
    }

    if(buildOption.cpu.match(/a53*/)) {
        build_property.cflags = cflags_a53;
        build_property.defines = defines_a53;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
