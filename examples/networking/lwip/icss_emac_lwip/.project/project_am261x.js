let path = require('path');

let device = "am261x";

const files = {
    common: [
            "test.c",
            "test_icss.c",
            "main.c",
            "udp_iperf.c",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
    ],
};

const libdirs_freertos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/lwipif/lib",

    ],
};

const includes_freertos_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am261x/r5f",
        "${MCU_PLUS_SDK_PATH}/source/networking",
        "${MCU_PLUS_SDK_PATH}/source/networking/icss_emac",
        "${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/lwipif/inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/source",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/include/lwip",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/freertos/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am261x/icss_emac",
        "${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/firmware",
        "${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/firmware/icss_dual_emac",
        "${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/firmware/source",
        "${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/firmware/icss_switch",
    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "icss_emac.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "icss_emac_lwip_if.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "lwip-freertos-icss_emac.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "lwip-contrib-freertos-icss_emac.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const defines_r5f = {
    common: [
    ],
};

const cflags_r5f = {
    common: [
    ],
};

const lflags_r5f = {
    common: [
        "--zero_init=on",
    ],
};

const lnkfiles = {
    common: [
        "../linker.cmd",
    ]
};

const defines_r5f_cc = {
    common: [
        "AM261X_CC"
    ],
};

const defines_r5f_lp = {
    common: [
        "AM261X_LP"
    ],
};
const syscfgfile = "../example.syscfg";


const templates_freertos_r5f =
[
    {
        input: ".project/templates/am261x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "icss_lwip_example",
        },
    },
    {
        input: ".project/templates/am261x/common/linker_r5f.cmd.xdt",
        output: "linker.cmd",
        options: {
            stackSize: "65536",
            codeDataAddr: "70040000",
            codeDataSize: "000140000",
            isIcssPktBufEnable: true,
        },

    },

];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-som", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-lp", os: "freertos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "icss_emac_lwip";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.projecspecFileAction = "link";
    if(buildOption.cpu.match(/r5f*/)) {
        if(buildOption.os.match(/freertos*/) )
        {
            build_property.includes = includes_freertos_r5f;
            build_property.libdirs = libdirs_freertos;
            build_property.libs = libs_freertos_r5f;
            build_property.templates = templates_freertos_r5f;
            build_property.defines = defines_r5f;
            build_property.cflags = cflags_r5f;
            build_property.lflags = lflags_r5f;
        }
    }
    if(buildOption.board.match(/cc*/)) {
        build_property.defines = defines_r5f_cc;
    }

    if(buildOption.board.match(/lp*/)) {
        build_property.defines = defines_r5f_lp;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};



