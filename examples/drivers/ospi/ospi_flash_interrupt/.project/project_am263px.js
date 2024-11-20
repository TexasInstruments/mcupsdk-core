let path = require('path');

const device = "am263px";

const TAG_CC_BOARD  = "am263px-cc";
const TAG_LP_BOARD  = "am263px-lp";

const files = {
    "am263px-cc":
    {
        common:
        [
            "ospi_flash_interrupt.c",
            "main.c",
            "board.c"
        ],
    },
    "am263px-lp":
    {
        common:
        [
            "ospi_flash_interrupt.c",
            "main.c",
        ],
    }
}
const projectSpecFiles = {
    "am263px-cc":
    {
        common:
        [
            "board.h"
        ]
    },
    "am263px-lp":
    {
        common: []
    }
};

const defines_board = {
    common: [
        "AM263P_LP",
    ],
}

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
    ],
};

const libdirs = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
    ],
};

const libs_r5f = {
    common: [
        "nortos.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg"

const readmeDoxygenPageTag = "EXAMPLES_DRIVERS_OSPI_FLASH_INTERRUPT";

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-cc", os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-lp", os: "nortos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "ospi_flash_interrupt";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};
    build_property.files = files[buildOption.board];
    build_property.projectspecfiles = projectSpecFiles[buildOption.board];
    build_property.filedirs = filedirs;
    build_property.libdirs = libdirs;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;

    if(buildOption.cpu.match(/r5f*/)) {
        build_property.libs = libs_r5f;
    }

    if(buildOption.board === "am263px-lp") {
        build_property.defines = defines_board;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
