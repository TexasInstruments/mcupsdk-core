let path = require('path');

let device = "am243x";

const files = {
    common: [
        "threadx_empty.c",
        "main.c",
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

const libdirs = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/eclipse_threadx/threadx/lib",
        "${MCU_PLUS_SDK_PATH}/source/eclipse_threadx/filex/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/eclipse_threadx/threadx/threadx_src/common/inc",
        "${MCU_PLUS_SDK_PATH}/source/eclipse_threadx/filex/filex_src/common/inc",
        "${MCU_PLUS_SDK_PATH}/source/eclipse_threadx/filex/filex_src/common/inc",
        "${MCU_PLUS_SDK_PATH}/source/eclipse_threadx/filex/filex_src/ports/generic/inc",
        "${MCU_PLUS_SDK_PATH}/source/eclipse_threadx/threadx/ports/ti_arm_gcc_clang_cortex_r5/inc",
    ],
};

const libs = {
    common: [
        "threadx.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "filex.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_gcc = {
    common: [
        "threadx.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "filex.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "drivers.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "board.am243x.r5f.gcc-armv7.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_ECLIPSE_THREADX";

const templates_gcc =
[
    {
        input: ".project/templates/am243x/common/linker_r5f_gcc.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/am243x/threadx/main_threadx.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "threadx_empty_main",
        },
    }
];

const templates =
[
    {
        input: ".project/templates/am243x/threadx/main_threadx.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "threadx_empty_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "threadx"},
    { device: device, cpu: "r5fss0-0", cgt: "gcc-armv7",    board: "am243x-evm", os: "threadx"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "threadx_empty";
    property.isInternal = false;
    property.tirexResourceSubClass = [ "example.gettingstarted" ];
    property.description = "An empty Eclipse-ThreadX example. "
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.libdirs = libdirs;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.includes = includes;
    build_property.libdirs = libdirs;
    build_property.templates = templates;
    if(buildOption.cpu.match(/gcc*/)) {
        build_property.libs = libs_gcc;
    } else {
        build_property.libs = libs;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
