let path = require('path');

let device = "am243x";

const files_r5f = {
    common: [
        "test_freertos.c",
        "main.c",
    ],
};

const files_m4f = {
    common: [
        "test_freertos.c",
        "main.c",
        "float_ops.c",
    ],
};

const asmfiles_r5f = {
    common: [
        "float_ops_r5f_asm.S",
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
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/test/unity/lib",
    ],
};

const includes_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am243x/r5f",
        "${MCU_PLUS_SDK_PATH}/test/unity/",
    ],
};

const includes_m4f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CM4F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am243x/m4f",
        "${MCU_PLUS_SDK_PATH}/test/unity/",
    ],
};

const libs_r5f = {
    common: [
        "freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "unity.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_r5f_gcc = {
    common: [
        "freertos.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "drivers.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "unity.am243x.r5f.gcc-armv7.${ConfigName}.lib",
    ],
};

const libs_m4f = {
    common: [
        "freertos.am243x.m4f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.m4f.ti-arm-clang.${ConfigName}.lib",
        "unity.am243x.m4f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const templates_r5f =
[
    {
        input: ".project/templates/am243x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_freertos_main",
        },
    }
];

const templates_r5f_gcc =
[
    {
        input: ".project/templates/am243x/common/linker_r5f_gcc.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/am243x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_freertos_main",
        },
    }
];

const templates_m4f =
[
    {
        input: ".project/templates/am243x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_freertos_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "freertos"},
    { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "freertos"},
    { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-evm", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-lp", os: "freertos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "test_freertos";
    property.isInternal = true;
    property.skipProjectSpec = true;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.filedirs = filedirs;
    build_property.libdirs = libdirs;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;

    if(buildOption.cpu.match(/r5f*/)) {

        build_property.files = files_r5f;
        build_property.asmfiles = asmfiles_r5f;
        build_property.includes = includes_r5f;
        if(buildOption.cgt.match(/gcc*/) )
        {
            build_property.libs = libs_r5f_gcc;
            build_property.templates = templates_r5f_gcc;
        }
        else
        {
            build_property.libs = libs_r5f;
            build_property.templates = templates_r5f;
        }
    }
    if(buildOption.cpu.match(/m4f*/)) {
        build_property.files = files_m4f;
        build_property.includes = includes_m4f;
        build_property.libs = libs_m4f;
        build_property.templates = templates_m4f;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
