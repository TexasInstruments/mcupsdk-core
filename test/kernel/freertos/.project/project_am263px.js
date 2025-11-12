let path = require('path');

let device = "am263px";

const files_r5f_common = {
    common: [
        "test_freertos.c",
        "test_critical_section.c",
        "test_critical_section_deep_nested.c",
        "main.c",
    ],
};

const files_r5f = {
    common: [
        ...files_r5f_common.common,
    ],
};

const files_r5f_mpu = {
    common: [
        ...files_r5f_common.common,
        "test_freertos_mpu.c",
    ],
};

const asmfiles_r5f_common = {
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

const includes_r5f_common = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/test/unity/",
    ],
};

const includes_r5f = {
    common: [
        ...includes_r5f_common.common,
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am263px/r5f",
    ],
};

const includes_r5f_mpu = {
    common: [
        ...includes_r5f_common.common,
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F_MPU",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am263px/r5f_mpu",
    ],
};

const libs_r5f_common = {
    common: [
        "drivers.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "unity.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_r5f = {
    common: [
        ...libs_r5f_common.common,
        "freertos.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_r5f_mpu = {
    common: [
        ...libs_r5f_common.common,
        "freertos.am263px.r5f-mpu.ti-arm-clang.${ConfigName}.lib",
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
        input: ".project/templates/am263px/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_freertos_main",
        },
    }
];

const templates_r5f_mpu =
[
    {
        input: ".project/templates/am263px/freertos/main_freertos_mpu.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_freertos_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-cc", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-cc", os: "freertos_mpu"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-lp", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-lp", os: "freertos_mpu"},
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

        if(buildOption.os.match(/mpu/)) {
            build_property.files = files_r5f_mpu;
            build_property.includes = includes_r5f_mpu;
            build_property.libs = libs_r5f_mpu;
            build_property.templates = templates_r5f_mpu;
    } else {
            build_property.files = files_r5f;
            build_property.includes = includes_r5f;
            build_property.libs = libs_r5f;
            build_property.templates = templates_r5f;
        }

        build_property.asmfiles = asmfiles_r5f_common;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
