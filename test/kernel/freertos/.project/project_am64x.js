let path = require('path');

let device = "am64x";

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

const files_a53 = {
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
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am64x/r5f",
        "${MCU_PLUS_SDK_PATH}/test/unity/",
    ],
};

const includes_m4f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CM4F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am64x/m4f",
        "${MCU_PLUS_SDK_PATH}/test/unity/",
    ],
};

const includes_a53 = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/GCC/ARM_CA53",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am64x/a53",
        "${MCU_PLUS_SDK_PATH}/test/unity/",
    ],
};

const libs_r5f = {
    common: [
        "freertos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "unity.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_m4f = {
    common: [
        "freertos.am64x.m4f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am64x.m4f.ti-arm-clang.${ConfigName}.lib",
        "unity.am64x.m4f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_a53 = {
    common: [
        "freertos.am64x.a53.gcc-aarch64.${ConfigName}.lib",
        "drivers.am64x.a53.gcc-aarch64.${ConfigName}.lib",
        "unity.am64x.a53.gcc-aarch64.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const defines_freertos_amp = {
    common: [
        "AMP_FREERTOS_A53"
    ],
}

const syscfgfile = "../example.syscfg";

const templates_r5f =
[
    {
        input: ".project/templates/am64x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_freertos_main",
        },
    }
];

const templates_m4f =
[
    {
        input: ".project/templates/am64x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_freertos_main",
        },
    }
];

const templates_freertos_a53ss00 =
[
    {
        input: ".project/templates/am64x/common/linker_a53.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/am64x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_freertos_main",
        },
    }
];

const templates_freertos_a53ss01 =
[
    {
        input: ".project/templates/am64x/common/linker_a53ss0-1.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/am64x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_freertos_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "freertos"},
    { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "freertos"},
    { device: device, cpu: "a53ss0-0", cgt: "gcc-aarch64", board: "am64x-evm", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-sk", os: "freertos"},
    { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am64x-sk", os: "freertos"},
    { device: device, cpu: "a53ss0-0", cgt: "gcc-aarch64", board: "am64x-sk", os: "freertos"},
    { device: device, cpu: "a53ss0-1", cgt: "gcc-aarch64", board: "am64x-evm", os: "freertos"},
    { device: device, cpu: "a53ss0-1", cgt: "gcc-aarch64", board: "am64x-sk", os: "freertos"},
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
        build_property.libs = libs_r5f;
        build_property.templates = templates_r5f;
    }
    if(buildOption.cpu.match(/m4f*/)) {
        build_property.files = files_m4f;
        build_property.includes = includes_m4f;
        build_property.libs = libs_m4f;
        build_property.templates = templates_m4f;
    }
    if(buildOption.cpu.match(/a53*/)) {
        build_property.files = files_a53;
        build_property.includes = includes_a53;
        build_property.libs = libs_a53;
        build_property.defines = defines_freertos_amp;
        build_property.isAmpSHM = true;
        if(buildOption.cpu.match(/a53ss0-1/))
            build_property.templates = templates_freertos_a53ss01;
        else
            build_property.templates = templates_freertos_a53ss00;
    }
    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
