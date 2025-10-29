let path = require('path');

let device = "am261x";

const files = {
    common: [
        "test_dpl.c",
        "test_r5f_critical_section.c",
        "main.c",
    ],
};

const asmfiles = {
    common: [
        "float_ops_r5f_asm.S",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs_common = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
    ],
};

const filedirs_ti_arm_clang = {
    common: [
        ...filedirs_common.common,
        "../../../ti-arm-clang",
    ],
};

const filedirs_iar_arm = {
    common: [
        ...filedirs_common.common,
        "../../../iar-arm",
    ],
};

const defines_freertos = {
    common: [
        "OS_FREERTOS"
    ],
}

const defines_nortos = {
    common: [
        "OS_NORTOS"
    ],
}

const libdirs_nortos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/test/unity/lib",
    ],
};

const libdirs_freertos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/test/unity/lib",
    ],
};

const includes_nortos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/test/unity/",
    ],
};

const includes_freertos_r5f_common = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am261x/r5f",
        "${MCU_PLUS_SDK_PATH}/test/unity/",
    ],
};

const includes_freertos_r5f_ti_arm_clang = {
    common:[
        ...includes_freertos_r5f_common.common,
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
    ]
}

const includes_freertos_r5f_iar_arm = {
    common:[
        ...includes_freertos_r5f_common.common,
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/IAR/ARM_CR5F",
    ]
}
const libs_nortos_r5f_ti_arm_clang = {
    common: [
        "nortos.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "unity.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_nortos_r5f_iar_arm = {
    common: [
        "nortos.am261x.r5f.iar-arm.${ConfigName}.lib",
        "drivers.am261x.r5f.iar-arm.${ConfigName}.lib",
        "unity.am261x.r5f.iar-arm.${ConfigName}.lib",
    ],
};

const libs_freertos_r5f_ti_arm_clang = {
    common: [
        "freertos.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "unity.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_freertos_r5f_iar_arm = {
    common: [
        "freertos.am261x.r5f.iar-arm.${ConfigName}.lib",
        "drivers.am261x.r5f.iar-arm.${ConfigName}.lib",
        "unity.am261x.r5f.iar-arm.${ConfigName}.lib",
    ],
};

const lnkfiles_ti_arm_clang = {
    common: [
        "linker.cmd",
    ]
};

const lnkfiles_iar_arm = {
    common: [
        "linker.icf",
    ]
};

const syscfgfile = "example.syscfg";

const templates_nortos_r5f =
[
    {
        input: ".project/templates/am261x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_main",
        },
    }
];

const templates_freertos_r5f =
[
    {
        input: ".project/templates/am261x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-som", os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-som", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-lp", os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-lp", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "iar-arm", board: "am261x-lp", os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "iar-arm", board: "am261x-lp", os: "freertos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "test_dpl";
    property.isInternal = true;
    property.skipProjectSpec = true;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.asmfiles = asmfiles;
    build_property.libdirs = libdirs_nortos;
    build_property.syscfgfile = syscfgfile;

    if(buildOption.cpu.match(/r5f*/)) {
        if(buildOption.cgt.match(/ti-arm-clang*/)) {
            build_property.filedirs = filedirs_ti_arm_clang;
            build_property.lnkfiles = lnkfiles_ti_arm_clang;
            if(buildOption.os.match(/freertos*/) )
            {
                build_property.includes = includes_freertos_r5f_ti_arm_clang;
                build_property.libdirs = libdirs_freertos;
                build_property.libs = libs_freertos_r5f_ti_arm_clang;
                build_property.templates = templates_freertos_r5f;
                build_property.defines = defines_freertos;
            }
            else
            {
                build_property.includes = includes_nortos;
                build_property.libs = libs_nortos_r5f_ti_arm_clang;
                build_property.templates = templates_nortos_r5f;
                build_property.defines = defines_nortos;
            }
        }
        else if(buildOption.cgt.match(/iar-arm*/)) {
            build_property.filedirs = filedirs_iar_arm;
            build_property.lnkfiles = lnkfiles_iar_arm;
            if(buildOption.os.match(/freertos*/) )
            {
                 build_property.includes = includes_freertos_r5f_iar_arm;
                build_property.libdirs = libdirs_freertos;
                build_property.libs = libs_freertos_r5f_iar_arm;
                build_property.templates = templates_freertos_r5f;
                build_property.defines = defines_freertos;
            }
            else
            {
                build_property.includes = includes_nortos;
                build_property.libs = libs_nortos_r5f_iar_arm;
                build_property.templates = templates_nortos_r5f;
                build_property.defines = defines_nortos;
            }
        }
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
