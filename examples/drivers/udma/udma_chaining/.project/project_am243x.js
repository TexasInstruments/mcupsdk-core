let path = require('path');

const device_project = require("../../../../../.project/device/project_am243x.js");

let device = "am243x";

const files = {
    common: [
        "udma_chaining.c",
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

const libdirs_threadx = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/threadx/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
    ],
};

const libdirs_freertos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
    ],
};

const includes_threadx_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/threadx/threadx_src/common/inc",
        "${MCU_PLUS_SDK_PATH}/source/kernel/threadx/ports/ti_arm_gcc_clang_cortex_r5/inc",
    ],
};

const includes_freertos_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am243x/r5f",
    ],
};

const libs_threadx_r5f = {
    common: [
        "threadx.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_freertos_r5f_gcc = {
    common: [
        "freertos.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "drivers.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "board.am243x.r5f.gcc-armv7.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg"

const readmeDoxygenPageTag = "EXAMPLES_DRIVERS_UDMA_CHAINING";

const templates_freertos_r5f =
[

    {
        input: ".project/templates/am243x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "udma_chaining_main",
        },
    }
];

const templates_freertos_r5f_gcc =
[
    {
        input: ".project/templates/am243x/common/linker_r5f_gcc.cmd.xdt",
        output: "linker.cmd",
        options: {
            heapSize: 16*1024,
            isSingleCore: true,
        },
    },
    {
        input: ".project/templates/am243x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "udma_chaining_main",
        },
    }
];

const templates_threadx_r5f =
[
    {
        input: ".project/templates/am243x/threadx/main_threadx.c.xdt",
        output: "../main.c",
        options: {
        entryFunction: "udma_chaining_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp",  os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-evm", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-lp",  os: "freertos"},
];

const buildOptionCombos_threadx = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "threadx"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp",  os: "threadx"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "udma_chaining";
    property.isInternal = false;

    if (device_project.getThreadXEnabled() == true)
    {
        property.buildOptionCombos = buildOptionCombos.concat(buildOptionCombos_threadx);
    }
    else
    {
        property.buildOptionCombos = buildOptionCombos;
    }

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    if(buildOption.cpu.match(/r5f*/)) {
        if(buildOption.os.match(/freertos*/) )
        {
            build_property.includes = includes_freertos_r5f;
            build_property.libdirs = libdirs_freertos;
            if(buildOption.cgt.match(/gcc*/) )
            {
                build_property.libs = libs_freertos_r5f_gcc;
                build_property.templates = templates_freertos_r5f_gcc;
            }
            else
            {
                build_property.libs = libs_freertos_r5f;
                build_property.templates = templates_freertos_r5f;
            }
        }
        else if (buildOption.os.match(/threadx*/)) {
            build_property.includes = includes_threadx_r5f;
            build_property.libdirs = libdirs_threadx;
            build_property.libs = libs_threadx_r5f;
            build_property.templates = templates_threadx_r5f;
        }
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
