let path = require('path');

let device = "am263px";

const files_nortos = {
    common: [
        "Serial_Cmd_HAL.c",
        "Serial_Cmd_Monitor.c",
        "real_time_debug_no_rtos.c",
        "main.c",
    ],
};

const files_freertos = {
    common: [
        "Serial_Cmd_HAL.c",
        "Serial_Cmd_Monitor.c",
        "real_time_debug_free_rtos.c",
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

const libdirs_nortos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib",
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

const includes_freertos_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am263px/r5f",
    ],
};

const libs_nortos_r5f = {
    common: [
        "nortos.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const projectspec_files = {
    common: [
        "../../../Serial_Cmd_HAL.h",
        "../../../Serial_Cmd_Monitor.h",
        "../../../device.h",
    ]
}

const syscfgfile = "../example.syscfg"

const readmeDoxygenPageTag = "EXAMPLES_REAL_TIME_DEBUG";

const templates_nortos_r5f =
[
    {
        input: ".project/templates/am263px/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "real_time_debug_main",
        },
    }
];

const templates_freertos_r5f =
[
    {
        input: ".project/templates/am263px/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "real_time_debug_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-cc", os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-cc", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-lp", os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-lp", os: "freertos"},
];

function getComponentProperty(device) {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "real_time_debug";
    property.isInternal = false;
    property.description = "An example that demonstrates Real time debug feature."
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.filedirs = filedirs;
    build_property.libdirs = libdirs_nortos;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.projectspec_files = projectspec_files;

    if(buildOption.cpu.match(/r5f*/)) {
        if(buildOption.os.match(/freertos*/) )
        {
            build_property.files = files_freertos;
            build_property.includes = includes_freertos_r5f;
            build_property.libdirs = libdirs_freertos;
            build_property.libs = libs_freertos_r5f;
            build_property.templates = templates_freertos_r5f;
        }
        else
        {
            build_property.files = files_nortos;
            build_property.libs = libs_nortos_r5f;
            build_property.templates = templates_nortos_r5f;
        }
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
