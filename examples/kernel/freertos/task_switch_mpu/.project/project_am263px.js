let path = require('path');

let device = "am263px";

const files = {
    common: [
        "task_switch_mpu.c",
        "task_switch_mpu_common.c",
        "task_switch_mpu_ping.c",
        "task_switch_mpu_pong.c",
        "main.c",
    ],
};

const projectspecfiles = {
    common:
    [
        "task_switch_mpu_private.h"
    ]
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

const includes_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F_MPU",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am263px/r5f_mpu",
    ],
};

const libdirs = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
    ],
};

const libs_r5f = {
    common: [
        "freertos.am263px.r5f-mpu.ti-arm-clang.${ConfigName}.lib",
        "drivers.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_KERNEL_FREERTOS_TASK_SWITCH_MPU";

const templates_r5f =
[
    {
        input: ".project/templates/am263px/freertos/main_freertos_mpu.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "task_switch_mpu_main",
        },
    },
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-lp", os: "freertos_mpu"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-cc", os: "freertos_mpu"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "task_switch_mpu";
    property.tirexResourceSubClass = [ "example.gettingstarted" ];
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.projectspecfiles = projectspecfiles;
    build_property.filedirs = filedirs;
    build_property.libdirs = libdirs;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;

    if(buildOption.cpu.includes("r5f")) {
        build_property.templates = templates_r5f;
        build_property.includes = includes_r5f;
        build_property.libs = libs_r5f;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
