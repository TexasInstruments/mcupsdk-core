let path = require('path');

let device = "am243x";

const files = {
    common: [
        "dpl_demo.c",
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
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am243x/r5f",
    ],
};

const includes_freertos_m4f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CM4F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am243x/m4f",
    ],
};

const libs_nortos_r5f = {
    common: [
        "nortos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
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

const libs_freertos_r5f = {
    common: [
        "freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_nortos_m4f = {
    common: [
        "nortos.am243x.m4f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.m4f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.m4f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_nortos_r5f_gcc = {
    common: [
        "nortos.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "drivers.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "board.am243x.r5f.gcc-armv7.${ConfigName}.lib",
    ],
};

const libs_freertos_m4f = {
    common: [
        "freertos.am243x.m4f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.m4f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.m4f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_KERNEL_DPL_DEMO";

const templates_nortos_r5f =
[
    {
        input: ".project/templates/am243x/common/linker_r5f.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/am243x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "dpl_demo_main",
        },
    }
];

const templates_nortos_r5f_gcc =
[
    {
        input: ".project/templates/am243x/common/linker_r5f_gcc.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/am243x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "dpl_demo_main",
        },
    }
];

const templates_freertos_r5f_gcc =
[
    {
        input: ".project/templates/am243x/common/linker_r5f_gcc.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/am243x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "dpl_demo_main",
        },
    }
];

const templates_freertos_r5f =
[
    {
        input: ".project/templates/am243x/common/linker_r5f.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/am243x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "dpl_demo_main",
        },
    }
];


const templates_nortos_m4f =
[
    {
        input: ".project/templates/am243x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "dpl_demo_main",
        },
    }
];

const templates_freertos_m4f =
[
    {
        input: ".project/templates/am243x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "dpl_demo_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "freertos"},
    { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "nortos"},
    { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp",  os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp",  os: "freertos"},
    { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am243x-lp",  os: "nortos"},
    { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am243x-lp",  os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-evm", os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-evm", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-lp",  os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-lp",  os: "freertos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "dpl_demo";
    property.isInternal = false;
    property.tirexResourceSubClass = [ "example.gettingstarted" ];
    property.description = "A \"Hello, World\" Example. This example shows configuration and usage of CPU+OS for MPU, cache, interrupts, heaps, semaphores, timers and cycle counters."
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.libdirs = libdirs_nortos;
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
        else
        {
            if(buildOption.cgt.match(/gcc*/) )
            {
                build_property.libs = libs_nortos_r5f_gcc;
                build_property.templates = templates_nortos_r5f_gcc;
            }
            else
            {
                build_property.libs = libs_nortos_r5f;
                build_property.templates = templates_nortos_r5f;
            }
        }
    }
    if(buildOption.cpu.match(/m4f*/)) {
        if(buildOption.os.match(/freertos*/) )
        {
            build_property.includes = includes_freertos_m4f;
            build_property.libdirs = libdirs_freertos;
            build_property.libs = libs_freertos_m4f;
            build_property.templates = templates_freertos_m4f;
        }
        else
        {
            build_property.libs = libs_nortos_m4f;
            build_property.templates = templates_nortos_m4f;
        }
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
