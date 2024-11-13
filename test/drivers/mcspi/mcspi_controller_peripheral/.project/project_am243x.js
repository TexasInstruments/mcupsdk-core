let path = require('path');

const device_project = require("../../../../../.project/device/project_am243x.js");

let device = "am243x";

const files_freertos_rf5 = {
    common: [
        "test_mcspi_peripheral.c",
        "main.c",
    ],
};

const files_threadx_rf5 = {
    common: [
        "test_mcspi_peripheral.c",
        "main.c",
    ],
};

const files_nortos_rf5 = {
    common: [
        "test_mcspi_controller.c",
        "main.c",
    ],
};

const files_m4f = {
    common: [
        "test_mcspi_controller.c",
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
        "${MCU_PLUS_SDK_PATH}/test/unity/lib",
    ],
};

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

const libs_nortos_r5f = {
    common: [
        "nortos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "unity.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "unity.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_nortos_r5f_gcc = {
    common: [
        "nortos.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "drivers.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "unity.am243x.r5f.gcc-armv7.${ConfigName}.lib",
    ],
};

const libs_freertos_r5f_gcc = {
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

const libs_threadx_r5f = {
    common: [
        "threadx.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "unity.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const includes_nortos_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/test/unity/",
    ],
};

const includes_freertos_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am243x/r5f",
        "${MCU_PLUS_SDK_PATH}/test/unity/",
    ],
};

const includes_freertos_m4f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CM4F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am243x/m4f",
        "${MCU_PLUS_SDK_PATH}/test/unity/",
    ],
};

const includes_threadx_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/threadx/threadx_src/common/inc",
        "${MCU_PLUS_SDK_PATH}/source/kernel/threadx/ports/ti_arm_gcc_clang_cortex_r5/inc",
        "${MCU_PLUS_SDK_PATH}/test/unity/",
    ],
};


const syscfgfile = "../example.syscfg"

const templates_nortos_r5f =
[
    {
        input: ".project/templates/am243x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_mcspi_controller_main",
        },
    }
];

const templates_freertos_r5f =
[
    {
        input: ".project/templates/am243x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_mcspi_peripheral_main",
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
            entryFunction: "test_mcspi_controller_main",
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
            entryFunction: "test_mcspi_peripheral_main",
        },
    }
];

const templates_freertos_m4f =
[
    {
        input: ".project/templates/am243x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_mcspi_controller_main",
        },
    }
];

const templates_threadx_r5f =
[
    {
        input: ".project/templates/am243x/threadx/main_threadx.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_mcspi_peripheral_main",
        },
    }
];

const buildOptionCombos = [
    { device: "am243x", cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "freertos", isPartOfSystemProject: true},
    { device: "am243x", cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "freertos", isPartOfSystemProject: true},
    { device: "am243x", cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "freertos", isPartOfSystemProject: true},
    { device: "am243x", cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am243x-lp", os: "nortos", isPartOfSystemProject: true},
    { device: "am243x", cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-evm", os: "freertos", isPartOfSystemProject: true},
    { device: "am243x", cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-lp", os: "freertos", isPartOfSystemProject: true},
    { device: "am243x", cpu: "r5fss0-1", cgt: "gcc-armv7", board: "am243x-lp", os: "nortos", isPartOfSystemProject: true},
];

const systemProjects = [
    {
        name: "test_mcspi_controller_peripheral",
        tag: "freertos",
        skipProjectSpec: false,
        board: "am243x-evm",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "freertos", isPartOfSystemProject: true},
            { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "freertos", isPartOfSystemProject: true},
        ],
    },
    {
        name: "test_mcspi_controller_peripheral",
        tag: "freertos_gcc-armv7",
        skipProjectSpec: false,
        board: "am243x-evm",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-evm", os: "freertos", isPartOfSystemProject: true},
            { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "freertos", isPartOfSystemProject: true},
        ],
    },
    {
        name: "test_mcspi_controller_peripheral",
        tag: "freertos_nortos",
        skipProjectSpec: false,
        board: "am243x-lp",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "freertos", isPartOfSystemProject: true},
            { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am243x-lp", os: "nortos", isPartOfSystemProject: true},
        ],
    },
    {
        name: "test_mcspi_controller_peripheral",
        tag: "freertos_nortos_gcc-armv7",
        skipProjectSpec: false,
        board: "am243x-lp",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-lp", os: "freertos", isPartOfSystemProject: true},
            { device: device, cpu: "r5fss0-1", cgt: "gcc-armv7", board: "am243x-lp", os: "nortos", isPartOfSystemProject: true},
        ],
    },
];

const buildOptionCombos_threadx = [
    { device: "am243x", cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "threadx", isPartOfSystemProject: true},
    { device: "am243x", cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "threadx", isPartOfSystemProject: true},
];

const systemProjects_threadx = [
    {
        name: "test_mcspi_controller_peripheral",
        tag: "threadx",
        skipProjectSpec: false,
        board: "am243x-evm",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "threadx", isPartOfSystemProject: true},
            { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "freertos", isPartOfSystemProject: true},
        ],
    },
    {
        name: "test_mcspi_controller_peripheral",
        tag: "threadx_nortos",
        skipProjectSpec: false,
        board: "am243x-lp",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "threadx", isPartOfSystemProject: true},
            { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am243x-lp", os: "nortos", isPartOfSystemProject: true},
        ],
    },
];


function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "test_mcspi_controller_peripheral";
    property.isInternal = true;
    property.skipProjectSpec = false;

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

    build_property.filedirs = filedirs;
    build_property.libdirs = libdirs_freertos;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;

    if(buildOption.cpu.match(/r5f*/)) {
        if(buildOption.os.match(/freertos*/) )
        {
            build_property.includes = includes_freertos_r5f;
            build_property.files = files_freertos_rf5;
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
        else if (buildOption.os.match(/threadx*/))
        {
            build_property.files = files_threadx_rf5;
            build_property.includes = includes_threadx_r5f;
            build_property.libdirs = libdirs_threadx;
            build_property.libs = libs_threadx_r5f;
            build_property.templates = templates_threadx_r5f;
        }
        else
        {
            build_property.includes = includes_nortos_r5f;
            build_property.files = files_nortos_rf5;
            build_property.libdirs = libdirs_nortos;
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
        build_property.includes = includes_freertos_m4f;
        build_property.files = files_m4f;
        build_property.libs = libs_m4f;
        build_property.templates = templates_freertos_m4f;
    }

    return build_property;
}

function getSystemProjects(device)
{
    let sp = systemProjects
    if (device_project.getThreadXEnabled() == true)
    {
        sp = sp.concat(systemProjects_threadx);
    }
    return (sp);
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
    getSystemProjects,
};
