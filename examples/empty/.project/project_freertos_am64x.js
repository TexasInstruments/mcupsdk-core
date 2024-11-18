let path = require('path');

let device = "am64x";

const files = {
    common: [
        "empty.c",
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

const libdirs_freertos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
    ],
};

const includes_freertos_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am64x/r5f",
    ],
};

const includes_freertos_m4f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CM4F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am64x/m4f",
    ],
};

const includes_freertos_a53 = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/GCC/ARM_CA53",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am64x/a53",
    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_freertos_m4f = {
    common: [
        "freertos.am64x.m4f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am64x.m4f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_freertos_a53 = {
    common: [
        "freertos.am64x.a53.gcc-aarch64.${ConfigName}.lib",
        "drivers.am64x.a53.gcc-aarch64.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const defines_a53_amp = {
    common: [
        "AMP_FREERTOS_A53"
    ]
}

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_EMPTY";

const templates_freertos_r5f =
[
    {
        input: ".project/templates/am64x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "empty_main",
        },
    }
];

const templates_freertos_m4f =
[
    {
        input: ".project/templates/am64x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "empty_main",
        },
    }
];

const templates_freertos_a53 =
[
    {
        input: ".project/templates/am64x/common/linker_a53.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/am64x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "empty_main",
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
            entryFunction: "empty_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "freertos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am64x-evm", os: "freertos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss1-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "freertos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss1-1", cgt: "ti-arm-clang", board: "am64x-evm", os: "freertos", isPartOfSystemProject: true},
    { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "freertos", isPartOfSystemProject: true},
    { device: device, cpu: "a53ss0-0", cgt: "gcc-aarch64", board: "am64x-evm", os: "freertos", isPartOfSystemProject: false},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-sk", os: "freertos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am64x-sk", os: "freertos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss1-0", cgt: "ti-arm-clang", board: "am64x-sk", os: "freertos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss1-1", cgt: "ti-arm-clang", board: "am64x-sk", os: "freertos", isPartOfSystemProject: true},
    { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am64x-sk", os: "freertos", isPartOfSystemProject: true},
    { device: device, cpu: "a53ss0-0", cgt: "gcc-aarch64", board: "am64x-sk", os: "freertos", isPartOfSystemProject: false},
    { device: device, cpu: "a53ss0-1", cgt: "gcc-aarch64", board: "am64x-evm", os: "freertos", isPartOfSystemProject: false},
    { device: device, cpu: "a53ss0-1", cgt: "gcc-aarch64", board: "am64x-sk", os: "freertos", isPartOfSystemProject: false},
];

const systemProjects =[
    {
        name: "empty",
        tag: "freertos",
        skipProjectSpec: false,
        readmeDoxygenPageTag: readmeDoxygenPageTag,
        board: "am64x-evm",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "freertos"},
            { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am64x-evm", os: "freertos"},
            { device: device, cpu: "r5fss1-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "freertos"},
            { device: device, cpu: "r5fss1-1", cgt: "ti-arm-clang", board: "am64x-evm", os: "freertos"},
            { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "freertos"},
        ],
    },
    {
        name: "empty",
        tag: "freertos",
        skipProjectSpec: false,
        readmeDoxygenPageTag: readmeDoxygenPageTag,
        board: "am64x-sk",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-sk", os: "freertos"},
            { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am64x-sk", os: "freertos"},
            { device: device, cpu: "r5fss1-0", cgt: "ti-arm-clang", board: "am64x-sk", os: "freertos"},
            { device: device, cpu: "r5fss1-1", cgt: "ti-arm-clang", board: "am64x-sk", os: "freertos"},
            { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am64x-sk", os: "freertos"},
        ],
    },
]

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "empty";
    property.isInternal = false;
    property.tirexResourceSubClass = [ "example.gettingstarted" ];
    property.description = "An Empty Example. This example is intended to be a starting point for new development"
    property.buildOptionCombos = buildOptionCombos;

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
            build_property.libs = libs_freertos_r5f;
            build_property.templates = templates_freertos_r5f;
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
    }
    if(buildOption.cpu.match(/a53*/)) {
        if(buildOption.os.match(/freertos*/) )
        {
            build_property.includes = includes_freertos_a53;
            build_property.libdirs = libdirs_freertos;
            build_property.libs = libs_freertos_a53;
            build_property.defines = defines_a53_amp;
            build_property.isAmpSHM = true;
            if(buildOption.cpu.match(/a53ss0-1/))
                build_property.templates = templates_freertos_a53ss01;
            else
                build_property.templates = templates_freertos_a53;
        }
    }
    return build_property;
}

function getSystemProjects(device)
{
    return systemProjects;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
    getSystemProjects,
};
