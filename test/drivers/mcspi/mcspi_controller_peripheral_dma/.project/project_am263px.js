let path = require('path');

let device = "am263px";

const files_freertos_rf5 = {
    common: [
        "test_mcspi_peripheral_dma.c",
        "main.c",
    ],
};

const files_nortos_rf5 = {
    common: [
        "test_mcspi_controller_dma.c",
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
        "nortos.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "unity.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "unity.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
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
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am263px/r5f",
        "${MCU_PLUS_SDK_PATH}/test/unity/",
    ],
};

const syscfgfile = "../example.syscfg"

const templates_nortos_r5f =
[
    {
        input: ".project/templates/am263px/common/linker_r5f.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/am263px/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_mcspi_controller_dma_main",
        },
    }
];

const templates_freertos_r5f =
[
    {
        input: ".project/templates/am263px/common/linker_r5f.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/am263px/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_mcspi_peripheral_dma_main",
        },
    }
];

const buildOptionCombos = [
    { device: "am263px", cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-cc", os: "freertos", isPartOfSystemProject: true},
    { device: "am263px", cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am263px-cc", os: "nortos", isPartOfSystemProject: true},
    { device: "am263px", cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-lp", os: "freertos", isPartOfSystemProject: true},
    { device: "am263px", cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am263px-lp", os: "nortos", isPartOfSystemProject: true},
];

const systemProjects = [
    {
        name: "test_mcspi_controller_peripheral_dma",
        tag: "freertos_nortos",
        skipProjectSpec: true,
        board: "am263px-cc",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-cc", os: "freertos", isPartOfSystemProject: true},
            { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am263px-cc", os: "nortos", isPartOfSystemProject: true},
        ],
    },
    {
        name: "test_mcspi_controller_peripheral_dma",
        tag: "freertos_nortos",
        skipProjectSpec: true,
        board: "am263px-lp",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-lp", os: "freertos", isPartOfSystemProject: true},
            { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am263px-lp", os: "nortos", isPartOfSystemProject: true},
        ],
    },
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "test_mcspi_controller_peripheral_dma";
    property.isInternal = true;
    property.skipProjectSpec = true;
    property.buildOptionCombos = buildOptionCombos;

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
            build_property.libs = libs_freertos_r5f;
            build_property.templates = templates_freertos_r5f;
        }
        else
        {
            build_property.includes = includes_nortos_r5f;
            build_property.files = files_nortos_rf5;
            build_property.libdirs = libdirs_nortos;
            build_property.libs = libs_nortos_r5f;
            build_property.templates = templates_nortos_r5f;
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
