let path = require('path');

let device = "awr294x";

const files = {
    common: [
        "hello_world.c",
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

const defines_safertos = {
    common: [
        "OS_SAFERTOS"
    ],
}

const libdirs_safertos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/safertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/test/unity/lib",
    ],
};

const includes_safertos_c66 = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/safertos/safeRTOS/api/201_C66x",
        "${MCU_PLUS_SDK_PATH}/source/kernel/safertos/safeRTOS/api/PrivWrapperStd",
        "${MCU_PLUS_SDK_PATH}/source/kernel/safertos/safeRTOS/portable/201_C66x",
        "${MCU_PLUS_SDK_PATH}/source/kernel/safertos/safeRTOS/portable/201_C66x/005_TI_CGT",
        "${MCU_PLUS_SDK_PATH}/source/kernel/safertos/safeRTOS/kernel/include_api",
        "${MCU_PLUS_SDK_PATH}/source/kernel/safertos/safeRTOS/config",

        "${MCU_PLUS_SDK_PATH}/test/unity/",
    ],
};

const libs_safertos_c66 = {
    common: [
        "safertos.awr294x.c66.ti-c6000.${ConfigName}.lib",
        "drivers.awr294x.c66.ti-c6000.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_HELLO_WORLD";

const templates_safertos_c66 =
[
    {
        input: ".project/templates/awr294x/common/linker_c66_safertos.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/awr294x/safertos/main_safertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "hello_world_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "c66ss0",   cgt: "ti-c6000",     board: "awr294x-evm", os: "safertos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "hello_world";
    property.isInternal = false;
    property.isSkipTopLevelBuild = true;
    property.skipProjectSpec = true;
    property.description = "A simple \"Hello, World\" Example. "
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

    build_property.includes = includes_safertos_c66;
    build_property.libdirs = libdirs_safertos;
    build_property.libs = libs_safertos_c66;
    build_property.templates = templates_safertos_c66;
    build_property.defines = defines_safertos;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
