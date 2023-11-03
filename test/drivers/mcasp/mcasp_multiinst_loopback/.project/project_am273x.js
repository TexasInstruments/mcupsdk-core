let path = require('path');

let device = "am273x";

const files = {
    common: [
        "mcasp_loopback.c",
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
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/test/unity/lib",
    ],
};


const libs_freertos_c66 = {
    common: [
        "freertos.am273x.c66.ti-c6000.${ConfigName}.lib",
        "drivers.am273x.c66.ti-c6000.${ConfigName}.lib",
        "board.am273x.c66.ti-c6000.${ConfigName}.lib",
    ],
};

const includes_freertos_c66 = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_CGT/DSP_C66",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am273x/c66",
    ],
};


const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg"

const readmeDoxygenPageTag = "EXAMPLES_DRIVERS_MCASP_LOOPBACK";

const templates_freertos_c66 =
[
    {
        input: ".project/templates/am273x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "mcasp_loopback_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "c66ss0",   cgt: "ti-c6000",     board: "am273x-evm", os: "freertos"},
];

function getComponentProperty(device) {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "mcasp_multiinst_loopback";
    property.isInternal = true;
    property.description = "This example verifies MCASP loopback mode of operation"
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
    build_property.includes = includes_freertos_c66;
    build_property.libdirs = libdirs_freertos;
    build_property.libs = libs_freertos_c66;
    build_property.templates = templates_freertos_c66;
    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
