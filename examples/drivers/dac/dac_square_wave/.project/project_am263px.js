let path = require('path');

let device = "am263px";

const files = {
    common: [
        "dac_square_wave.c",
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

const libs_nortos_r5f = {
    common: [
        "nortos.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg"

const readmeDoxygenPageTag = "EXAMPLES_DRIVERS_DAC_SQUARE_WAVE";

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
            entryFunction: "dac_square_wave_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-cc", os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-lp", os: "nortos"},
];

function getComponentProperty(device) {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "dac_square_wave";
    property.isInternal = false;
    property.description = "An DAC example that generates a square wave on DAC output."
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
            build_property.libs = libs_nortos_r5f;
            build_property.templates = templates_nortos_r5f;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
