let path = require('path');

let device = "am263x";

const files = {
    common: [
		"ecc_trigger.c",
        "ecc_main.c",
        "ecc_pru_io.c",
		"dpl_interface.c",
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
        "../../../sdl",
        "../../../../../dpl", /* SDL DPL base */
    ],
};

const libdirs_freertos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/source/sdl/lib",
    ],
};

const includes_freertos_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am263x/r5f",
		"${MCU_PLUS_SDK_PATH}/examples/sdl/dpl/",
		"${MCU_PLUS_SDK_PATH}/examples/sdl/ecc/",
		"${MCU_PLUS_SDK_PATH}/examples/sdl/ecc/sdl_ecc_pru_scrubber/",
    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "sdl.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const defines_r5f = {
    common: [
        "SUBSYS_MSS",
        "R5F0_INPUTS",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_SDL_ECC";

const templates_freertos_r5f =
[
    {
        input: ".project/templates/am263x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "ecc_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263x-cc", os: "freertos"},
];
function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "sdl_ecc_pru_scrubber";
    property.isInternal = false;
	property.description = "This example verifies ECC inject operations in single and double bits for PRU Scrubber"
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;

    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.defines = defines_r5f;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;

    if(buildOption.cpu.match(/r5f*/)) {
        build_property.includes = includes_freertos_r5f;
        build_property.libdirs = libdirs_freertos;
        build_property.libs = libs_freertos_r5f;
        build_property.templates = templates_freertos_r5f;
    }
    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};