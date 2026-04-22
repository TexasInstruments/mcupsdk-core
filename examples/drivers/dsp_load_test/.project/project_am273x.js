let path = require('path');

let device = "am273x";

const files = {
    common: [
        "dsp_load_test.c",
        "gen_twiddle_fft32x32.c",
        "main.c",
    ],
};

const projectspecfiles = {
    common: [
        "gen_twiddle_fft32x32.h",
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

const includes_c66 = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_CGT/DSP_C66",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am273x/c66",
        "${DSPLIB_PATH}/packages",
    ],
};

const libdirs_freertos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${DSPLIB_PATH}/packages/ti/dsplib/lib",
    ],
};

const libs_freertos_c66 = {
    common: [
        "freertos.am273x.c66.ti-c6000.${ConfigName}.lib",
        "drivers.am273x.c66.ti-c6000.${ConfigName}.lib",
        "board.am273x.c66.ti-c6000.${ConfigName}.lib",
        "dsplib.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_DRIVERS_DSP_LOAD_TEST";

const templates_freertos_c66 =
[
    {
        input: ".project/templates/am273x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "dsp_load_test_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "c66ss0", cgt: "ti-c6000", board: "am273x-evm", os: "freertos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "dsp_load_test";
    property.isInternal = false;
    property.description = "A DSP load test example that runs DSP_fft32x32 in a tight loop to achieve >90% C66x CPU load. CPU load is reported periodically via TaskP_loadGetTotalCpuLoad.";
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.projectspecfiles = projectspecfiles;
    build_property.filedirs = filedirs;
    build_property.libdirs = libdirs_freertos;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;

    if(buildOption.cpu.match(/c66*/)) {
        build_property.includes = includes_c66;
        build_property.libs = libs_freertos_c66;
        build_property.templates = templates_freertos_c66;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
