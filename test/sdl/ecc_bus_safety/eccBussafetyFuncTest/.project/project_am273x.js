let path = require('path');

let device = "am273x";

const files = {
    common: [
        "ecc_bus_safety_main.c",
        "sdl_ecc_bus_safety.c",
        "ecc_bus_safety_common.c",
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
        "../../../../../dpl", /* SDL DPL base */
    ],
};

const libdirs_nortos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/test/unity/lib",
        "${MCU_PLUS_SDK_PATH}/source/sdl/lib",
    ],
};

const includes_nortos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/test/unity/",
        "${MCU_PLUS_SDK_PATH}/test/sdl/dpl/",
    ],
};

const libs_nortos_r5f = {
    common: [
        "nortos.am273x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am273x.r5f.ti-arm-clang.${ConfigName}.lib",
        "unity.am273x.r5f.ti-arm-clang.${ConfigName}.lib",
        "sdl.am273x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_nortos_c66 = {
    common: [
        "nortos.am273x.c66.ti-c6000.${ConfigName}.lib",
        "drivers.am273x.c66.ti-c6000.${ConfigName}.lib",
        "unity.am273x.c66.ti-c6000.${ConfigName}.lib",
        "sdl.am273x.c66.ti-c6000.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const defines_r5f = {
    common: [
        "SUBSYS_MSS",
    ],
};

const defines_c66f = {
    common: [
        "SUBSYS_DSS",
    ],
};

const syscfgfile = "../example.syscfg"

const templates_nortos_r5f =
[
    {
        input: ".project/templates/am273x/common/linker_r5f.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/am273x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_main",
        },
    }
];

const templates_nortos_c66 =
[
    {
        input: ".project/templates/am273x/common/linker_c66.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/am273x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am273x-evm", os: "nortos"},
    { device: device, cpu: "c66ss0",   cgt: "ti-c6000",     board: "am273x-evm", os: "nortos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "test_eccBusSafetyFuncTest";
    property.isInternal = true;
    property.skipProjectSpec = true;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.includes = includes_nortos;
    build_property.libdirs = libdirs_nortos;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;

    if(buildOption.cpu.match(/r5f*/)) {
      build_property.libs = libs_nortos_r5f;
      build_property.templates = templates_nortos_r5f;
      build_property.defines = defines_r5f;
  }

    if(buildOption.cpu.match(/c66*/)) {
        build_property.libs = libs_nortos_c66;
        build_property.templates = templates_nortos_c66;
        build_property.defines = defines_c66f;
    }
    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
