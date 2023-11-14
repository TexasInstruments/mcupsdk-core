let path = require('path');

let device = "am64x";

const files_r5f = {
    common: [
        "tog_test_main.c",
        "tog_test_func.c",
        "dpl_interface.c",
        "main.c",
    ],
};

const files_m4f = {
    common: [
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

const r5_macro = {
    common: [
        "R5F_CORE",
    ],
};

const m4_macro = {
    common: [
        "M4F_CORE",
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

const libs_m4f = {
    common: [
        "nortos.am64x.m4f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am64x.m4f.ti-arm-clang.${ConfigName}.lib",
        "unity.am64x.m4f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_r5f = {
    common: [
        "nortos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "unity.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "sdl.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "../linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg"

const systemProjects = [
    {
        name: "stog_func_test_r5f",
        tag: "nortos",
        skipProjectSpec: false,
        board: "am64x-evm",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "nortos"},
            { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "nortos"},
        ],
    }
];

const templates_nortos_m4f =
[
    {
        input: ".project/templates/am64x/nortos/main_nortos.c.xdt",
        options: {
            entryFunction: "tog_test_main",
        },
    }
];

const templates_nortos_r5f =
[
    {
        input: ".project/templates/am64x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "tog_test_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "nortos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "stog_func_test_r5f";
    property.isInternal = true;
    property.skipProjectSpec = true;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.filedirs = filedirs;
    build_property.includes = includes_nortos;
    build_property.libdirs = libdirs_nortos;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;

    if(buildOption.cpu.match(/m4f*/)) {
        build_property.files = files_m4f;
        build_property.libs = libs_m4f;
        build_property.templates = templates_nortos_m4f;
        build_property.defines = m4_macro;
    }

    if(buildOption.cpu.match(/r5f*/)) {
        build_property.files = files_r5f;
        build_property.libs = libs_r5f;
        build_property.templates = templates_nortos_r5f;
        build_property.defines = r5_macro;
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
