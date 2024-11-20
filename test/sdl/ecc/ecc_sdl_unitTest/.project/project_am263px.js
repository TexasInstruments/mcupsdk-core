let path = require('path');

let device = "am263px";

const files = {
    common: [
		"dpl_interface.c",
		"main.c",
		"ecc_main.c",
        "ecc_ip_test_err.c",
        "ecc_r5_test_err.c",
        "ecc_test_err.c",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
		"../../../../common",
        "../../../../../dpl", /* SDL DPL base */
		"../../../../ecc_sdl_unitTest/am263px-cc",/* SDL UT base */
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
		"${MCU_PLUS_SDK_PATH}/test/sdl/ecc/common/",
	    "${MCU_PLUS_SDK_PATH}/test/sdl/ecc/ecc_sdl_unitTest/am263px-cc/",
    ],
};

const libs_r5f = {
    common: [
        "nortos.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "unity.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "sdl.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_r5fss1 = {
    common: [
        "nortos.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "unity.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "sdl.am263px.r5fss1.ti-arm-clang.${ConfigName}.lib",
    ],
};

const r5f0_macro = {
    common: [
        "R5F0_INPUTS",
    ],

};

const r5fss1_macro = {
    common: [
        "R5F1_INPUTS",
    ],

};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg"

const templates_nortos_r5f =
[
    {
        input: ".project/templates/am263px/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_main",
        },
    }
];

const templates_nortos_r5fss1 =
[
    {
        input: ".project/templates/am263px/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_main",
        },
    }
];

const buildOptionCombos = [

	{ device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-cc", os: "nortos"},
    { device: device, cpu: "r5fss1-0", cgt: "ti-arm-clang", board: "am263px-cc", os: "nortos"}
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "ecc_ut_app";
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

    if(buildOption.cpu.match(/r5fss0-0*/)) {
        build_property.libs = libs_r5f;
        build_property.templates = templates_nortos_r5f;
        build_property.defines = r5f0_macro;
    }
    if(buildOption.cpu.match(/r5fss1-0*/)) {
        build_property.libs = libs_r5fss1;
        build_property.templates = templates_nortos_r5fss1;
        build_property.defines = r5fss1_macro;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
