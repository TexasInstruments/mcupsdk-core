let path = require('path');

let device = "am273x";

const files_r5f = {
    common: [
        "dpl_interface.c",
        "main.c",
        "ecc_main.c",
		"ecc_ip_test_func.c",
		"ecc_r5_test_func.c",
		"ecc_sdl_test_func.c",
    ],
};
const asmfiles_r5f = {
    common: [
		"resetvecs.S",
	],
};
const files_c66 = {
    common: [
        "dpl_interface.c",
        "main.c",
        "ecc_main.c",
		"dss_ecc_ip_test_func.c",
		"dss_ecc_sdl_test_func.c",
		"dss_parity_edc_test_func.c",
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
		"../../../../ecc_sdl_funcTest/am273x-evm",/* SDL FT base */
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
		"${MCU_PLUS_SDK_PATH}/test/sdl/ecc/ecc_sdl_funcTest/am273x-evm/",
    ],
};

const libs_r5f = {
    common: [
        "nortos.am273x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am273x.r5f.ti-arm-clang.${ConfigName}.lib",
        "unity.am273x.r5f.ti-arm-clang.${ConfigName}.lib",
        "sdl.am273x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_c66 = {
    common: [
        "nortos.am273x.c66.ti-c6000.${ConfigName}.lib",
        "drivers.am273x.c66.ti-c6000.${ConfigName}.lib",
        "unity.am273x.c66.ti-c6000.${ConfigName}.lib",
        "sdl.am273x.c66.ti-c6000.${ConfigName}.lib",
    ],
};

const r5_macro = {
    common: [
        "R5F_INPUTS",
    ],

};

const c66_macro = {
    common: [
        "C66_INPUTS",
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
        input: ".project/templates/am273x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am273x-evm", os: "nortos"},
	{ device: device, cpu: "c66ss0", cgt: "ti-c6000", board: "am273x-evm", os: "nortos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "sdl_ecc_test_app";
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

    if(buildOption.cpu.match(/r5f*/)) {
		build_property.files = files_r5f;
        build_property.libs = libs_r5f;
        build_property.templates = templates_nortos_r5f;
		build_property.asmfiles = asmfiles_r5f;
		build_property.defines = r5_macro;
    }
	if(buildOption.cpu.match(/c66*/)) {
		build_property.files = files_c66;
        build_property.libs = libs_c66;
        build_property.templates = templates_nortos_c66;
		build_property.defines = c66_macro;
    }
	

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
