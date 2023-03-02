let path = require('path');

let device = "am64x";

const files = {
    common: [
        "sdl_pok_example.c",
        "dpl_interface.c",
		"pok_main.c",
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
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/source/sdl/lib",
    ],
};

const includes_nortos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/examples/sdl/dpl/",
    ],
};

const libs_nortos_m4f = {
    common: [
        "nortos.am64x.m4f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am64x.m4f.ti-arm-clang.${ConfigName}.lib",
        "board.am64x.m4f.ti-arm-clang.${ConfigName}.lib",
        "sdl.am64x.m4f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const m4_macro = {
    common: [
        "M4F_CORE",
    ],

};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg"

const projectspecfiles = {
    common: [
        "pok_main.h",
    ]
};


const readmeDoxygenPageTag = "EXAMPLES_SDL_POK";

const templates_nortos_m4f =
[
    {
        input: ".project/templates/am64x/common/linker_m4f.cmd.xdt",
        output: "linker.cmd",
        options: {
            isSingleCore: true,
        },
    },
    {
        input: ".project/templates/am64x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "nortos"},
];

function getComponentProperty(device) {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "pok_example";
    property.isInternal = false;
    property.description = "This example verifies POK operation"
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
	build_property.projectspecfiles = projectspecfiles;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;

    if(buildOption.cpu.match(/m4f*/))
    {
        build_property.libs = libs_nortos_m4f;
        build_property.templates = templates_nortos_m4f;
		build_property.defines = m4_macro;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
