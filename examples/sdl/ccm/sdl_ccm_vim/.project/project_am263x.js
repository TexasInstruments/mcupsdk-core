let path = require('path');

let device = "am263x";

const files = {
    common: [
        "vim_example.c",
		"vim_main.c",
        "main.c",
		"dpl_interface.c",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
        "../../../../../dpl", /* SDL dpl base */
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
		  "${MCU_PLUS_SDK_PATH}/examples/sdl/ccm/",
          "${MCU_PLUS_SDK_PATH}/examples/sdl/ccm/sdl_ccm_vim/",
    ],
};

const libs_nortos_r5f = {
    common: [
        "nortos.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "sdl.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg"

const readmeDoxygenPageTag = "EXAMPLES_SDL_CCM";

const templates_nortos_r5f =
[
    {
        input: ".project/templates/am263x/common/linker_r5f.cmd.xdt",
        output: "linker.cmd",
        options: {
            isSingleCore: true,
        },
    },
    {
        input: ".project/templates/am263x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "VIM_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263x-cc", os: "nortos"},
];

function getComponentProperty(device) {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "sdl_vim_example";
    property.isInternal = false;
    property.description = "This example verifies the operation of VIM module"
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
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;

    if(buildOption.cpu.match(/r5f*/))
    {
        build_property.libs = libs_nortos_r5f;
        build_property.templates = templates_nortos_r5f;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
