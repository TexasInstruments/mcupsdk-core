let path = require('path');

let device = "awr294x";

const files_r5f = {
    common: [
        "stc_main.c",
        "main.c",
        "dpl_interface.c",

    ],
};

const files_c66 = {
    common: [
        "stc_dsp_main.c",
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
        "../../../../../dpl", /* SDL dpl base add an extra lvl*/
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


const libs_nortos_r5f = {
    common: [
        "nortos.awr294x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.awr294x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.awr294x.r5f.ti-arm-clang.${ConfigName}.lib",
        "sdl.awr294x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};



const includes_nortos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/examples/sdl/dpl/",

    ],
};


const libs_nortos_c66 = {
    common: [
        "nortos.awr294x.c66.ti-c6000.${ConfigName}.lib",
        "drivers.awr294x.c66.ti-c6000.${ConfigName}.lib",
        "board.awr294x.c66.ti-c6000.${ConfigName}.lib",
        "sdl.awr294x.c66.ti-c6000.${ConfigName}.lib",
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

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_SDL_STC";

const templates_nortos_r5f =
[
    {
        input: ".project/templates/awr294x/common/linker_r5f.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/awr294x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "STC_main",
        },
    }
];


const templates_nortos_c66 =
[
    {
        input: ".project/templates/awr294x/common/linker_c66.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/awr294x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "STC_main",
        },
    }
];
const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "awr294x-evm", os: "nortos",isPartOfSystemProject: true},
     { device: device, cpu: "c66ss0",   cgt: "ti-c6000",     board: "awr294x-evm", os: "nortos",isPartOfSystemProject: true},
];

const systemProjects = [
    {
        name: "sdl_stc",
        tag: "nortos_nortos",
        skipProjectSpec: false,
        readmeDoxygenPageTag: readmeDoxygenPageTag,
        board: "awr294x-evm",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am273x-evm", os: "nortos",isPartOfSystemProject: true},
            { device: device, cpu: "c66ss0", cgt: "ti-c6000", board: "am273x-evm", os: "nortos",isPartOfSystemProject: true},
        ],
    }
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "sdl_stc";
    property.isInternal = false;
    property.description = "This example verifies STC is Done for R5F core followed by DSP Core."
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};


    build_property.filedirs = filedirs;
    build_property.libdirs = libdirs_nortos;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.includes = includes_nortos;


    if(buildOption.cpu.match(/r5f*/)) {
        {
            build_property.libs = libs_nortos_r5f;
            build_property.templates = templates_nortos_r5f;
            build_property.defines = r5_macro;
            build_property.files = files_r5f;
        }
    }
    if(buildOption.cpu.match(/c66*/)) {
        build_property.libs = libs_nortos_c66;
        build_property.templates = templates_nortos_c66;
        build_property.defines = c66_macro;
        build_property.files = files_c66;

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