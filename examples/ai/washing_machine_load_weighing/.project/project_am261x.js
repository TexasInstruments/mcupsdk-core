let path = require('path');

let device = "am261x";

const files = {
    common: [
        "washing_machine_load_weighing.c",
        "test_vector.c",
        "main.c",
        "feature_extract.c",
        "feature_extract_am26.c",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
        "../../../../../../source/ai/feature_extract", /* feature_extract */
        "../../../artifacts", /* artifacts for tvmgen_default.h, mod.a */
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source",
        "${MCU_PLUS_SDK_PATH}/source/ai/feature_extract",
        "${MCU_PLUS_SDK_PATH}/source/ai/hann",
        "../../../artifacts",
        "../../..",  /* Example base for user_input_config.h */
    ],
};

const libdirs_nortos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/cmsis/lib",
        "../../../artifacts",
    ],
};

const libs_nortos_r5f = {
    common: [
        "nortos.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "cmsis.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "mod.a",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const defines = {
    common: [
        "SOC_AM261X",
        "OS_NORTOS",
    ],
};

const cflags = {
    common: [
        "-Wno-extern-initializer",
        "-Wno-unused-variable",
    ],
};

const projectspecfiles = {
    common: [
        "user_input_config.h",
        "tvmgen_default.h",
        "mod.a",
    ],
};

/* CCS-specific paths - these are only used in projectspec, not in makefile */
const projectspecIncludes = {
    common: [
        "${PROJECT_ROOT}",  /* For copied files: user_input_config.h, tvmgen_default.h */
    ],
};

const projectspecLnkPath = {
    common: [
        "${PROJECT_ROOT}",  /* For copied mod.a */
    ],
};

const syscfgfile = "../example.syscfg"

const readmeDoxygenPageTag = "EXAMPLES_AI_WASHING_MACHINE_LOAD_WEIGHING";

const templates_nortos_r5f =
[
    {
        input: ".project/templates/am261x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "washing_machine_load_weighing_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-lp", os: "nortos"},
];

function getComponentProperty(device) {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "washing_machine_load_weighing";
    property.isInternal = false;
    property.description = "AI washing machine load weighing example using feature extraction and neural network inference."
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.includes = includes;
    build_property.libdirs = libdirs_nortos;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.defines = defines;
    build_property.projectspecfiles = projectspecfiles;
    build_property.projectspecIncludes = projectspecIncludes;
    build_property.projectspecLnkPath = projectspecLnkPath;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;

    if(buildOption.cpu.match(/r5f*/)) {
        build_property.libs = libs_nortos_r5f;
        build_property.cflags = cflags;
        build_property.templates = templates_nortos_r5f;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
