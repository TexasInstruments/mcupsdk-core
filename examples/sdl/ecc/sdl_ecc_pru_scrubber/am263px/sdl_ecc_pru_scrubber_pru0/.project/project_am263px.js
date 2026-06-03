let path = require('path');

let device = "am263px";

const files = {
    common: [
        "main.asm",
        "linker.cmd",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
        ".",        /* generated linker.cmd */
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const lflags = {
    common: [
        "--entry_point=main",
        "--diag_suppress=10063-D",
    ],
};

const includes_pru = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/common",
    ],
};

const readmeDoxygenPageTag = "EXAMPLES_SDL_ECC";

const templates_pru =
[
    {
        input: ".project/templates/am263px/common/pru/linker_pru0.cmd.xdt",
        output: "linker.cmd",
    }
];

const buildOptionCombos = [
    { device: device, cpu: "icss_m0_pru0", cgt: "ti-pru-cgt", board: "am263px-cc", os: "fw"},
];

function getmakefilePruPostBuildSteps(cpu, board)
{
    let core = "pru0";

    switch(cpu)
    {
        case "icss_m0_pru1":
            core = "pru1"
            break;
        case "icss_m0_pru0":
            core = "pru0"
    }

    return  [
        "$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=PRU0Firmware -o pru0_load_bin.h sdl_ecc_pru_scrubber_pru0_" + board + "_" + cpu + "_fw_ti-pru-cgt.out; $(MOVE) pru0_load_bin.h ${MCU_PLUS_SDK_PATH}/examples/sdl/ecc/sdl_ecc_pru_scrubber/am263px/sdl_ecc_pru_scrubber_r5f0/pru0_load_bin.h"
    ];
}

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "sdl_ecc_pru_scrubber_pru0";
    property.isInternal = false;
    property.description = "This example performs active memory scrubbing, detects and analyzes ECC errors and communicates findings back to the R5F core for PRU0"
    property.buildOptionCombos = buildOptionCombos;
    property.isSkipTopLevelBuild = true;
    property.skipUpdatingTirex = true;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.lnkfiles = lnkfiles;
    build_property.includes = includes_pru;
    build_property.lflags = lflags;
    build_property.templates = templates_pru;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.skipMakefileCcsBootimageGen = true;
    build_property.makefilePruPostBuildSteps = getmakefilePruPostBuildSteps(buildOption.cpu, buildOption.board);

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
