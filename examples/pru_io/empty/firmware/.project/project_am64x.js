let path = require('path');

let device = "am64x";

const files = {
    common: [
        "main.asm",
        "linker.cmd"
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
        "."
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/common",
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
    ],
};

const readmeDoxygenPageTag = "EXAMPLES_PRU_EMPTY";

const templates_pru =
[
    {
        input: ".project/templates/am64x/common/pru/linker_pru0.cmd.xdt",
        output: "linker.cmd",
    }
];



function getPostBuildSteps(cpu, board)
{
    let core = "PRU0"

    switch(cpu)
    {
        case "icssg0-txpru1":
            core = "TXPRU1"
            break;
        case "icssg0-txpru0":
            core = "TXPRU0"
            break;
        case "icssg0-rtupru1":
            core = "RTUPRU1"
            break;
        case "icssg0-rtupru0":
            core = "RTUPRU0"
            break;
        case "icssg0-pru1":
            core = "PRU1"
            break;
        case "icssg0-pru0":
            core = "PRU0"
    }

    return [
        " $(CG_TOOL_ROOT)/bin/hexpru.exe --diag_wrap=off --array --array:name_prefix="+ core + "Firmware  -o "+ core.toLocaleLowerCase() + "_load_bin.h " + "empty_" + board + "_" + cpu + "_fw_ti-pru-cgt.out; move "+ core.toLocaleLowerCase() + "_load_bin.h " + "${MCU_PLUS_SDK_PATH}/examples/pru_io/empty/firmware/"+ board + "/" +core.toLocaleLowerCase() + "_load_bin.h "
    ];
}
const buildOptionCombos = [
    { device: device, cpu: "icssg0-pru0", cgt: "ti-pru-cgt", board: "am64x-evm", os: "fw"},
    { device: device, cpu: "icssg0-pru1", cgt: "ti-pru-cgt", board: "am64x-evm", os: "fw"},
    { device: device, cpu: "icssg0-rtupru0", cgt: "ti-pru-cgt", board: "am64x-evm", os: "fw"},
    { device: device, cpu: "icssg0-rtupru1", cgt: "ti-pru-cgt", board: "am64x-evm", os: "fw"},
    { device: device, cpu: "icssg0-txpru0", cgt: "ti-pru-cgt", board: "am64x-evm", os: "fw"},
    { device: device, cpu: "icssg0-txpru1", cgt: "ti-pru-cgt", board: "am64x-evm", os: "fw"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "empty";
    property.isInternal = false;
    property.description = "Empty PRU Project"
    property.buildOptionCombos = buildOptionCombos;
    property.pru_main_file = "main";
    property.pru_linker_file = "linker";
    property.isSkipTopLevelBuild = true;
    property.skipUpdatingTirex = true;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.lnkfiles = lnkfiles;
    build_property.includes = includes;
    build_property.templates = templates_pru;
    build_property.lflags = lflags;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.projecspecFileAction = "copy";
    build_property.skipMakefileCcsBootimageGen = true;
    build_property.postBuildSteps = getPostBuildSteps(buildOption.cpu, buildOption.board);
    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
