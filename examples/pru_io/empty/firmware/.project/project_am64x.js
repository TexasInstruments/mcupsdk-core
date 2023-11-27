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


const readmeDoxygenPageTag = "EXAMPLES_PRU_EMPTY";

const templates_pru =
[
    {
        input: ".project/templates/am64x/common/pru/linker_pru0.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/am64x/common/pru/hexpru.cmd.xdt",
        output: "hexpru.cmd",
    },
];

const buildOptionCombos = [
    { device: device, cpu: "icssg0-pru0", cgt: "ti-pru-cgt", board: "am64x-evm", os: "fw"},
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
    property.defaultPruPostBuildSteps = true;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.lnkfiles = lnkfiles;
    build_property.includes = includes;
    build_property.templates = templates_pru;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.projecspecFileAction = "link";
    build_property.skipMakefileCcsBootimageGen = true;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
