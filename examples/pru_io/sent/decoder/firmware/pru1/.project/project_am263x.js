let path = require('path');

let device = "am263x";

const files = {
    common: [
        "main.asm",
        "decoder_pru1_hexpru.cmd",
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
        // "${CG_TOOL_ROOT}/include",
        // "${MCU_PLUS_SDK_PATH}/source",
        "${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/common",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const readmeDoxygenPageTag = "EXAMPLES_PRU_EMPTY";

const buildOptionCombos = [
    { device: device, cpu: "icssm-pru1", cgt: "ti-pru-cgt", board: "am263x-cc", os: "fw"},
];

const hexBuildOptions = [
    "--diag_wrap=off",
    "--array",
    "--array:name_prefix=PRUFirmware",
    "-o=firmware_binary.h",
];

const cflags = {
    common: [
        "-v4"
    ],
};

const templates_pru =
[
    {
        input: ".project/templates/am263x/common/pru/linker_pru1.cmd.xdt",
        output: "linker.cmd",
    }
];

const lflags = {
    common: [
        "--entry_point=main",
        // "--disable_auto_rts",
    ],
};
const libdirs = {
    common: [
        "${CG_TOOL_ROOT}/lib"
    ],
};
let postBuildSteps = [
    "$(CG_TOOL_ROOT)/bin/hexpru.exe ${MCU_PLUS_SDK_PATH}/examples/pru_io/sent/decoder/firmware/pru1/decoder_pru1_hexpru.cmd sent_decoder_pru1_fw_am263x-cc_icssm-pru1_fw_ti-pru-cgt.out; ${MCU_PLUS_SDK_PATH}/tools/bin2header/bin2header.exe sent_decoder_pru1_fw_am263x-cc_icssm-pru1_fw_ti-pru-cgt.b00 sent_decoder_pru1_bin.h SentDecoderFirmwarePru1 4; move sent_decoder_pru1_bin.h ${MCU_PLUS_SDK_PATH}/examples/pru_io/sent/decoder/example/firmware/sent_decoder_pru1_bin.h ;"
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "sent_decoder_pru1_fw";
    property.isInternal = false;
    property.description = "sent decoder pru1 fw"
    property.buildOptionCombos = buildOptionCombos;
    property.pru_main_file = "main";
    property.pru_linker_file = "linker";
    property.isSkipTopLevelBuild = true;
    property.skipUpdatingTirex = true;
    property.enableHexTool = true;
    property.hexBuildOptions = hexBuildOptions;
    property.postBuildSteps = postBuildSteps;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.lnkfiles = lnkfiles;
    build_property.includes = includes;
    // build_property.cflags = cflags;
    build_property.lflags = lflags;
    build_property.libdirs = libdirs;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.projecspecFileAction = "link";
    build_property.skipMakefileCcsBootimageGen = true;
    build_property.templates = templates_pru;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
