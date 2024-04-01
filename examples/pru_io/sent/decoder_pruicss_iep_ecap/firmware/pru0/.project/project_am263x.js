let path = require('path');

let device = "am263x";

const files = {
    common: [
        "linker.cmd",
        "main.asm",
        "decoder_pru0_hexpru.cmd",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        ".",
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
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

const readmeDoxygenPageTag = "EXAMPLES_SENT_DECODER_PRUICSS_IEP_ECAP";

const buildOptionCombos = [
    { device: device, cpu: "icssm-pru0", cgt: "ti-pru-cgt", board: "am263x-cc", os: "fw"},
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

const lflags = {
    common: [
        "--entry_point=main",
        // "--disable_auto_rts",
    ],
};

const defines = {
    common: [
        "ENABLE_SHORT_SERIAL_MESSAGE",
    ],
};

let postBuildSteps = [
    "$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=SentDecoderFirmwarePru  -o sent_decoder_pru0_bin.h sent_decoder_using_ecap_am263x-cc_icssm-pru0_fw_ti-pru-cgt.out; $(CCS_INSTALL_DIR)/utils/cygwin/mv sent_decoder_pru0_bin.h ${MCU_PLUS_SDK_PATH}/examples/pru_io/sent/decoder_pruicss_iep_ecap/example/firmware/sent_decoder_pru0_bin.h;"
];

const templates_pru =
[
    {
        input: ".project/templates/am263x/common/pru/linker_pru0.cmd.xdt",
        output: "linker.cmd",
    }
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "sent_decoder_using_ecap";
    property.isInternal = false;
    property.description = "PRU0 Firmware for Sent Decoder Using ECAP"
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
    build_property.defines = defines;
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
