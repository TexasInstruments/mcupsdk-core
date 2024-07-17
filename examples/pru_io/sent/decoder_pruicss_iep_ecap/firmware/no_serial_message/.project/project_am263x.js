let path = require('path');

let device = "am263x";

const files = {
    common: [
        "linker.cmd",
        "main.asm",
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
        "../../../../pru0",
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
    { device: device, cpu: "icss_m0_pru0", cgt: "ti-pru-cgt", board: "am263x-cc", os: "fw"},
];


const lflags = {
    common: [
        "--entry_point=main",
        "--diag_suppress=10063-D", /* Added to suppress entry_point related warning */
    ],
};

const defines = {
    common: [
    ],
};

const templates_pru =
[
    {
        input: ".project/templates/am263x/common/pru/linker_pru0.cmd.xdt",
        output: "linker.cmd",
    }
];

function getmakefilePruPostBuildSteps(cpu, board)
{
    return  [
        "$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=SentDecoderFirmwarePru -o sent_decoder_using_iep_capture_pru0_bin.h sent_decoder_using_iep_capture_" + board + "_" + cpu + "_fw_ti-pru-cgt.out; $(SED) -i '0r ${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/pru_load_bin_copyright.h' sent_decoder_using_iep_capture_pru0_bin.h ; $(MOVE) sent_decoder_using_iep_capture_pru0_bin.h ${MCU_PLUS_SDK_PATH}/examples/pru_io/sent/decoder_pruicss_iep_ecap/example/firmware/sent_decoder_using_iep_capture_pru0_bin.h"
    ];
}

function getccsPruPostBuildSteps(cpu, board)
{
    return  [
        "$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=SentDecoderFirmwarePru -o sent_decoder_using_iep_capture_pru0_bin.h sent_decoder_using_iep_capture_" + board + "_" + cpu + "_fw_ti-pru-cgt.out; if ${CCS_HOST_OS} == win32 $(CCS_INSTALL_DIR)/utils/cygwin/sed -i '0r ${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/pru_load_bin_copyright.h' sent_decoder_using_iep_capture_pru0_bin.h ; if ${CCS_HOST_OS} == linux sed -i '0r ${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/pru_load_bin_copyright.h' sent_decoder_using_iep_capture_pru0_bin.h; if ${CCS_HOST_OS} == win32 $(CCS_INSTALL_DIR)/utils/cygwin/mv sent_decoder_using_iep_capture_pru0_bin.h ${MCU_PLUS_SDK_PATH}/examples/pru_io/sent/decoder_pruicss_iep_ecap/example/firmware/sent_decoder_using_iep_capture_pru0_bin.h; if ${CCS_HOST_OS} == linux mv sent_decoder_using_iep_capture_pru0_bin.h ${MCU_PLUS_SDK_PATH}/examples/pru_io/sent/decoder_pruicss_iep_ecap/example/firmware/sent_decoder_using_iep_capture_pru0_bin.h"
    ];
}

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "sent_decoder_using_iep_capture";
    property.isInternal = false;
    property.description = "PRU0 Firmware for Sent Decoder Using ECAP (No serial message support)"
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
    build_property.lflags = lflags;
    build_property.defines = defines;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.projecspecFileAction = "link";
    build_property.skipMakefileCcsBootimageGen = true;
    build_property.templates = templates_pru;
    build_property.ccsPruPostBuildSteps = getccsPruPostBuildSteps(buildOption.cpu, buildOption.board);
    build_property.makefilePruPostBuildSteps = getmakefilePruPostBuildSteps(buildOption.cpu, buildOption.board);

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
