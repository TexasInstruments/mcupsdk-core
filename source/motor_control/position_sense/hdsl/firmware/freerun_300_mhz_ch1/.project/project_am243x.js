let path = require('path');

let device = "am243x";

const files = {
    common: [
        "main.asm",
        "datalink.asm",
        "datalink_init.asm",
        "transport.asm",
        "utils.asm",
        "hdsl_master_icssg_hexpru.cmd",
        "hdsl_master_icssg.cmd",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
        "../../../..",
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/motor_control/position_sense/hdsl/firmware",
    ],
};

const defines = {
    common: [
        "icss1",
        "PRU1",
        "ENDAT_CHANNEL_1",
        "ICSS_G_V_1_0",
        "FREERUN_300_MHZ",
    ],
};

const lflags = {
    common: [
        "--disable_auto_rts",
        "--entry_point=main",
    ],
};

let postBuildSteps = [
    "$(CG_TOOL_ROOT)/bin/hexpru.exe ${MCU_PLUS_SDK_PATH}/source/motor_control/position_sense/hdsl/firmware/hdsl_master_icssg_hexpru.cmd hdsl_master_freerun_300_mhz_ch1_am243x-evm_icssg0-pru1_fw_ti-pru-cgt.out; ${MCU_PLUS_SDK_PATH}/tools/bin2header/bin2header.exe hdsl_master_freerun_300_mhz_ch1_am243x-evm_icssg0-pru1_fw_ti-pru-cgt.b00 hdsl_master_icssg_300_mhz_ch1_bin.h Hiperface_DSL2_0_PRU 4;  move  hdsl_master_icssg_300_mhz_ch1_bin.h  ${MCU_PLUS_SDK_PATH}/source/motor_control/position_sense/hdsl/firmware/hdsl_master_icssg_300_mhz_ch1_bin.h ;"

];

const readmeDoxygenPageTag = "HDSL_DESIGN";

const buildOptionCombos = [
    { device: device, cpu: "icssg0-pru1", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "hdsl_master_freerun_300_mhz_ch1";
    property.description = "HDSL Master Free Run Mode Firmware for PRU-ICSS running at 300 MHz";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;
    property.pru_main_file = "main";
    property.pru_linker_file = "hdsl_master_icssg";
    property.isSkipTopLevelBuild = true;
    property.skipUpdatingTirex = true;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.includes = includes;
    build_property.defines = defines;
    build_property.lflags = lflags;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.postBuildSteps = postBuildSteps;
    build_property.projecspecFileAction = "copy";
    build_property.skipMakefileCcsBootimageGen = true;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
