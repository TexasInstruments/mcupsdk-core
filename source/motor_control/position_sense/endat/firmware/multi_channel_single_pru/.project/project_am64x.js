let path = require('path');

let device = "am64x";

const files = {
    common: [
        "endat_main.asm",
        "endat_diagnostic.cmd",
        "endat_master_hexpru.cmd"
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
        "${MCU_PLUS_SDK_PATH}/source/motor_control/position_sense/endat/firmware",
    ],
};

const defines = {
    common: [
        "ENABLE_MULTI_CHANNEL",
    ],
};



const readmeDoxygenPageTag = "ENDAT_DESIGN";

const cflags = {
    common: [
        "-v4"
    ],
};

const lflags = {
    common: [
        "--entry_point=ENDAT_INIT",
        "--disable_auto_rts",
    ],
};


const buildOptionCombos = [
    { device: device, cpu: "icssg0-pru1", cgt: "ti-pru-cgt", board: "am64x-evm", os: "fw"},
];

let postBuildSteps = [
    "$(CG_TOOL_ROOT)/bin/hexpru.exe ${MCU_PLUS_SDK_PATH}/source/motor_control/position_sense/endat/firmware/endat_master_hexpru.cmd endat_peripheral_interface_multi_ch_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.out; ${MCU_PLUS_SDK_PATH}/tools/bin2header/bin2header.exe endat_peripheral_interface_multi_ch_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.b00  endat_master_multi_bin.h EnDatFirmwareMulti 4; move endat_master_multi_bin.h ${MCU_PLUS_SDK_PATH}/source/motor_control/position_sense/endat/firmware/endat_master_multi_bin.h;",


];
function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "endat_peripheral_interface_multi_ch";
    property.isInternal = false;
    property.description = "Endat Multi Channel Interface for Same make encoders"
    property.buildOptionCombos = buildOptionCombos;
    property.pru_main_file = "main";
    property.pru_linker_file = "linker";
    property.isSkipTopLevelBuild = true;
    property.skipUpdatingTirex = true;
    property.postBuildSteps = postBuildSteps;
    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.includes = includes;
    build_property.defines = defines;
    build_property.cflags = cflags;
    build_property.lflags = lflags;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.projecspecFileAction = "copy";
    build_property.skipMakefileCcsBootimageGen = true;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
