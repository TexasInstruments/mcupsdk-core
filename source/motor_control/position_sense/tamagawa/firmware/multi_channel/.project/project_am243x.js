let path = require('path');

let device = "am243x";

const files = {
    common: [
        "tamagawa_main.asm",
        "tamagawa_diagnostic.cmd",
        "tamagawa_master_hexpru.cmd",
    ],
};

const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../../..", /* Example base */
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/motor_control/position_sense/tamagawa/firmware",
    ],
};

const defines = {
    common: [
        "ENABLE_MULTI_CHANNEL",
    ],
};

const readmeDoxygenPageTag = "TAMAGAWA_DESIGN";

const cflags = {
    common: [
        "-v4"
    ],
};

const lflags = {
    common: [
        "--entry_point=TAMAGAWA_INIT",
        "--disable_auto_rts",
    ],
};


const buildOptionCombos = [
    { device: device, cpu: "icssg0-pru1", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
];

let postBuildSteps = [
    "$(CG_TOOL_ROOT)/bin/hexpru.exe ${MCU_PLUS_SDK_PATH}/source/motor_control/position_sense/tamagawa/firmware/tamagawa_master_hexpru.cmd tamagawa_multi_channel_am243x-evm_icssg0-pru1_fw_ti-pru-cgt.out; ${MCU_PLUS_SDK_PATH}/tools/bin2header/bin2header.exe tamagawa_multi_channel_am243x-evm_icssg0-pru1_fw_ti-pru-cgt.b00 tamagawa_master_multi_channel_bin.h TamagawaFirmware 4; move tamagawa_master_multi_channel_bin.h ${MCU_PLUS_SDK_PATH}/source/motor_control/position_sense/tamagawa/firmware/tamagawa_master_multi_channel_bin.h ;"
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "tamagawa_multi_channel";
    property.isInternal = false;
    property.description = "Tamagawa Peripheral Interface"
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
