let path = require('path');

let device = "am261x";

const files = {
    common: [
        "emac_MII_Rcv.asm",
        "emac_MII_Xmt.asm",
        "emac_ptp.asm",
        "emac_statistics.asm",
        "emac_tts.asm",
        "micro_scheduler.asm",
        "pru.cmd",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "../..",        /* core_os_combo base */
        "../../..",     /* Example base */
        "../../../../../source",
    ],
};

const includes_freertos_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/common",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am261x/r5f",
        "${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/firmware/icss_dual_emac",
        "${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/firmware/icss_switch",
        "${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/firmware/source",
        "${MCU_PLUS_SDK_PATH}/test/unity/",
    ],
};

const cflags = {
    common: [
        "-al",
        "-ax",
        "--silicon_version=4",
        "--diag_suppress=10063",
        "--display_error_number",
        "--code_address_listing_unit=word",
        "-DTX_L2_ENABLED",
        "-DMII_TX_PIN_SWAP",
        "-v3",
        "-g",
        "--diag_wrap=off",
        "--diag_warning=225",
        "--hardware_mac=on",
        "--preproc_with_compile",
        "-DICSS_REV2",
        "-DPRU",
        "-DTTS",
        "-DHALF_DUPLEX_ENABLED",
        "-DICSS_DUAL_EMAC_BUILD",
        "-DPTP",

        //Need to fix these flags
        //"-eo.$(OBJEXT)",
        //"-fr=$(OBJDIR)",
        //"-fs=$(OBJDIR)",
    ],
};

const lflags = {
    common: [
        "-i${CG_TOOL_ROOT}/lib",
        "-i${CG_TOOL_ROOT}/include",
        "--stack_size=0",
        "--heap_size=0",
        "--disable_auto_rts",
        "-g",
        "--diag_wrap=off",
        "--diag_warning=225",
        "--reread_libs",
        "--warn_sections",
        "--entry_point=micro_scheduler",

        //Need to fix these flags
        //"-v3",
        //"--hardware_mac=on",
        //"-z", //
    ],
};

const lnkfiles = {
    common: [
        "../../../../../source/pru.cmd",
    ]
};


//Need to change the location PRU header files
const som_postBuildSteps_pru0 = [
    "$(CG_TOOL_ROOT)/bin/hexpru ${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/firmware/icss_dual_emac/mii/icss_emac_hexpru.cmd dual_emac_am261x-som_icss_m0_pru0_fw_ti-pru-cgt.out;",
    "${MCU_PLUS_SDK_PATH}/tools/bin2header/bin2header.exe dual_emac_am261x-som_icss_m0_pru0_fw_ti-pru-cgt.b00 PRU0_bin.h PRU0_b00 4;",
    "cp PRU0_bin.h ${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/firmware/icss_dual_emac/mii;",

];

const som_postBuildSteps_pru1 = [
    "$(CG_TOOL_ROOT)/bin/hexpru ${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/firmware/icss_dual_emac/mii/icss_emac_hexpru.cmd dual_emac_am261x-som_icss_m0_pru1_fw_ti-pru-cgt.out;",
    "${MCU_PLUS_SDK_PATH}/tools/bin2header/bin2header.exe dual_emac_am261x-som_icss_m0_pru1_fw_ti-pru-cgt.b00 PRU1_bin.h PRU1_b00 4;",
    "cp PRU1_bin.h ${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/firmware/icss_dual_emac/mii;",
];

const lp_postBuildSteps_pru0 = [
    "$(CG_TOOL_ROOT)/bin/hexpru ${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/firmware/icss_dual_emac/mii/icss_emac_hexpru.cmd dual_emac_am261x-lp_icss_m0_pru0_fw_ti-pru-cgt.out;",
    "${MCU_PLUS_SDK_PATH}/tools/bin2header/bin2header.exe dual_emac_am261x-lp_icss_m0_pru0_fw_ti-pru-cgt.b00 PRU0_bin.h PRU0_b00 4;",
    "cp PRU0_bin.h ${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/firmware/icss_dual_emac/mii;",

];

const lp_postBuildSteps_pru1 = [
    "$(CG_TOOL_ROOT)/bin/hexpru ${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/firmware/icss_dual_emac/mii/icss_emac_hexpru.cmd dual_emac_am261x-lp_icss_m0_pru1_fw_ti-pru-cgt.out;",
    "${MCU_PLUS_SDK_PATH}/tools/bin2header/bin2header.exe dual_emac_am261x-lp_icss_m0_pru1_fw_ti-pru-cgt.b00 PRU1_bin.h PRU1_b00 4;",
    "cp PRU1_bin.h ${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/firmware/icss_dual_emac/mii;",
];

const buildOptionCombos = [
    // { device: device, cpu: "icss_m0_pru0", cgt: "ti-pru-cgt", board: "am261x-som", os: "fw"},
    // { device: device, cpu: "icss_m0_pru1", cgt: "ti-pru-cgt", board: "am261x-som", os: "fw"},
    { device: device, cpu: "icss_m0_pru0", cgt: "ti-pru-cgt", board: "am261x-lp", os: "fw"},
    { device: device, cpu: "icss_m0_pru1", cgt: "ti-pru-cgt", board: "am261x-lp", os: "fw"}
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "dual_emac";
    property.isInternal = true;
    property.description = "PRU FW Interface for ICSS EMAC Project"
    property.buildOptionCombos = buildOptionCombos;
    property.pru_main_file = "micro_scheduler";
    property.pru_linker_file = "pru";
    property.isSkipTopLevelBuild = true;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.includes = includes_freertos_r5f;
    if(buildOption.cpu.match(/pru0/) && buildOption.board.match(/som/)) {
        build_property.postBuildSteps = som_postBuildSteps_pru0;
    }
    if(buildOption.cpu.match(/pru1/) && buildOption.board.match(/som/)) {
        build_property.postBuildSteps = som_postBuildSteps_pru1;
    }
    if(buildOption.cpu.match(/pru0/) && buildOption.board.match(/lp/)) {
        build_property.postBuildSteps = lp_postBuildSteps_pru0;
    }
    if(buildOption.cpu.match(/pru1/) && buildOption.board.match(/lp/)) {
        build_property.postBuildSteps = lp_postBuildSteps_pru1;
    }
    build_property.lnkfiles = lnkfiles;
    build_property.cflags = cflags;
    build_property.lflags = lflags;
    build_property.projecspecFileAction = "link";
    build_property.skipMakefileCcsBootimageGen = true;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
