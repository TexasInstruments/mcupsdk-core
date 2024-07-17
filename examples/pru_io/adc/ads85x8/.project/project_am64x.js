let path = require('path');

let device = "am64x";

const files_r5f = {
    common: [
        "ads8598_example.c",
        "adc_functions.c",
        "main.c",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs_r5f = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
    ],
};

const libdirs_freertos_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/source/pru_io/lib",
    ],
};

const includes_freertos_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am64x/r5f",
        "${MCU_PLUS_SDK_PATH}/source/pru_io/driver",
        "${MCU_PLUS_SDK_PATH}/examples/pru_io/adc/ads85x8/firmware",
    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "pru_ipc.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const lnkfiles_r5f = {
    common: [
        "../linker.cmd",
    ]
};

const files_pru = {
    common: [
        "main.asm",
        "linker.cmd"
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs_pru = {
    common: [
        "..",       /* core_os_combo base */
        "../../../firmware", /* Example base */
        "."
    ],
};

const includes_pru = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/common",
        "${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/adc/include",
    ],
};

const lflags_pru = {
    common: [
        "--entry_point=main",
        "--diag_suppress=10063-D", /* Added to suppress entry_point related warning */
    ],
};


const lnkfiles_pru = {
    common: [
        "linker.cmd",
    ]
};

const templates_pru =
[
    {
        input: ".project/templates/am64x/common/pru/linker_pru0.cmd.xdt",
        output: "linker.cmd",
    }
];


function getmakefilePruPostBuildSteps(cpu, board)
{
    let core = "PRU0"

    switch(cpu)
    {
        case "icss_g0_tx_pru1":
            core = "TXPRU1"
            break;
        case "icss_g0_tx_pru0":
            core = "TXPRU0"
            break;
        case "icss_g0_rtu_pru1":
            core = "RTUPRU1"
            break;
        case "icss_g0_rtu_pru0":
            core = "RTUPRU0"
            break;
        case "icss_g0_pru1":
            core = "PRU1"
            break;
        case "icss_g0_pru0":
            core = "PRU0"
    }

    return [
        " $(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix="+ core + "Firmware  -o "+ core.toLocaleLowerCase() + "_load_bin.h " + "ads85x8_" + board + "_" + cpu + "_fw_ti-pru-cgt.out; $(SED) -i '0r ${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/pru_load_bin_copyright.h' "+ core.toLocaleLowerCase() + "_load_bin.h ; $(MOVE) "+ core.toLocaleLowerCase() + "_load_bin.h " + "${MCU_PLUS_SDK_PATH}/examples/pru_io/adc/ads85x8/firmware/"+ core.toLocaleLowerCase() + "_load_bin.h "
    ];
}

function getccsPruPostBuildSteps(cpu, board)
{
    let core = "PRU0"

    switch(cpu)
    {
        case "icss_g0_tx_pru1":
            core = "TXPRU1"
            break;
        case "icss_g0_tx_pru0":
            core = "TXPRU0"
            break;
        case "icss_g0_rtu_pru1":
            core = "RTUPRU1"
            break;
        case "icss_g0_rtu_pru0":
            core = "RTUPRU0"
            break;
        case "icss_g0_pru1":
            core = "PRU1"
            break;
        case "icss_g0_pru0":
            core = "PRU0"
    }

    return [
        " $(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix="+ core + "Firmware  -o "+ core.toLocaleLowerCase() + "_load_bin.h " + "ads85x8_" + board + "_" + cpu + "_fw_ti-pru-cgt.out; if ${CCS_HOST_OS} == win32 $(CCS_INSTALL_DIR)/utils/cygwin/sed -i '0r ${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/pru_load_bin_copyright.h' "+ core.toLocaleLowerCase() + "_load_bin.h ; if ${CCS_HOST_OS} == linux sed -i '0r ${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/pru_load_bin_copyright.h' "+ core.toLocaleLowerCase() + "_load_bin.h ;" + "if ${CCS_HOST_OS} == win32 move "+ core.toLocaleLowerCase() + "_load_bin.h " + "${MCU_PLUS_SDK_PATH}/examples/pru_io/adc/ads85x8/firmware/" + core.toLocaleLowerCase() + "_load_bin.h; if ${CCS_HOST_OS} == linux mv "+ core.toLocaleLowerCase() + "_load_bin.h " + "${MCU_PLUS_SDK_PATH}/examples/pru_io/adc/ads85x8/firmware/"+ core.toLocaleLowerCase() + "_load_bin.h "
    ];
}

const syscfgfile = "../example.syscfg"

const readmeDoxygenPageTag = "EXAMPLES_PRU_ADC_ADS85x8";

const templates_freertos_r5f =
[
    {
        input: ".project/templates/am64x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "ads_example_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "freertos", isPartOfSystemProject: true},
    { device: device, cpu: "icss_g0_pru0", cgt: "ti-pru-cgt", board: "am64x-evm", os: "fw", isPartOfSystemProject: true}
];

const systemProject = [
    {
        name: "ads85x8",
        tag: "freertos_prufw",
        skipProjectSpec: false,
        skipUpdatingTirex: true,
        readmeDoxygenPageTag: readmeDoxygenPageTag,
        board: "am64x-evm",
        projects: [
            { device: device, cpu: "icss_g0_pru0", cgt: "ti-pru-cgt", board: "am64x-evm", os: "fw"},
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "freertos"}      
        ],
    }
];


function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "ads85x8";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;
    property.isSkipTopLevelBuild = true;
    property.skipUpdatingTirex = true;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    if(buildOption.cpu == "r5fss0-0")
    {
        build_property.files = files_r5f;
        build_property.filedirs = filedirs_r5f;
        build_property.lnkfiles = lnkfiles_r5f;
        build_property.includes = includes_freertos_r5f;
        build_property.libdirs = libdirs_freertos_r5f;
        build_property.libs = libs_freertos_r5f;
        build_property.templates = templates_freertos_r5f;

    }

    else if(buildOption.cpu == "icss_g0_pru0")
    {
        build_property.files = files_pru;
        build_property.filedirs = filedirs_pru;
        build_property.lnkfiles = lnkfiles_pru;
        build_property.includes = includes_pru;
        build_property.templates = templates_pru;
        build_property.projecspecFileAction = "link";
        build_property.skipMakefileCcsBootimageGen = true;
        build_property.lflags = lflags_pru;
    
        build_property.makefile = "pru";  // makefile type
        build_property.ccsPruPostBuildSteps = getccsPruPostBuildSteps(buildOption.cpu, buildOption.board);
        build_property.makefilePruPostBuildSteps = getmakefilePruPostBuildSteps(buildOption.cpu, buildOption.board);
    }


    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;

    return build_property;
}

function getSystemProjects(device)
{
    return systemProject;
}

module.exports = {
    getSystemProjects,
    getComponentProperty,
    getComponentBuildProperty,
};
