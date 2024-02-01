let path = require('path');

let device = "am64x";

const files = {
    common: [
        "main.c",
        "empty_example.c",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
    ],
};

const libdirs_freertos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/source/pru_io/lib",
    ],
};

const includes_freertos_r5f_am64x_evm = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am64x/r5f",
        "${MCU_PLUS_SDK_PATH}/source/pru_io/driver",
        "${MCU_PLUS_SDK_PATH}/examples/pru_io/empty/firmware/am64x-evm/icssg0-pru0_fw/ti-pru-cgt",
    ],
};



const libs_freertos_r5f = {
    common: [
        "freertos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "../linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg"

const readmeDoxygenPageTag = "EXAMPLES_PRU_EMPTY";

const templates_freertos_r5f =
[
    {
        input: ".project/templates/am64x/common/linker_r5f.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/am64x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "pru_io_empty_example_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "freertos"},
];


function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "empty_pru_io";
    property.isInternal = false;
    property.description = "R5F EMPTY PRU IO Project"
    property.buildOptionCombos = buildOptionCombos;
    property.isSkipTopLevelBuild = true;
    property.skipUpdatingTirex = true;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.importOtherProject="${COM_TI_MCU_PLUS_SDK_AMXXX_INSTALL_DIR}/examples/pru_io/empty/firmware/am64x-evm/icssg0-pru0_fw/ti-pru-cgt/example.projectspec";
    build_property.preBuildSteps = [
        "$(MAKE) -C ${MCU_PLUS_SDK_PATH}/examples/pru_io/empty/firmware/am64x-evm/icssg0-pru0_fw/ti-pru-cgt -f makefile -k clean MCU_PLUS_SDK_PATH=${MCU_PLUS_SDK_PATH} CCS_INSTALL_DIR=${CCS_INSTALL_DIR} CCS_PROJECT_DEBUG=${CWD};",
        "$(MAKE) -C ${MCU_PLUS_SDK_PATH}/examples/pru_io/empty/firmware/am64x-evm/icssg0-pru0_fw/ti-pru-cgt -f makefile -k all MCU_PLUS_SDK_PATH=${MCU_PLUS_SDK_PATH} CCS_INSTALL_DIR=${CCS_INSTALL_DIR} CCS_PROJECT_DEBUG=${CWD};",
    ];
    build_property.includes = includes_freertos_r5f_am64x_evm;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.libdirs = libdirs_freertos;
    build_property.libs = libs_freertos_r5f;
    build_property.templates = templates_freertos_r5f;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
