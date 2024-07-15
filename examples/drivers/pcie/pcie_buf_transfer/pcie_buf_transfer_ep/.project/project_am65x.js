let path = require('path');

let device = "am65x";

const files = {
    common: [
        "pcie_buf_transfer_ep.c",
        "main.c",
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

const includes_freertos_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am65x/r5f",
    ],
};

const libdirs_freertos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
    ],
};


const libs_freertos_r5f = {
    common: [
        "freertos.am65x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am65x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg"

const readmeDoxygenPageTag = "EXAMPLES_DRIVERS_PCIE_BUF_TRANSFER_EP";

const templates_freertos_r5f =
    [
        {
            input: ".project/templates/am65x/common/linker_r5f.cmd.xdt",
            output: "linker.cmd",
        },
        {
            input: ".project/templates/am65x/freertos/main_freertos.c.xdt",
            output: "../main.c",
            options: {
                entryFunction: "pcie_buf_transfer_ep_main",
            },
        }
    ];

const buildOptionCombos = [
    { device: "am65x", cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am65x-idk", os: "freertos" },
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "pcie_buf_transfer_ep";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;
    property.isPCIEdstBuf = true;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.libdirs = libdirs_freertos;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;

    if (buildOption.cpu.match(/r5f*/)) {
        build_property.libs = libs_freertos_r5f;
        build_property.templates = templates_freertos_r5f;
        build_property.includes = includes_freertos_r5f;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
