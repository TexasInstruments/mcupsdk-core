let path = require('path');

let device = "am64x";

const files = {
    common: [
        "bootmode.c",
        "coeappl.c",
        "ecatappl.c",
        "ecatcoe.c",
        "ecateoe.c",
        "ecatfoe.c",
        "ecatslv.c",
        "eoeappl.c",
        "foeappl.c",
        "mailbox.c",
        "objdef.c",
        "sdoserv.c",
        "tiescsoc.c",
        "tiescappl.c",
        "tieschw.c",
        "tiesceoefoe.c",
        "tiescutils.c",
        "main.c",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../..",    /* Board base */
        "../../..", /* Example base */
        "../../../../../../source/industrial_comms/ethercat_slave/beckhoff_stack/stack_sources",
        "../../../../../../source/industrial_comms/ethercat_slave/beckhoff_stack/stack_hal",
    ],
};

const libdirs_freertos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/source/industrial_comms/ethercat_slave/icss_fwhal/lib",
    ],
};

const includes_freertos_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am64x/r5f",
        "${MCU_PLUS_SDK_PATH}/source/industrial_comms/ethercat_slave/beckhoff_stack/stack_sources",
        "${MCU_PLUS_SDK_PATH}/source/industrial_comms/ethercat_slave/beckhoff_stack/stack_hal",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_beckhoff_ssc_demo/",
    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "ethercat_slave_icss_fwhal.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const defines_r5f = {
    common: [
        "TIESC_APPLICATION=1",
    ],
};

const cflags_r5f = {
    common: [
        "-Wno-self-assign",
        "-Wno-parentheses-equality",
        "-Wno-tautological-constant-out-of-range-compare",
        "-Wno-address-of-packed-member",
        "-Wno-tautological-pointer-compare",
    ],
};

const lflags_r5f = {
    common: [
        "--use_memcpy=fast",
        "--use_memset=fast",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_INDUSTRIAL_COMMS_ETHERCAT_SLAVE_BECKHOFF_SSC_DEMO";

const templates_freertos_r5f =
[
    {
        input: ".project/templates/am64x/common/linker_r5f.cmd.xdt",
        output: "linker.cmd",
        options: {
            stackSize: "32768",
            codeDataAddr: "70080000",
            codeDataSize: "00080000",
        },
    },
    {
        input: ".project/templates/am64x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "ethercat_slave_beckhoff_ssc_demo_main",
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
    property.name = "ethercat_slave_beckhoff_ssc_demo";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;
    property.isSkipTopLevelBuild = true;
    property.skipProjectSpec = true;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.projecspecFileAction = "link";

    if(buildOption.cpu.match(/r5f*/)) {
        if(buildOption.os.match(/freertos*/) )
        {
            build_property.includes = includes_freertos_r5f;
            build_property.libdirs = libdirs_freertos;
            build_property.libs = libs_freertos_r5f;
            build_property.templates = templates_freertos_r5f;
            build_property.defines = defines_r5f;
            build_property.cflags = cflags_r5f;
            build_property.lflags = lflags_r5f;
        }
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
