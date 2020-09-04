let path = require('path');

let device = "am64x";

const files = {
    common: [
        "ESL_BOARD_OS_config.c",
        "ESL_eeprom.c",
        "ESL_fileHandling.c",
        "ESL_foeDemo.c",
        "ESL_gpioHelper.c",
        "ESL_OS_os.c",
        "ESL_soeDemo.c",
        "ecSlvSimple.c",
        "EtherCAT_Slave_Simple.c",
        "ESL_version.c",
        "CUST_PHY_base.c",
        "CUST_PHY_dp83869.c",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
        "../../../../common/board/am64gpevm/freertos",
        "../../../../common/os/freertos",
        "../../../../common",
        "../../../../customPhy/src",
    ],
};

const libdirs_freertos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_IND_COMMS_LIBS_PATH}/ethercat_slave",
    ],
};

const includes_freertos_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am64x/r5f",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_demo/simple",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_demo/common",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_demo/common/os",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_demo/common/os/freertos",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_demo/common/board/am64gpevm",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_demo/common/board/am64gpevm/freertos",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_demo/customPhy/inc",
        "${MCU_PLUS_SDK_PATH}/source/industrial_comms/ethercat_slave/stack/inc",
    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "ethercat_slave.am64x.r5f.ti-arm-clang.release.lib",
        "ethercat_slave_bkhfSsc.am64x.r5f.ti-arm-clang.release.lib"
    ],
};

const defines_r5f = {
    common: [
       "SOC_AM64X=1",
       "OSAL_FREERTOS=1",
       "core0",
       "am64x",
       "am64x_evm",
       "SSC_CHECKTIMER=1",
       "USE_ECAT_TIMER=1",
    ],
};

const cflags_r5f = {
    common: [
        "-Wno-unused-but-set-variable",
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

const readmeDoxygenPageTag = "EXAMPLES_INDUSTRIAL_COMMS_ETHERCAT_SLAVE_DEMOS";

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "freertos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "ethercat_slave_simple_demo";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

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
