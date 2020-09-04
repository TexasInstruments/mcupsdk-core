let path = require('path');

let device = "am243x";

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
const filedirs_evm = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
        "../../../../common/board/am243evm/freertos",
        "../../../../common/os/freertos",
        "../../../../common",
        "../../../../customPhy/src",
    ],
};

const filedirs_lp = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
        "../../../../common/board/am243lp/freertos",
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

const includes_freertos_r5f_evm = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am243x/r5f",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_demo/ctt",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_demo/common",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_demo/common/os",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_demo/common/os/freertos",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_demo/common/board/am243evm",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_demo/common/board/am243evm/freertos",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_demo/customPhy/inc",
        "${MCU_PLUS_SDK_PATH}/source/industrial_comms/ethercat_slave/stack/inc",
    ],
};

const includes_freertos_r5f_lp = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am243x/r5f",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_demo/ctt",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_demo/common",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_demo/common/os",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_demo/common/os/freertos",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_demo/common/board/am243lp",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_demo/common/board/am243lp/freertos",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethercat_slave_demo/customPhy/inc",
        "${MCU_PLUS_SDK_PATH}/source/industrial_comms/ethercat_slave/stack/inc",
    ],
};

const libs_freertos_r5f_evm = {
    common: [
        "freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "ethercat_slave.am243x_evm.r5f.ti-arm-clang.release.lib",
        "ethercat_slave_bkhfSsc.am243x_evm.r5f.ti-arm-clang.release.lib"
    ],
};

const libs_freertos_r5f_lp = {
    common: [
        "freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "ethercat_slave.am243x_lp.r5f.ti-arm-clang.release.lib",
        "ethercat_slave_bkhfSsc.am243x_lp.r5f.ti-arm-clang.release.lib"
    ],
};

const defines_r5f_evm = {
    common: [
       "SOC_AM243X=1",
       "OSAL_FREERTOS=1",
       "core0",
       "am243x",
       "am243x_evm",
       "SSC_CHECKTIMER=1",
       "USE_ECAT_TIMER=1",
    ],
};

const defines_r5f_lp = {
    common: [
       "SOC_AM243X=1",
       "OSAL_FREERTOS=1",
       "core0",
       "am243x",
       "am243x_lp",
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
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "freertos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "ethercat_slave_ctt_demo";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.projecspecFileAction = "link";

    if(buildOption.cpu.match(/r5f*/)) {
        if(buildOption.os.match(/freertos*/) )
        {
            build_property.libdirs = libdirs_freertos;
            build_property.cflags = cflags_r5f;
            build_property.lflags = lflags_r5f;

            if(buildOption.board.match(/am243x-evm*/) )
            {
                build_property.defines = defines_r5f_evm;
                build_property.libs = libs_freertos_r5f_evm;
                build_property.filedirs = filedirs_evm;
                build_property.includes = includes_freertos_r5f_evm;
            }
            else if(buildOption.board.match(/am243x-lp*/) )
            {
                build_property.defines = defines_r5f_lp;
                build_property.libs = libs_freertos_r5f_lp;
                build_property.filedirs = filedirs_lp;
                build_property.includes = includes_freertos_r5f_lp;
            }
        }
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
