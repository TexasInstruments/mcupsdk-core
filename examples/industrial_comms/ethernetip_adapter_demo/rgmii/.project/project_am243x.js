let path = require('path');

let device = "am243x";

const files = {
    common: [
        "app.c",
        "appClass71.c",
        "appPerm.c",
        "appProduct.c",
        "appWebServer.c",
        "appPhyReset.c",
        "CUST_drivers.c",
        "CUST_eeprom.c",
        "CUST_phy.c",
        "CUST_flash.c",
        "CUST_led.c",
        "CUST_uart.c",
        "CUST_PHY_base.c",
        "CUST_PHY_dp83869.c",
        "CMN_CPU_main.c",
        "CMN_app.c",
        "CMN_board.c",
        "CMN_os.c",
    ],
};
/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs_evm = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
        "../../../..",
        "../../../../board/am243x-evm/freertos/drivers",
        "../../../../board/am243x-evm/freertos/drivers/eeprom",
        "../../../../board/am243x-evm/freertos/drivers/ethphy",
        "../../../../board/am243x-evm/freertos/drivers/flash",
        "../../../../board/am243x-evm/freertos/drivers/led",
        "../../../../board/am243x-evm/freertos/drivers/uart",
        "../../../../customPhy/src",
        "../../../../os/freertos",
    ],
};

const filedirs_lp = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
        "../../../..",
        "../../../../board/am243x-lp/freertos/drivers",
        "../../../../board/am243x-lp/freertos/drivers/eeprom",
        "../../../../board/am243x-lp/freertos/drivers/ethphy",
        "../../../../board/am243x-lp/freertos/drivers/flash",
        "../../../../board/am243x-lp/freertos/drivers/led",
        "../../../../board/am243x-lp/freertos/drivers/uart",
        "../../../../customPhy/src",
        "../../../../os/freertos",
    ],
};

const libdirs_freertos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/icss_emac/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/icss_timesync/lib",
        "${MCU_PLUS_SDK_PATH}/source/industrial_comms/ethernetip_adapter/icss_fwhal/lib",
        "${MCU_PLUS_SDK_IND_COMMS_LIBS_PATH}/ethernetip_adapter",
        "${MCU_PLUS_SDK_PATH}/source/industrial_comms/ethernetip_adapter/stack/lwip/lib",
    ],
};

const includes_freertos_r5f_evm = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am243x/r5f",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethernetip_adapter_demo",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethernetip_adapter_demo/board/am243x-evm/freertos",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethernetip_adapter_demo/customPhy/inc",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethernetip_adapter_demo/os/freertos",
        "${MCU_PLUS_SDK_PATH}/source/industrial_comms/ethernetip_adapter/stack",
        "${MCU_PLUS_SDK_PATH}/source/industrial_comms/ethernetip_adapter/stack/inc",
        "${MCU_PLUS_SDK_PATH}/source/industrial_comms/ethernetip_adapter/stack/lwip/lwip-config/am243x",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/freertos/include",
    ],
};

const includes_freertos_r5f_lp = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am243x/r5f",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethernetip_adapter_demo",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethernetip_adapter_demo/board/am243x-lp/freertos",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethernetip_adapter_demo/customPhy/inc",
        "${MCU_PLUS_SDK_PATH}/examples/industrial_comms/ethernetip_adapter_demo/os/freertos",
        "${MCU_PLUS_SDK_PATH}/source/industrial_comms/ethernetip_adapter/stack",
        "${MCU_PLUS_SDK_PATH}/source/industrial_comms/ethernetip_adapter/stack/inc",
        "${MCU_PLUS_SDK_PATH}/source/industrial_comms/ethernetip_adapter/stack/lwip/lwip-config/am243x",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/freertos/include",
    ],
};

const libs_freertos_r5f_evm = {
    common: [
        "freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "icss_emac.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "icss_timesync.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "ethernetip_adapter_rgmii_icss_fwhal.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "ethernetip_adapter_lwip_contrib.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "ethernetip_adapter_lwip_freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "ethernetipadapter_rgmii.am243x-evm.r5f.ti-arm-clang.release.lib",
    ],
};

const libs_freertos_r5f_lp = {
    common: [
        "freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "icss_emac.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "icss_timesync.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "ethernetip_adapter_rgmii_icss_fwhal.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "ethernetip_adapter_lwip_contrib.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "ethernetip_adapter_lwip_freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "ethernetipadapter_rgmii.am243x-lp.r5f.ti-arm-clang.release.lib",
    ],
};

const defines_r5f = {
    common: [
        "TIME_SYNC",
        "CPU_LOAD_MONITOR=0"
    ],
};

const cflags_r5f = {
    common: [
        "-mllvm -align-all-functions=2",
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

const readmeDoxygenPageTag = "EXAMPLES_INDUSTRIAL_COMMS_ETHERNETIP_ADAPTER_DEMOS";

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "freertos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "ethernetip_adapter_rgmii_demo";
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
            build_property.defines = defines_r5f;
            build_property.cflags = cflags_r5f;
            build_property.lflags = lflags_r5f;

            if(buildOption.board.match(/am243x-evm*/) )
            {
                build_property.filedirs = filedirs_evm;
                build_property.includes = includes_freertos_r5f_evm;
                build_property.libs = libs_freertos_r5f_evm;
            }
            else if(buildOption.board.match(/am243x-lp*/) )
            {
                build_property.filedirs = filedirs_lp;
                build_property.includes = includes_freertos_r5f_lp;
                build_property.libs = libs_freertos_r5f_lp;
            }
        }
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
