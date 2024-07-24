const common = require("../common.js");

const component_file_list = [
    "source/board/.project/project.js",
    "source/drivers/.project/project.js",
    "source/fs/freertos_fat/.project/project.js",
    "source/middleware/.project/project.js",
    "source/kernel/freertos/.project/project.js",
    "source/kernel/nortos/.project/project.js",
    "test/unity/.project/project.js",
    "docs_src/docs/api_guide/doxy_samples/.project/project.js",
];

const device_defines = {
    common: [
        "SOC_AM65X",
    ],
};

const example_file_list = [
    "examples/drivers/boot/sbl_null/.project/project.js",
    "examples/drivers/boot/sbl_ospi/.project/project.js",
    "examples/drivers/boot/sbl_ospi_linux/.project/project.js",
    "examples/drivers/boot/sbl_sd/.project/project.js",
    "examples/drivers/boot/sbl_pcie/.project/project.js",
    "examples/drivers/boot/sbl_pcie_host/.project/project.js",
    "examples/drivers/boot/sbl_uart/.project/project.js",
    "examples/drivers/boot/sbl_uart_uniflash/.project/project.js",
    "examples/drivers/gpio/gpio_input_interrupt/.project/project.js",
    "examples/drivers/gpio/gpio_led_blink/.project/project.js",
    "examples/drivers/i2c/i2c_led_blink/.project/project.js",
    "examples/drivers/i2c/i2c_read/.project/project.js",
    "examples/drivers/i2c/i2c_temperature/.project/project.js",
    "examples/drivers/ipc/ipc_rpmsg_echo_linux/.project/project.js",
    "examples/drivers/mmcsd/mmcsd_file_io/.project/project.js",
    "examples/drivers/mmcsd/mmcsd_raw_io/.project/project.js",
    "examples/drivers/ospi/ospi_flash_diag/.project/project.js",
    "examples/drivers/ospi/ospi_flash_dma/.project/project.js",
    "examples/drivers/ospi/ospi_flash_io/.project/project.js",
    "examples/drivers/pcie/pcie_buf_transfer/pcie_buf_transfer_ep/.project/project.js",
    "examples/drivers/pcie/pcie_buf_transfer/pcie_buf_transfer_rc/.project/project.js",
    "examples/drivers/sciclient/sciclient_get_version/.project/project.js",
    "examples/drivers/sciclient/sciclient_set_boardcfg/.project/project.js",
    "examples/drivers/uart/uart_echo/.project/project.js",
    "examples/drivers/uart/uart_echo_callback/.project/project.js",
    "examples/drivers/uart/uart_echo_dma/.project/project.js",
    "examples/drivers/uart/uart_echo_low_latency_interrupt/.project/project.js",
    "examples/drivers/uart/uart_echo_low_latency_polling/.project/project.js",
    "examples/drivers/udma/udma_chaining/.project/project.js",
    "examples/drivers/udma/udma_memcpy_interrupt/.project/project.js",
    "examples/drivers/udma/udma_memcpy_polling/.project/project.js",
    "examples/drivers/udma/udma_sw_trigger/.project/project.js",
    "examples/empty/.project/project_freertos.js",
    "examples/hello_world/.project/project.js",
    "examples/hello_world_cpp/.project/project.js",
    "examples/kernel/dpl/dpl_demo/.project/project.js",
    "examples/kernel/freertos/posix_demo/.project/project.js",
    "examples/kernel/freertos/task_switch/.project/project.js",
    "test/drivers/gpio/.project/project.js",
    "test/drivers/i2c/.project/project.js",
    "test/drivers/ipc_notify/.project/project.js",
    "test/drivers/ipc_rpmsg/.project/project.js",
    "test/drivers/mmcsd/mmcsd/.project/project.js",
    "test/drivers/ospi/.project/project.js",
    "test/drivers/sciclient/.project/project.js",
    "test/drivers/uart/.project/project.js",
    "test/kernel/dpl/.project/project.js",
    "test/kernel/freertos/.project/project.js",
];

function getProjectSpecCpu(cpu) {
    let projectSpecCpu =
    {
        "r5fss0-0": "MAIN_PULSAR_Cortex_R5_0_0",
        "r5fss0-1": "MAIN_PULSAR_Cortex_R5_0_1",
    }

    return projectSpecCpu[cpu];
}

function getComponentList() {
    return component_file_list;
}

function getExampleList() {
    return example_file_list;
}

function getSysCfgDevice(board) {
    return "AM65xx_SR2.0_beta";
}

function getProjectSpecDevice(board) {
    return "AM6548.IDK_AM65x";
}

function getSysCfgCpu(cpu) {
    return cpu;
}

function getSysCfgPkg(board) {
    return "ACD";
}

function getSysCfgPart(board) {
    return "Default";
}

function getDevToolTirex(board) {
    return "AM65x_GP_EVM";
}

function getProperty() {
    let property = {};

    property.defines = device_defines;

    return property;
}

function getLinuxFwName(cpu) {

    switch(cpu) {
        case "r5fss0-0":
            return "am65-mcu-r5f0_0-fw";
        case "r5fss0-1":
            return "am65-mcu-r5f0_1-fw";
    }
    return undefined;
}

function getProductNameProjectSpec() {
    return "MCU_PLUS_SDK_AM65X";
}

function getFlashAddr() {
    return 0x58000000;
}

module.exports = {
    getComponentList,
    getExampleList,
    getSysCfgDevice,
    getSysCfgCpu,
    getSysCfgPkg,
    getSysCfgPart,
    getProjectSpecDevice,
    getProjectSpecCpu,
    getDevToolTirex,
    getProperty,
    getLinuxFwName,
    getProductNameProjectSpec,
    getFlashAddr,
};
