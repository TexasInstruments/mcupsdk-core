const common = require("../common.js");

const component_file_list = [
    "source/board/.project/project.js",
    "source/drivers/.project/project.js",
    "source/middleware/.project/project.js",
    "source/kernel/nortos/.project/project.js",
    "source/kernel/freertos/.project/project.js",
    "test/unity/.project/project.js",
    "docs_src/docs/api_guide/doxy_samples/.project/project.js",
];

const device_defines = {
    common: [
        "SOC_AM62X",
    ],
};

const example_file_list = [
     "examples/hello_world/.project/project.js",
     "examples/kernel/dpl/dpl_demo/.project/project.js",
     "examples/empty/.project/project_freertos.js",
     "examples/empty/.project/project_nortos.js",
     "examples/drivers/gpio/gpio_input_interrupt/.project/project.js",
     "examples/drivers/gpio/gpio_led_blink/.project/project.js",
     "examples/drivers/i2c/i2c_read/.project/project.js",
     "examples/drivers/ipc/ipc_rpmsg_echo_linux/.project/project.js",
     "examples/drivers/mcan/mcan_loopback_polling/.project/project.js",
     "examples/drivers/mcan/mcan_loopback_interrupt/.project/project.js",
     "examples/drivers/mcspi/mcspi_loopback/.project/project.js",
     "examples/drivers/mcspi/mcspi_performance_8bit/.project/project.js",
     "examples/drivers/mcspi/mcspi_performance_32bit/.project/project.js",
     "examples/drivers/sciclient/sciclient_get_version/.project/project.js",
     "examples/drivers/uart/uart_echo/.project/project.js",
     "examples/drivers/uart/uart_echo_callback/.project/project.js",
     "examples/drivers/uart/uart_echo_low_latency_interrupt/.project/project.js",
     "examples/drivers/uart/uart_echo_low_latency_polling/.project/project.js",
     "test/kernel/dpl/.project/project.js",
     "test/kernel/freertos/.project/project.js",
     "test/drivers/i2c/.project/project.js",
     "test/drivers/sciclient/.project/project.js",
     "test/drivers/uart/.project/project.js",
     "test/drivers/gpio/.project/project.js",
];

function getProjectSpecCpu(cpu) {
    let projectSpecCpu =
    {
        "r5fss0-0": "MAIN_PULSAR_Cortex_R5_0_0",
        "m4fss0-0": "Cortex_M4F_0",
        "a53ss0-0": "CortexA53_0",
        "a53ss0-1": "CortexA53_1",
        "a53ss1-0": "CortexA53_2",
        "a53ss1-1": "CortexA53_3",
        "icssm-pru0": "ICSSM_PRU_0",
        "icssm-pru1": "ICSSM_PRU_1",
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
    return "AM62x";
}

function getProjectSpecDevice(board) {
    return "AM64x.AM62x_SK_EVM";
}

function getSysCfgCpu(cpu) {
    return cpu;
}

function getSysCfgPkg(board) {
    return "ALW";
}

function getSysCfgPart(board) {
    return "Default";
}

function getDevToolTirex(board) {
    return "AM62x_SK_EVM";
}

function getProperty() {
    let property = {};

    property.defines = device_defines;

    return property;
}

function getLinuxFwName(cpu) {

    switch(cpu) {
        case "m4fss0-0":
            return "am62-mcu-m4f0_0-fw";
    }
    return undefined;
}

function getProductNameProjectSpec() {
    return "MCU_PLUS_SDK_AM62X";
}

function getFlashAddr() {
    return 0x60000000;
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
