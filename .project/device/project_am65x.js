const common = require("../common.js");

const component_file_list = [
    "source/drivers/.project/project.js",
    "source/kernel/freertos/.project/project.js",
    "source/kernel/nortos/.project/project.js",
    "test/unity/.project/project.js",
];

const device_defines = {
    common: [
        "SOC_AM65X",
    ],
};

const example_file_list = [
    "examples/drivers/sciclient/sciclient_get_version/.project/project.js",
    "examples/drivers/sciclient/sciclient_set_boardcfg/.project/project.js",
    "examples/empty/.project/project_freertos.js",
    "examples/hello_world/.project/project.js",
    "examples/hello_world_cpp/.project/project.js",
    "examples/kernel/dpl/dpl_demo/.project/project.js",
    "examples/kernel/freertos/posix_demo/.project/project.js",
    "examples/kernel/freertos/task_switch/.project/project.js",
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
