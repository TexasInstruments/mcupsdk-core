
let common = system.getScript("/common");

let staticConfig_m4f = [
    {
        name: "M4_SYSTICK",
        timerBaseAddr: 0xE000E010, /* Setting to SYST_CSR as defined by ARMv7-M */
        timerHwiIntNum: 15,
        timerInputPreScaler: 1, /* NOT USED */
        clkSelMuxAddr: 0, /* NOT USED */
        disableClkSourceConfig: true,
    }
];

let timerClockSourceConfig_m4f = {
    name: "clkSource",
    displayName: "Input Clock Source",
    default: "M4_CPU_CLK",
    options: [
        {
            "name": "M4_CPU_CLK",
        },
    ],
    readOnly: true,
};

function getTimerClockSourceValue(instance) {
    let clkSelMuxValue = 0;

    switch(instance.clkSource) {
        default:
        case "MCU_HFOSC0":
            clkSelMuxValue = 0x0;
            break;
    }
    return clkSelMuxValue;
}

function makeInstanceConfig() {
    let config = {};
    let staticConfigArr = getStaticConfigArr();
    let defaultInstanceIndex = getDefaultInstance();

    if(staticConfigArr.length == 0)
        return undefined;

    config.name = "instance";
    config.displayName = "Instance";
    config.description = "Select Instance";
    config.default = staticConfigArr[defaultInstanceIndex].name;
    config.options = [];

    for (let i = 0; i < staticConfigArr.length; i++) {
        let option = {};

        option.name = staticConfigArr[i].name;
        config.options.push(option);
    }

    return config;
}

function getDefaultInstance() {
    let cpu = common.getSelfSysCfgCoreName();
    let defaultInstanceMap = {
        "m4fss0-0": 0,
    }
    return defaultInstanceMap[cpu];
}

function getStaticConfigArr() {
    let cpu = common.getSelfSysCfgCoreName();
    let staticConfigArr;

    if(cpu.match(/m4f*/)) {
        staticConfigArr = staticConfig_m4f;
    }

    return staticConfigArr;
}

function getTimerClockSourceConfigArr() {
    let cpu = common.getSelfSysCfgCoreName();
    let timerClockSourceConfig = timerClockSourceConfig_m4f;

    if(cpu.match(/m4f*/)) {
        timerClockSourceConfig = timerClockSourceConfig_m4f;
    }

    return timerClockSourceConfig;
}

function getDefaultTimerClockSourceMhz() {
    let cpu = common.getSelfSysCfgCoreName();

    if(cpu.match(/m4f*/)) {
        return 400*1000000;
    }

    return 25000000;
}

exports = {
    getStaticConfigArr,
    getTimerClockSourceConfigArr,
    getTimerClockSourceValue,
    getDefaultTimerClockSourceMhz,
    makeInstanceConfig,
};
