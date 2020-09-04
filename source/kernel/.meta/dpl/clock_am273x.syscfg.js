
let common = system.getScript("/common");

let timerClockSourceConfig = {
    name: "clkSource",
    displayName: "Input Clock Source",
    default: "SYSCLK",
    options: [
        {
            "name": "SYSCLK",
        },
    ],
};

function getTimerClockSourceValue(instance) {
    let clkSelMuxValue = 0;

    switch(instance.clkSource) {
        default:
        case "SYSCLK":
            clkSelMuxValue = 0x222;
            break;
    }
    return clkSelMuxValue;
}

let staticConfig_r5f = [
    {
        name: "MSS_RTIA",
        timerBaseAddr: 0x02F7A000,
        timerHwiIntNum: 3,
        timerInputPreScaler: 1,
        clkSelMuxAddr: 0x02100000 + 0x28,
        disableClkSourceConfig: false,
        lockUnlockDomain: "SOC_DOMAIN_ID_MSS_RCM",
        lockUnlockPartition: 0,
    },
    {
        name: "MSS_RTIB",
        timerBaseAddr: 0x02F7A100,
        timerHwiIntNum: 9,
        timerInputPreScaler: 1,
        clkSelMuxAddr: 0x02100000 + 0x2C,
        disableClkSourceConfig: false,
        lockUnlockDomain: "SOC_DOMAIN_ID_MSS_RCM",
        lockUnlockPartition: 0,
    },
    {
        name: "MSS_RTIC",
        timerBaseAddr: 0x02F7A200,
        timerHwiIntNum: 13,
        timerInputPreScaler: 1,
        clkSelMuxAddr: 0x02100000 + 0x30,
        disableClkSourceConfig: false,
        lockUnlockDomain: "SOC_DOMAIN_ID_MSS_RCM",
        lockUnlockPartition: 0,
    },
];

let staticConfig_c66 = [
    {
        name: "DSS_RTIA",
        timerBaseAddr: 0x06F7A000,
        timerHwiIntNum: 66,
        timerInputPreScaler: 1,
        clkSelMuxAddr: 0x06000000 + 0x94,
        disableClkSourceConfig: false,
        lockUnlockDomain: "SOC_DOMAIN_ID_DSS_RCM",
        lockUnlockPartition: 0,
    },
    {
        name: "DSS_RTIB",
        timerBaseAddr: 0x06F7A100,
        timerHwiIntNum: 72,
        timerInputPreScaler: 1,
        clkSelMuxAddr: 0x06000000 + 0x98,
        disableClkSourceConfig: false,
        lockUnlockDomain: "SOC_DOMAIN_ID_DSS_RCM",
        lockUnlockPartition: 0,
    },
];

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
        "r5fss0-0": 0,
        "r5fss0-1": 1,
        "c66ss0": 0,
        "hsm0-0": 0,
    }
    return defaultInstanceMap[cpu];
}

function getStaticConfigArr() {
    let cpu = common.getSelfSysCfgCoreName();
    let staticConfigArr;

    if(cpu.match(/r5f*/)) {
        staticConfigArr = staticConfig_r5f;
    }
    else if(cpu.match(/hsm*/) || cpu.match(/m4f*/)) {
        staticConfigArr = system.getScript(`/imports/kernel/dpl/clock_${common.getSocName()}_hsm.syscfg.js`).staticConfig_m4f;
    }
    else
    {
        staticConfigArr = staticConfig_c66;
    }

    return staticConfigArr;
}

function getTimerClockSourceConfigArr() {

    return timerClockSourceConfig;
}

function getDefaultTimerClockSourceMhz() {

    return 200000000;
}

exports = {
    getStaticConfigArr,
    getTimerClockSourceConfigArr,
    getTimerClockSourceValue,
    getDefaultTimerClockSourceMhz,
    makeInstanceConfig,
};
