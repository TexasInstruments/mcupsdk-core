
let common = system.getScript("/common");

let timerClockSourceConfig = [
        {
            "name": "SYSCLK",
        },
];

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
        timerName: "MSS_RTIA",
        timerBaseAddr: 0x02F7A000,
        timerHwiIntNum: 3,
        timerInputPreScaler: 1,
        clkSelMuxAddr: 0x02100000 + 0x28,
        disableClkSourceConfig: false,
        lockUnlockDomain: "SOC_DOMAIN_ID_MSS_RCM",
        lockUnlockPartition: 0,
    },
    {
        timerName: "MSS_RTIB",
        timerBaseAddr: 0x02F7A100,
        timerHwiIntNum: 9,
        timerInputPreScaler: 1,
        clkSelMuxAddr: 0x02100000 + 0x2C,
        disableClkSourceConfig: false,
        lockUnlockDomain: "SOC_DOMAIN_ID_MSS_RCM",
        lockUnlockPartition: 0,
    },
    {
        timerName: "MSS_RTIC",
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
        timerName: "DSS_RTIA",
        timerBaseAddr: 0x06F7A000,
        timerHwiIntNum: 66,
        timerInputPreScaler: 1,
        clkSelMuxAddr: 0x06000000 + 0x94,
        disableClkSourceConfig: false,
        lockUnlockDomain: "SOC_DOMAIN_ID_DSS_RCM",
        lockUnlockPartition: 0,
    },
    {
        timerName: "DSS_RTIB",
        timerBaseAddr: 0x06F7A100,
        timerHwiIntNum: 72,
        timerInputPreScaler: 1,
        clkSelMuxAddr: 0x06000000 + 0x98,
        disableClkSourceConfig: false,
        lockUnlockDomain: "SOC_DOMAIN_ID_DSS_RCM",
        lockUnlockPartition: 0,
    },
];

function getStaticConfigArr() {
    let cpu = common.getSelfSysCfgCoreName();
    let staticConfigArr;

    if(cpu.match(/r5f*/)) {
        staticConfigArr = staticConfig_r5f;
    }
    else
    {
        staticConfigArr = staticConfig_c66;
    }

    return staticConfigArr;
}

function getInterfaceName(inst) {
    let cpu = common.getSelfSysCfgCoreName();

    if(cpu.match(/r5f*/)) {
        return "MSS_RTI";
    }
    return "DSS_RTI";
}

function getTimerClockSourceConfigArr() {

    return timerClockSourceConfig;
}

function getTimerClockSourceHz(clkSource) {

    return 200000000;
}

function getBlockedTimers() {

    let cpu = common.getSelfSysCfgCoreName();
    let blockTimers = [];

    if(cpu.match(/r5f*/)) {
        if(system.contexts["r5fss0-0"])
            blockTimers.push('MSS_RTIA');
        if(system.contexts["r5fss0-1"])
            blockTimers.push('MSS_RTIB');
    }
    else
    {
        if(system.contexts["c66ss0"])
            blockTimers.push('DSS_RTIA');
    }
    return blockTimers;
}

exports = {
    getStaticConfigArr,
    getTimerClockSourceConfigArr,
    getTimerClockSourceValue,
    getTimerClockSourceHz,
    getInterfaceName,
    getBlockedTimers,
    oneShotModeSupport: false,
};
