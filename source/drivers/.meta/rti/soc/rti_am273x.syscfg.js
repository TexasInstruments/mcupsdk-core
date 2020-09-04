
let common = system.getScript("/common");

let rtiClockSourceConfig = [
        {
            "name": "SYSCLK",
        },
];

function getRtiClockSourceValue(instance) {
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
        baseAddr: 0x02F7A000,
        hwiIntNum: 3,
        clkSelMuxAddr: 0x02100000 + 0x28,
        disableClkSourceConfig: false,
        lockUnlockDomain: "SOC_DOMAIN_ID_MSS_RCM",
        lockUnlockPartition: 0,
    },
    {
        name: "MSS_RTIB",
        baseAddr: 0x02F7A100,
        hwiIntNum: 9,
        clkSelMuxAddr: 0x02100000 + 0x2C,
        disableClkSourceConfig: false,
        lockUnlockDomain: "SOC_DOMAIN_ID_MSS_RCM",
        lockUnlockPartition: 0,
    },
    {
        name: "MSS_RTIC",
        baseAddr: 0x02F7A200,
        hwiIntNum: 13,
        clkSelMuxAddr: 0x02100000 + 0x30,
        disableClkSourceConfig: false,
        lockUnlockDomain: "SOC_DOMAIN_ID_MSS_RCM",
        lockUnlockPartition: 0,
    },
];

let staticConfig_c66 = [
    {
        name: "DSS_RTIA",
        baseAddr: 0x06F7A000,
        hwiIntNum: 66,
        clkSelMuxAddr: 0x06000000 + 0x94,
        disableClkSourceConfig: false,
        lockUnlockDomain: "SOC_DOMAIN_ID_DSS_RCM",
        lockUnlockPartition: 0,
    },
    {
        name: "DSS_RTIB",
        baseAddr: 0x06F7A100,
        hwiIntNum: 72,
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

function getRtiClockSourceConfigArr() {

    return rtiClockSourceConfig;
}

function getRtiClockSourceHz(clkSource) {

    return 200000000;
}

function getBlockedTimers() {

    let cpu = common.getSelfSysCfgCoreName();

    if(cpu.match(/r5f*/)) {
        return ['MSS_RTIA', 'MSS_RTIB'];
    }
    else
    {
        return ['DSS_RTIA'];
    }

}

exports = {
    getStaticConfigArr,
    getRtiClockSourceConfigArr,
    getRtiClockSourceValue,
    getRtiClockSourceHz,
    getInterfaceName,
    getBlockedTimers,
    ntuSupport: false,
};
