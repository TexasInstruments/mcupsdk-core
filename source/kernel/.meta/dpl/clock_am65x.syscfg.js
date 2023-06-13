
let common = system.getScript("/common");

let timerClockSourceConfig_r5f = {
    name: "clkSource",
    displayName: "Input Clock Source",
    default: "WKUP_HFOSC0",
    options: [
        {
            "name": "WKUP_HFOSC0",
        },
    ],
};

function getTimerClockSourceValue(instance){
    let clkSelMuxValue = 0;

    switch(instance.clkSource) {
        default:
        case "WKUP_HFOSC0":
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
        "r5fss0-0": 0,
        "r5fss0-1": 1,
    }
    return defaultInstanceMap[cpu];
}

function getStaticConfigArr() {
    let cpu = common.getSelfSysCfgCoreName();
    let staticConfigArr;

    if(cpu.match(/r5f*/)) {
        let staticConfig_r5f = [];

        for(let i=2; i<4; i++)
        {
            staticConfig_r5f.push(
                {
                    name: `TIMER${i}`,
                    timerBaseAddr: 0x40400000 + i*0x10000,
                    timerHwiIntNum: 38 + i,
                    timerInputPreScaler: 1,
                    clkSelMuxAddr: 0x40F08100 + 4*i,
                    disableClkSourceConfig: false,
                    lockUnlockDomain: "SOC_DOMAIN_ID_MCU",
                    lockUnlockPartition: 2,
                }
            )
        }
        staticConfigArr = staticConfig_r5f;
    }

    return staticConfigArr;
}

function getTimerClockSourceConfigArr() {
    let cpu = common.getSelfSysCfgCoreName();
    let timerClockSourceConfig = timerClockSourceConfig_r5f;

    if(cpu.match(/r5f*/)) {
        timerClockSourceConfig = timerClockSourceConfig_r5f;
    }

    return timerClockSourceConfig;
}

function getDefaultTimerClockSourceMhz() {
    let cpu = common.getSelfSysCfgCoreName();

    return 25000000;
}

exports = {
    getStaticConfigArr,
    getTimerClockSourceConfigArr,
    getTimerClockSourceValue,
	getDefaultTimerClockSourceMhz,
    makeInstanceConfig,
};
