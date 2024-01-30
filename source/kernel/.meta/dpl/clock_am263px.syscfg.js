
let common = system.getScript("/common");

let timerClockSourceConfig_r5f = {
    name: "clkSource",
    displayName: "Input Clock Source",
    default: "WUCPUCLK",
    options: [
        {
            "name": "WUCPUCLK",
        },
        {
            "name": "XTALCLK",
        },
        {
            "name": "SYS_CLK",
        },
        {
            "name": "EXT_REFCLK",
        },
        {
            "name": "RCCLK10M",
        },
        {
            "name": "CTPS_GENF0",
        },
        {
            "name": "DPLL_CORE_HSDIV0_CLKOUT1",
        },
        {
            "name": "DPLL_PER_HSDIV0_CLKOUT1",
        }
    ],
};

function getTimerClockSourceValue(instance) {
    let clkSelMuxValue = 0;

    switch(instance.clkSource) {
        default:
        case "WUCPUCLK":
            clkSelMuxValue = 0x000;
            break;
        case "XTALCLK":
            clkSelMuxValue = 0x666;
            break;
        case "SYS_CLK":
            clkSelMuxValue = 0x222;
            break;
        case "EXT_REFCLK":
            clkSelMuxValue = 0x111;
            break;
        case "RCCLK10M":
            clkSelMuxValue = 0x555;
            break;
        case "CTPS_GENF0":
            clkSelMuxValue = 0x777;
            break;
        case "DPLL_CORE_HSDIV0_CLKOUT1":
            clkSelMuxValue = 0x444;
            break;
        case "DPLL_PER_HSDIV0_CLKOUT1":
            clkSelMuxValue = 0x333;
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
        "r5fss1-0": 2,
        "r5fss1-1": 3,
        "hsm0-0": 0,
    }
    return defaultInstanceMap[cpu];
}

function getStaticConfigArr() {
    let cpu = common.getSelfSysCfgCoreName();
    let staticConfigArr;

    if(cpu.match(/r5f*/)) {
        let staticConfig_r5f = [];
        for(let i=0; i<4; i++)
        {
            staticConfig_r5f.push(
                {
                    name: `RTI${i}`,
                    timerBaseAddr: 0x52180000 + i*0x1000,
                    /* RTI Interrupts are not continous. RTI 0-3 are grouped together and RTI 4-7 are grouped separately */
                    timerHwiIntNum: ((i < 4) ? (84 + (7 * i)) : (219 + (7 * (i - 4)))),
                    timerInputPreScaler: 1,
                    clkSelMuxAddr: 0x53208000 + 0x114 + 4*i,
                    disableClkSourceConfig: false,
                    lockUnlockDomain: "SOC_DOMAIN_ID_MAIN",
                    lockUnlockPartition: "MSS_RCM_PARTITION0",
                }
            )
        }
        staticConfigArr = staticConfig_r5f;
    }
    if(cpu.match(/hsm*/) || cpu.match(/m4f*/)) {
        staticConfigArr = system.getScript(`/imports/kernel/dpl/clock_${common.getSocName()}_hsm.syscfg.js`).staticConfig_m4f;

    }
    return staticConfigArr;
}

function getTimerClockSourceConfigArr() {
    let cpu = common.getSelfSysCfgCoreName();
    let timerClockSourceConfig = timerClockSourceConfig_r5f;

    if(cpu.match(/r5f*/)) {
        timerClockSourceConfig = timerClockSourceConfig_r5f;
    }
    if(cpu.match(/hsm*/) || cpu.match(/m4f*/)) {
        timerClockSourceConfig = system.getScript(`/imports/kernel/dpl/clock_${common.getSocName()}_hsm.syscfg.js`).timerClockSourceConfig_m4f;
    }

    return timerClockSourceConfig;
}

function getDefaultTimerClockSourceMhz(clkSource) {
    let clkSourceHz = 0;
    let cpu = common.getSelfSysCfgCoreName();

    if(cpu.match(/hsm*/) || cpu.match(/m4f*/)) {
        clkSourceHz = system.getScript(`/imports/kernel/dpl/clock_${common.getSocName()}_hsm.syscfg.js`).defaultTimerClockSourceMhz.clkSourceHz;
        return clkSourceHz;
    }
    if(cpu.match(/r5f*/)) {
        switch(clkSource) {
             default:
             case "WUCPUCLK":
                 clkSourceHz = 25*1000000;
             break;
             case "XTALCLK":
                 clkSourceHz = 25*1000000;
             break;
             case "SYS_CLK":
                 clkSourceHz = 200*1000000;
             break;
             case "EXT_REFCLK":
                 clkSourceHz = 100*1000000;
             break;
             case "RCCLK10M":
                 clkSourceHz = 10*1000000;
             break;
             case "CTPS_GENF0":
                 clkSourceHz = 250*1000000;
             break;
             case "DPLL_CORE_HSDIV0_CLKOUT1":
                 clkSourceHz = 500*1000000;
             break;
             case "DPLL_PER_HSDIV0_CLKOUT1":
                 clkSourceHz = 192*1000000;
             break;
         }
    }
    return clkSourceHz;
}

exports = {
    getStaticConfigArr,
    getTimerClockSourceConfigArr,
    getTimerClockSourceValue,
    getDefaultTimerClockSourceMhz,
    makeInstanceConfig,
};