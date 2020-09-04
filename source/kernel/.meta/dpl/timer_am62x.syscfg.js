
let common = system.getScript("/common");

let timerClockSourceConfig_mcu = [
        {
            "name": "MCU_HFOSC0",
        },
        {
            "name": "MCU_SYSCLK0",
        },
        {
            "name": "CLK_12M_RC",
        },
        {
            "name": "MCU_PLL0_HSDIV3",
        },
        {
            "name": "MCU_EXT_REFCLK0",
        },
        {
            "name": "MCU_HFOSC0_CLKOUT_32K",
        },
        {
            "name": "CPSW0_CPTS_GENF0",
        },
        {
            "name": "CLK_32K_RC",
        },
];

function getTimerClockSourceValue(instance) {
    let cpu = common.getSelfSysCfgCoreName();
    let clkSelMuxValue = 0;

    if(cpu.match(/m4f*/)) {
        switch(instance.clkSource) {
            default:
            case "MCU_HFOSC0":
                clkSelMuxValue = 0x0;
                break;
            case "MCU_SYSCLK0":
                clkSelMuxValue = 0x1;
                break;
            case "CLK_12M_RC":
                clkSelMuxValue = 0x2;
                break;
            case "MCU_PLL0_HSDIV3":
                clkSelMuxValue = 0x3;
                break;
            case "MCU_EXT_REFCLK0":
                clkSelMuxValue = 0x4;
                break;
            case "MCU_HFOSC0_CLKOUT_32K":
                clkSelMuxValue = 0x5;
                break;
            case "CPSW0_CPTS_GENF0":
                clkSelMuxValue = 0x6;
                break;
            case "CLK_32K_RC":
                clkSelMuxValue = 0x7;
                break;
        }
    }
    else
    {
        switch(instance.clkSource) {
            default:
            case "MCU_HFOSC0":
                clkSelMuxValue = 0x0;
                break;
            case "MCU_HFOSC0_CLKOUT_32K":
                clkSelMuxValue = 0x1;
                break;
            case "MAIN_PLL0_HSDIV7_CLKOUT":
                clkSelMuxValue = 0x2;
                break;
            case "CLK_12M_RC":
                clkSelMuxValue = 0x3;
                break;
            case "MCU_EXT_REFCLK0":
                clkSelMuxValue = 0x4;
                break;
            case "EXT_REFCLK1":
                clkSelMuxValue = 0x5;
                break;
            case "CPTS0_RFT_CLK":
                clkSelMuxValue = 0x6;
                break;
            case "CPSW0_CPTS_RFT_CLK":
                clkSelMuxValue = 0x7;
                break;
            case "MAIN_PLL1_HSDIV3_CLKOUT":
                clkSelMuxValue = 0x8;
                break;
            case "MAIN_PLL2_HSDIV6_CLKOUT":
                clkSelMuxValue = 0x9;
                break;
            case "CPSW0_CPTS_GENF0":
                clkSelMuxValue = 0xA;
                break;
            case "CPSW0_CPTS_GENF1":
                clkSelMuxValue = 0xB;
                break;
            case "CPTS0_CPTS_GENF1_0":
                clkSelMuxValue = 0xC;
                break;
            case "CPTS0_CPTS_GENF2_0":
                clkSelMuxValue = 0xD;
                break;
            case "CPTS0_CPTS_GENF3_0":
                clkSelMuxValue = 0xE;
                break;
            case "CPTS0_CPTS_GENF4_0":
                clkSelMuxValue = 0xF;
                break;
        }
    }
    return clkSelMuxValue;
}

function getStaticConfigArr() {
    let cpu = common.getSelfSysCfgCoreName();
    let staticConfigArr;

    if(cpu.match(/m4f*/)) {
        let staticConfig_m4f = [];
        for(let i=0; i<4; i++)
        {
            staticConfig_m4f.push(
                {
                    timerName: `MCU_DMTIMER${i}`,
                    timerBaseAddr: 0x04800000 + i*0x10000,
                    timerHwiIntNum: 4 + i + 16,
                    timerInputPreScaler: 1,
                    clkSelMuxAddr: 0x04508060 + 4*i,
                    lockUnlockDomain: "SOC_DOMAIN_ID_MCU",
                    lockUnlockPartition: 0,
                }
            )
        }
        staticConfigArr = staticConfig_m4f;
    }

    return staticConfigArr;
}

function getInterfaceName(inst) {
    let cpu = common.getSelfSysCfgCoreName();
    let interfaceName;
    if(cpu.match(/m4f*/)) {
        interfaceName = "MCU_TIMER";
    }

    return interfaceName;
}

function getTimerClockSourceConfigArr() {
    let cpu = common.getSelfSysCfgCoreName();
    let timerClockSourceConfig;
    if(cpu.match(/m4f*/)) {
        timerClockSourceConfig = timerClockSourceConfig_mcu;
    }

    return timerClockSourceConfig;
}

function getTimerClockSourceHz(clkSource) {
    let cpu = common.getSelfSysCfgCoreName();
    let clkSourceHz = 0;

    if(cpu.match(/m4f*/)) {
        switch(clkSource) {
            default:
            case "MCU_HFOSC0":
                clkSourceHz = 25*1000000;
                break;
            case "MCU_SYSCLK0":
                clkSourceHz = 100*1000000;
                break;
            case "CLK_12M_RC":
                clkSourceHz = 12*1000000;
                break;
            case "MCU_PLL0_HSDIV3":
                clkSourceHz = 200*1000000;
                break;
            case "MCU_EXT_REFCLK0":
                clkSourceHz = 1*1000000;
                break;
            case "MCU_HFOSC0_CLKOUT_32K":
                clkSourceHz = 32*1000;
                break;
            case "CPSW0_CPTS_GENF0":
                clkSourceHz = 1*1000000;
                break;
            case "CLK_32K_RC":
                clkSourceHz = 32*1000;
                break;
        }
    }
    else
    {
        switch(clkSource) {
            default:
            case "MCU_HFOSC0":
                clkSourceHz = 25*1000000;
                break;
            case "MCU_HFOSC0_CLKOUT_32K":
                clkSourceHz = 32*1000;
                break;
            case "MAIN_PLL0_HSDIV7_CLKOUT":
                clkSourceHz = 200*1000000;
                break;
            case "CLK_12M_RC":
                clkSourceHz = 12*1000000;
                break;
            case "MCU_EXT_REFCLK0":
                clkSourceHz = 1*1000000;
                break;
            case "EXT_REFCLK1":
                clkSourceHz = 1*1000000;
                break;
            case "CPTS0_RFT_CLK":
                clkSourceHz = 1*1000000;
                break;
            case "CPSW0_CPTS_RFT_CLK":
                clkSourceHz = 1*1000000;
                break;
            case "MAIN_PLL1_HSDIV3_CLKOUT":
                clkSourceHz = 192*1000000;
                break;
            case "MAIN_PLL2_HSDIV6_CLKOUT":
                clkSourceHz = 225*1000000;
                break;
            case "CPSW0_CPTS_GENF0":
                clkSourceHz = 1*1000000;
                break;
            case "CPSW0_CPTS_GENF1":
                clkSourceHz = 1*1000000;
                break;
            case "CPTS0_CPTS_GENF1_0":
                clkSourceHz = 1*1000000;
                break;
            case "CPTS0_CPTS_GENF2_0":
                clkSourceHz = 1*1000000;
                break;
            case "CPTS0_CPTS_GENF3_0":
                clkSourceHz = 1*1000000;
                break;
            case "CPTS0_CPTS_GENF4_0":
                clkSourceHz = 1*1000000;
                break;
        }
    }

    return clkSourceHz;
}

function getBlockedTimers() {
    return [];
}

exports = {
    getStaticConfigArr,
    getTimerClockSourceConfigArr,
    getTimerClockSourceValue,
    getTimerClockSourceHz,
    getInterfaceName,
    getBlockedTimers,
};
