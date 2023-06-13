
let common = system.getScript("/common");

let timerClockSourceConfig_main = [
        {
            "name": "WKUP_HFOSC0",
        },
		{
            "name": "HFOSC1_CLKOUT",
        },
		{
            "name": "MAINHSDIV_CLKOUT3",
        },
		{
            "name": "CLK_12M_RC",
        },
		{
            "name": "CPSWHSDIV_CLKOUT2",
        },
		{
            "name": "MCU_EXT_REFCLK0",
        },
		{
            "name": "EXT_REFCLK1",
        },
		{
            "name": "WKUP_LFOSC0_CLKOUT",
        },
		{
            "name": "CPTS_RFT_CLK",
        },
		{
            "name": "PER1HSDIV_CLKOUT2",
        },
		{
            "name": "PER0_PLL_CLKOUT_DIV5",
        },
		{
            "name": "CLK_32K_RC",
        },
		{
            "name": "CPTS_GENF2",
        },
		{
            "name": "CPTS_GENF3",
        },
		{
            "name": "CPTS_GENF4",
        },
		{
            "name": "CPTS_GENF5",
        },
];

let timerClockSourceConfig_mcu = [
        {
            "name": "WKUP_HFOSC0",
        },
		{
            "name": "MCU_SYSCLK0",
        },
		{
            "name": "CLK_12M_RC",
        },
		{
            "name": "MCU_CPSW_PLL_CLKOUT",
        },
		{
            "name": "MCU_EXT_REFCLK0",
        },
		{
            "name": "LFOSC_CLKOUT",
        },
		{
            "name": "CPSW_GENF0",
        },
		{
            "name": "CLK_32K_RC",
        },
];

function getTimerClockSourceValue(instance) {
    let cpu = common.getSelfSysCfgCoreName();
    let clkSelMuxValue = 0;

    if(cpu.match(/r5f*/)) {
        switch(instance.clkSource) {
            default:
            case "WKUP_HFOSC0":
                clkSelMuxValue = 0x0;
                break;
            case "MCU_SYSCLK0":
                clkSelMuxValue = 0x1;
                break;
            case "CLK_12M_RC":
                clkSelMuxValue = 0x2;
                break;
            case "MCU_CPSW_PLL_CLKOUT":
                clkSelMuxValue = 0x3;
                break;
            case "MCU_EXT_REFCLK0":
                clkSelMuxValue = 0x4;
                break;
            case "LFOSC_CLKOUT":
                clkSelMuxValue = 0x5;
                break;
            case "CPSW_GENF0":
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
            case "WKUP_HFOSC0":
                clkSelMuxValue = 0x0;
                break;
            case "HFOSC1_CLKOUT":
                clkSelMuxValue = 0x1;
                break;
            case "MAINHSDIV_CLKOUT3":
                clkSelMuxValue = 0x2;
                break;
            case "CLK_12M_RC":
                clkSelMuxValue = 0x3;
                break;
            case "CPSWHSDIV_CLKOUT2":
                clkSelMuxValue = 0x4;
                break;
            case "MCU_EXT_REFCLK0":
                clkSelMuxValue = 0x5;
                break;
            case "EXT_REFCLK1":
                clkSelMuxValue = 0x6;
                break;
            case "WKUP_LFOSC0_CLKOUT":
                clkSelMuxValue = 0x7;
                break;
            case "CPTS_RFT_CLK":
                clkSelMuxValue = 0x8;
                break;
            case "PER1HSDIV_CLKOUT2":
                clkSelMuxValue = 0x9;
                break;
            case "PER0_PLL_CLKOUT_DIV5":
                clkSelMuxValue = 0xA;
                break;
            case "CLK_32K_RC":
                clkSelMuxValue = 0xB;
                break;
            case "CPTS_GENF2":
                clkSelMuxValue = 0xC;
                break;
            case "CPTS_GENF3":
                clkSelMuxValue = 0xD;
                break;
            case "CPTS_GENF4":
                clkSelMuxValue = 0xE;
                break;
            case "CPTS_GENF5":
                clkSelMuxValue = 0xF;
                break;
        }
    }

    return clkSelMuxValue;
}

function getStaticConfigArr() {
    let cpu = common.getSelfSysCfgCoreName();
    let staticConfigArr;

    if(cpu.match(/r5f*/)) {
        let staticConfig_r5f = [];

        staticConfig_r5f.push(
            {
                timerName: `MCU_DMTIMER`,
                timerBaseAddr: 0x40400000,
                timerHwiIntNum: 38,
                timerInputPreScaler: 1,
                clkSelMuxAddr: 0x40F08100,
                lockUnlockDomain: "SOC_DOMAIN_ID_MCU",
                lockUnlockPartition: 2,
            }
        )

        staticConfigArr = staticConfig_r5f;
    }

    return staticConfigArr;
}

function getInterfaceName(inst) {
    let cpu = common.getSelfSysCfgCoreName();
	if(cpu.match(/r5f*/)) {
        return "MCU_TIMER";
    }

    return "TIMER";
}

function getTimerClockSourceConfigArr() {
    let cpu = common.getSelfSysCfgCoreName();
    if(cpu.match(/r5f*/)) {
        return timerClockSourceConfig_mcu;
    }

    return timerClockSourceConfig_main;
}

function getTimerClockSourceHz(clkSource) {
    let cpu = common.getSelfSysCfgCoreName();
    let clkSourceHz = 0;

    if(cpu.match(/r5f*/)) {
        switch(clkSource) {
            default:
            case "WKUP_HFOSC0":
                clkSourceHz = 25*1000000;
                break;
            case "MCU_SYSCLK0":
                clkSourceHz = 200*1000000;
                break;
            case "CLK_12M_RC":
                clkSourceHz = 12500000;
                break;
            case "MCU_CPSW_PLL_CLKOUT":
                clkSourceHz = 250*1000000;
                break;
            case "MCU_EXT_REFCLK0":
                clkSourceHz = 100*1000000;
                break;
            case "LFOSC_CLKOUT":
                clkSourceHz = 32768;
                break;
            case "CPSW_GENF0":
                clkSourceHz = 250*1000000;
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
            case "WKUP_HFOSC0":
                clkSourceHz = 25*1000000;
                break;
            case "HFOSC1_CLKOUT":
                clkSourceHz = 27*1000000;
                break;
            case "MAINHSDIV_CLKOUT3":
                clkSourceHz = 250*1000000;
                break;
            case "CLK_12M_RC":
                clkSourceHz = 12500000;
                break;
            case "CPSWHSDIV_CLKOUT2":
                clkSourceHz = 250*1000000;
                break;
            case "MCU_EXT_REFCLK0":
                clkSourceHz = 100*1000000;
                break;
            case "EXT_REFCLK1":
                clkSourceHz = 100*1000000;
                break;
            case "WKUP_LFOSC0_CLKOUT":
                clkSourceHz = 32760;
                break;
            case "CPTS_RFT_CLK":
                clkSourceHz = 200*1000000;
                break;
            case "PER1HSDIV_CLKOUT2":
                clkSourceHz = 100*1000000;
                break;
            case "PER0_PLL_CLKOUT_DIV5":
                clkSourceHz = 192*1000000;
                break;
            case "CLK_32K_RC":
                clkSourceHz = 32*1000000;
                break;
            case "CPTS_GENF2":
                clkSourceHz = 250*1000000;
                break;
            case "CPTS_GENF3":
                clkSourceHz = 250*1000000;
                break;
            case "CPTS_GENF4":
                clkSourceHz = 250*1000000;
                break;
            case "CPTS_GENF5":
                clkSourceHz = 250*1000000;
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
    dmaTriggerSupport: false,
};
