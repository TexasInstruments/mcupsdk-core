
let common = system.getScript("/common");

let wdt_func_clk = 5000000;

const watchdog_config_r5fss = [
    {
        name: "MSS_WDT",
		wdtInstance: "0",
        baseAddr: "CSL_MSS_WDT_U_BASE",
        funcClk: wdt_func_clk,
        clockIds        : [ "SOC_RcmPeripheralId_MSS_WDT" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_MSS_WDT",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : wdt_func_clk,
            },
        ],
    },
];

const watchdog_config_c66ss = [
    {
        name: "DSS_WDT",
		wdtInstance: "0",
        baseAddr: "CSL_DSS_WDT_U_BASE",
        funcClk: wdt_func_clk,
        clockIds        : [ "SOC_RcmPeripheralId_DSS_WDT" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_DSS_WDT",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : wdt_func_clk,
            },
        ],
    },
];

function getConfigArr() {
    let watchdog_config;

    if(common.getSelfSysCfgCoreName().includes("c66")) {
        watchdog_config = watchdog_config_c66ss;
    }
    else {
        watchdog_config = watchdog_config_r5fss;
    }

    return watchdog_config;
}

exports = {
    getConfigArr,
};