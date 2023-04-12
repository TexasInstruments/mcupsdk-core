
let common = system.getScript("/common");

let wdt_func_clk = 20000000;

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

const SOC_RcmClkSrcInfo = [
    {
        name: "SOC_RcmPeripheralClockSource_XTAL_CLK",
        displayName: "XTALCLK  (40 MHz)",
        freq: 40000000
    },
    {
        name: "SOC_RcmPeripheralClockSource_SYS_CLK",
        freq: 200000000,
        displayName: "SYS_CLK (200 MHz)"
    },
    {
        name: "SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1",
        freq: 192000000,
        displayName: "DPLL_CORE_HSDIV0_CLKOUT1 (192 MHz)",
    },
]

exports = {
    getConfigArr,
    SOC_RcmClkSrcInfo
};