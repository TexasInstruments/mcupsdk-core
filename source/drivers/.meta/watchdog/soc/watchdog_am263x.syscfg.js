
let common = system.getScript("/common");

let wdt_func_clk = 5000000;

const watchdog_config = [
    {
        name: "WDT0",
        wdtInstance: "WATCHDOG_INST_ID_0",
        baseAddr: "CSL_WDT0_U_BASE",
        funcClk: wdt_func_clk,
        clockIds        : [ "SOC_RcmPeripheralId_WDT0" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_WDT0",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : wdt_func_clk,
            },
        ],
    },
    {
        name: "WDT1",
        wdtInstance: "WATCHDOG_INST_ID_1",
        baseAddr: "CSL_WDT1_U_BASE",
        funcClk: wdt_func_clk,
        clockIds        : [ "SOC_RcmPeripheralId_WDT1" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_WDT1",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : wdt_func_clk,
            },
        ],
    },
    {
        name: "WDT2",
        wdtInstance: "WATCHDOG_INST_ID_2",
        baseAddr: "CSL_WDT2_U_BASE",
        funcClk: wdt_func_clk,
        clockIds        : [ "SOC_RcmPeripheralId_WDT2" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_WDT2",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : wdt_func_clk,
            },
        ],
    },
    {
        name: "WDT3",
        wdtInstance: "WATCHDOG_INST_ID_3",
        baseAddr: "CSL_WDT3_U_BASE",
        funcClk: wdt_func_clk,
        clockIds        : [ "SOC_RcmPeripheralId_WDT3" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_WDT3",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : wdt_func_clk,
            },
        ],
    },
];

function getConfigArr() {
    let wdtInst = [];

    if(system.context == "r5fss0-0")
        wdtInst.push(watchdog_config[0]);
    if(system.context == "r5fss0-1")
        wdtInst.push(watchdog_config[1]);
    if(system.context == "r5fss1-0")
        wdtInst.push(watchdog_config[2]);
    if(system.context == "r5fss1-1")
        wdtInst.push(watchdog_config[3]);

    return wdtInst;
}

exports = {
    getConfigArr,
};

