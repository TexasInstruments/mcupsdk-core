
let common = system.getScript("/common");

let wdt_func_clk = 32552;

const watchdog_config = [
    {
        name: "WDT0",
        wdtInstance: "WATCHDOG_INST_ID_0",
        baseAddr: "CSL_RTI0_CFG_BASE",
        intrNum         : 169,
        funcClk: wdt_func_clk,
        clockIds        : [ "TISCI_DEV_RTI0" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_RTI0",
                clkId   : "TISCI_DEV_RTI0_RTI_CLK",
                clkRate : wdt_func_clk,
            },
        ],
    },
    {
        name: "WDT1",
        wdtInstance: "WATCHDOG_INST_ID_1",
        baseAddr: "CSL_RTI1_CFG_BASE",
        intrNum         : 170,
        funcClk: wdt_func_clk,
        clockIds        : [ "TISCI_DEV_RTI1" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_RTI1",
                clkId   : "TISCI_DEV_RTI1_RTI_CLK",
                clkRate : wdt_func_clk,
            },
        ],
    },
    {
        name: "WDT8",
        wdtInstance: "WATCHDOG_INST_ID_8",
        baseAddr: "CSL_RTI8_CFG_BASE",
        intrNum         : 0,
        funcClk: wdt_func_clk,
        clockIds        : [ "TISCI_DEV_RTI8" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_RTI8",
                clkId   : "TISCI_DEV_RTI8_RTI_CLK",
                clkRate : wdt_func_clk,
            },
        ],
    },
    {
        name: "WDT9",
        wdtInstance: "WATCHDOG_INST_ID_9",
        baseAddr: "CSL_RTI9_CFG_BASE",
        intrNum         : 0,
        funcClk: wdt_func_clk,
        clockIds        : [ "TISCI_DEV_RTI9" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_RTI9",
                clkId   : "TISCI_DEV_RTI9_RTI_CLK",
                clkRate : wdt_func_clk,
            },
        ],
    },
    {
        name: "WDT10",
        wdtInstance: "WATCHDOG_INST_ID_10",
        baseAddr: "CSL_RTI10_CFG_BASE",
        intrNum         : 0,
        funcClk: wdt_func_clk,
        clockIds        : [ "TISCI_DEV_RTI10" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_RTI10",
                clkId   : "TISCI_DEV_RTI10_RTI_CLK",
                clkRate : wdt_func_clk,
            },
        ],
    },
    {
        name: "WDT11",
        wdtInstance: "WATCHDOG_INST_ID_11",
        baseAddr: "CSL_RTI11_CFG_BASE",
        intrNum         : 0,
        funcClk: wdt_func_clk,
        clockIds        : [ "TISCI_DEV_RTI11" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_RTI11",
                clkId   : "TISCI_DEV_RTI11_RTI_CLK",
                clkRate : wdt_func_clk,
            },
        ],
    },
];

function getConfigArr() {
    return watchdog_config;
}

exports = {
    getConfigArr,
};

