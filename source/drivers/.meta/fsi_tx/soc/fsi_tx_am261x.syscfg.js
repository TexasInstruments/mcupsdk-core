
let common = system.getScript("/common");

let fsi_tx_func_clk = 400 * 1000 * 1000;

const staticConfig = [
    {
        name: "FSITX0",
        baseAddr: "CSL_CONTROLSS_FSI_TX0_U_BASE",
        funcClk: fsi_tx_func_clk,
        clockIds        : [ "SOC_RcmPeripheralId_CONTROLSS_PLL" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_CONTROLSS_PLL",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2",
                clkRate : fsi_tx_func_clk,
            },
        ],
    },
    {
        name: "FSITX1",
        baseAddr: "CSL_CONTROLSS_FSI_TX1_U_BASE",
        funcClk: fsi_tx_func_clk,
        clockIds        : [ "SOC_RcmPeripheralId_CONTROLSS_PLL" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_CONTROLSS_PLL",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2",
                clkRate : fsi_tx_func_clk,
            },
        ],
    },
    {
        name: "FSITX2",
        baseAddr: "CSL_CONTROLSS_FSI_TX2_U_BASE",
        funcClk: fsi_tx_func_clk,
        clockIds        : [ "SOC_RcmPeripheralId_CONTROLSS_PLL" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_CONTROLSS_PLL",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2",
                clkRate : fsi_tx_func_clk,
            },
        ],
    },
    {
        name: "FSITX3",
        baseAddr: "CSL_CONTROLSS_FSI_TX3_U_BASE",
        funcClk: fsi_tx_func_clk,
        clockIds        : [ "SOC_RcmPeripheralId_CONTROLSS_PLL" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_CONTROLSS_PLL",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2",
                clkRate : fsi_tx_func_clk,
            },
        ],
    },
];

function getStaticConfigArr() {
    return staticConfig;
}

function getInterfaceName(inst) {
    return "FSITX";
}

let soc = {
    getStaticConfigArr,
    getInterfaceName,
    interruptXbarConfig: true,
};

exports = soc;
