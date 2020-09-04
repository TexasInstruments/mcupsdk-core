
let common = system.getScript("/common");

let fsi_tx_func_clk = 500 * 1000 * 1000;

const staticConfig = [
    {
        name: "FSI_TX0",
        baseAddr: "CSL_FSITX0_CFG_BASE",
        intrNum1: 28,
        intrNum2: 29,
        funcClk: fsi_tx_func_clk,
        clockIds: [ "TISCI_DEV_FSITX0" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_FSITX0",
                clkId   : "TISCI_DEV_FSITX0_FSI_TX_PLL_CLK",
                clkRate : fsi_tx_func_clk,
            },
        ],
    },
    {
        name: "FSI_TX1",
        baseAddr: "CSL_FSITX1_CFG_BASE",
        intrNum1: 30,
        intrNum2: 31,
        funcClk: fsi_tx_func_clk,
        clockIds: [ "TISCI_DEV_FSITX1" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_FSITX1",
                clkId   : "TISCI_DEV_FSITX1_FSI_TX_PLL_CLK",
                clkRate : fsi_tx_func_clk,
            },
        ],
    },
];

function getStaticConfigArr() {
    return staticConfig;
}

function getInterfaceName(inst) {
    return "FSI_TX";
}

let soc = {
    getStaticConfigArr,
    getInterfaceName,
    interruptXbarConfig: false,
};
exports = soc;
