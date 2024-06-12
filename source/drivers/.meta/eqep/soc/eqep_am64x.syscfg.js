
let common = system.getScript("/common");

let eqep_func_clk = 125 * 1000 * 1000;

const eqepConfig_r5fss = [
    {
        name: "EQEP0",
        baseAddr: "CSL_EQEP0_REG_BASE",
        intrNum: 143,
        funcClk: eqep_func_clk,
        clockIds: [ "TISCI_DEV_EQEP0" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_EQEP0",
                clkId   : "TISCI_DEV_EQEP0_VBUS_CLK",
                clkRate : eqep_func_clk,
            },
        ],
    },
    {
        name: "EQEP1",
        baseAddr: "CSL_EQEP1_REG_BASE",
        intrNum: 144,
        funcClk: eqep_func_clk,
        clockIds: [ "TISCI_DEV_EQEP1" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_EQEP1",
                clkId   : "TISCI_DEV_EQEP1_VBUS_CLK",
                clkRate : eqep_func_clk,
            },
        ],
    },
    {
        name: "EQEP2",
        baseAddr: "CSL_EQEP2_REG_BASE",
        intrNum: 145,
        funcClk: eqep_func_clk,
        clockIds: [ "TISCI_DEV_EQEP2" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_EQEP2",
                clkId   : "TISCI_DEV_EQEP2_VBUS_CLK",
                clkRate : eqep_func_clk,
            },
        ],
    },
];

const eqepConfig_a53ss = [
    {
        name: "EQEP0",
        baseAddr: "CSL_EQEP0_REG_BASE",
        intrNum: 148,
        funcClk: eqep_func_clk,
        clockIds: [ "TISCI_DEV_EQEP0" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_EQEP0",
                clkId   : "TISCI_DEV_EQEP0_VBUS_CLK",
                clkRate : eqep_func_clk,
            },
        ],
    },
    {
        name: "EQEP1",
        baseAddr: "CSL_EQEP1_REG_BASE",
        intrNum: 149,
        funcClk: eqep_func_clk,
        clockIds: [ "TISCI_DEV_EQEP1" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_EQEP1",
                clkId   : "TISCI_DEV_EQEP1_VBUS_CLK",
                clkRate : eqep_func_clk,
            },
        ],
    },
    {
        name: "EQEP2",
        baseAddr: "CSL_EQEP2_REG_BASE",
        intrNum: 150,
        funcClk: eqep_func_clk,
        clockIds: [ "TISCI_DEV_EQEP2" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_EQEP2",
                clkId   : "TISCI_DEV_EQEP2_VBUS_CLK",
                clkRate : eqep_func_clk,
            },
        ],
    },
];

function getStaticConfigArr() {
    let staticConfig;

    if(common.getSelfSysCfgCoreName().match(/a53*/))
    {
        staticConfig = eqepConfig_a53ss;
    }
    else
    {
        staticConfig = eqepConfig_r5fss;
    }
    return staticConfig;
}

let soc = {
    getStaticConfigArr,
};

exports = soc;
