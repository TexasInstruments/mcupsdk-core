
let common = system.getScript("/common");

let ecap_func_clk = 125 * 1000 * 1000;

const staticConfig_r5fss = [
    {
        name: "ECAP0",
        baseAddr: "CSL_ECAP0_CTL_STS_BASE",
        intrNum: 140,
        funcClk: ecap_func_clk,
        clockIds: [ "TISCI_DEV_ECAP0" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_ECAP0",
                clkId   : "TISCI_DEV_ECAP0_VBUS_CLK",
                clkRate : ecap_func_clk,
            },
        ],
    },
    {
        name: "ECAP1",
        baseAddr: "CSL_ECAP1_CTL_STS_BASE",
        intrNum: 141,
        funcClk: ecap_func_clk,
        clockIds: [ "TISCI_DEV_ECAP1" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_ECAP1",
                clkId   : "TISCI_DEV_ECAP1_VBUS_CLK",
                clkRate : ecap_func_clk,
            },
        ],
    },
    {
        name: "ECAP2",
        baseAddr: "CSL_ECAP2_CTL_STS_BASE",
        intrNum: 142,
        funcClk: ecap_func_clk,
        clockIds: [ "TISCI_DEV_ECAP2" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_ECAP2",
                clkId   : "TISCI_DEV_ECAP2_VBUS_CLK",
                clkRate : ecap_func_clk,
            },
        ],
    },
];

const staticConfig_a53ss = [
    {
        name: "ECAP0",
        baseAddr: "CSL_ECAP0_CTL_STS_BASE",
        intrNum: 145,
        funcClk: ecap_func_clk,
        clockIds: [ "TISCI_DEV_ECAP0" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_ECAP0",
                clkId   : "TISCI_DEV_ECAP0_VBUS_CLK",
                clkRate : ecap_func_clk,
            },
        ],
    },
    {
        name: "ECAP1",
        baseAddr: "CSL_ECAP1_CTL_STS_BASE",
        intrNum: 146,
        funcClk: ecap_func_clk,
        clockIds: [ "TISCI_DEV_ECAP1" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_ECAP1",
                clkId   : "TISCI_DEV_ECAP1_VBUS_CLK",
                clkRate : ecap_func_clk,
            },
        ],
    },
    {
        name: "ECAP2",
        baseAddr: "CSL_ECAP2_CTL_STS_BASE",
        intrNum: 147,
        funcClk: ecap_func_clk,
        clockIds: [ "TISCI_DEV_ECAP2" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_ECAP2",
                clkId   : "TISCI_DEV_ECAP2_VBUS_CLK",
                clkRate : ecap_func_clk,
            },
        ],
    },
];

function getStaticConfigArr() {
    let staticConfig;

    if(common.getSelfSysCfgCoreName().match(/a53*/))
    {
        staticConfig = staticConfig_a53ss;
    }
    else
    {
        staticConfig = staticConfig_r5fss;
    }
    return staticConfig;
}

let soc = {
    getStaticConfigArr,
};

exports = soc;
