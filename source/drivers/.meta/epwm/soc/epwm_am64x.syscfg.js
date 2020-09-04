
let common = system.getScript("/common");

let epwm_func_clk = 250 * 1000 * 1000;

const staticConfig = [
    {
        name: "EHRPWM0",
        baseAddr: "CSL_EPWM0_EPWM_BASE",
        intrNum: 108,
        tripIntrNum: 109,
        isSyncoPresent: true,
        isSynciPresent: true,
        funcClk: epwm_func_clk,
        clockIds: [ "TISCI_DEV_EPWM0_VBUSP_CLK" ],
    },
    {
        name: "EHRPWM1",
        baseAddr: "CSL_EPWM1_EPWM_BASE",
        intrNum: 110,
        tripIntrNum: 111,
        isSyncoPresent: false,
        isSynciPresent: false,
        funcClk: epwm_func_clk,
        clockIds: [ "TISCI_DEV_EPWM1_VBUSP_CLK" ],
    },
    {
        name: "EHRPWM2",
        baseAddr: "CSL_EPWM2_EPWM_BASE",
        intrNum: 112,
        tripIntrNum: 113,
        isSyncoPresent: false,
        isSynciPresent: false,
        funcClk: epwm_func_clk,
        clockIds: [ "TISCI_DEV_EPWM2_VBUSP_CLK" ],
    },
    {
        name: "EHRPWM3",
        baseAddr: "CSL_EPWM3_EPWM_BASE",
        intrNum: 114,
        tripIntrNum: 115,
        isSyncoPresent: true,
        isSynciPresent: true,
        funcClk: epwm_func_clk,
        clockIds: [ "TISCI_DEV_EPWM3_VBUSP_CLK" ],
    },
    {
        name: "EHRPWM4",
        baseAddr: "CSL_EPWM4_EPWM_BASE",
        intrNum: 116,
        tripIntrNum: 117,
        isSyncoPresent: false,
        isSynciPresent: false,
        funcClk: epwm_func_clk,
        clockIds: [ "TISCI_DEV_EPWM4_VBUSP_CLK" ],
    },
    {
        name: "EHRPWM5",
        baseAddr: "CSL_EPWM5_EPWM_BASE",
        intrNum: 118,
        tripIntrNum: 139,
        isSyncoPresent: false,
        isSynciPresent: false,
        funcClk: epwm_func_clk,
        clockIds: [ "TISCI_DEV_EPWM5_VBUSP_CLK" ],
    },
    {
        name: "EHRPWM6",
        baseAddr: "CSL_EPWM6_EPWM_BASE",
        intrNum: 146,
        tripIntrNum: 147,
        isSyncoPresent: true,
        isSynciPresent: true,
        funcClk: epwm_func_clk,
        clockIds: [ "TISCI_DEV_EPWM6_VBUSP_CLK" ],
    },
    {
        name: "EHRPWM7",
        baseAddr: "CSL_EPWM7_EPWM_BASE",
        intrNum: 148,
        tripIntrNum: 149,
        isSyncoPresent: false,
        isSynciPresent: false,
        funcClk: epwm_func_clk,
        clockIds: [ "TISCI_DEV_EPWM7_VBUSP_CLK" ],
    },
    {
        name: "EHRPWM8",
        baseAddr: "CSL_EPWM8_EPWM_BASE",
        intrNum: 150,
        tripIntrNum: 178,
        isSyncoPresent: false,
        isSynciPresent: false,
        funcClk: epwm_func_clk,
        clockIds: [ "TISCI_DEV_EPWM8_VBUSP_CLK" ],
    },
];

function getStaticConfigArr() {
    return staticConfig;
}

let soc = {
    getStaticConfigArr,
};

exports = soc;
