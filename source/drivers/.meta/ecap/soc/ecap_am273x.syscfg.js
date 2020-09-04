
let common = system.getScript("/common");

let ecap_func_clk = 200 * 1000 * 1000;

const staticConfig_r5fss = [
    {
        name: "MSS_ECAP",
        baseAddr: "CSL_RCSS_ECAP_U_BASE",
        intrNum: "CSL_MSS_INTR_RCSS_ECAP_INT",
        funcClk: ecap_func_clk,
        clockIds: [ "SOC_RcmPeripheralId_MSS_RTIA" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_MSS_RTIA",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : ecap_func_clk,
            },
        ],
    },
];

const staticConfig_c66ss = [
    {
        name: "RCSS_ECAP",
        baseAddr: "CSL_RCSS_ECAP_U_BASE",
        intrNum: "CSL_DSS_INTR_RCSS_ECAP_INT",
        funcClk: ecap_func_clk,
        clockIds: [ "SOC_RcmPeripheralId_DSS_RTIA" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_DSS_RTIA",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : ecap_func_clk,
            },
        ],
    
    },
];

function getStaticConfigArr() {
    let staticConfig;
    if(common.getSelfSysCfgCoreName().includes("c66")) {
        staticConfig = staticConfig_c66ss;
    }
    else {
        staticConfig = staticConfig_r5fss;
    }
    return staticConfig;
}

exports = {
    getStaticConfigArr,
};
