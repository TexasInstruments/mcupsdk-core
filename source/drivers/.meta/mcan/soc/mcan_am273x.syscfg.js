let common = system.getScript("/common");

let mcan_func_clk = 80 * 1000 * 1000;

const mcan_config_r5fss = [
    {
        name            : "MSS_MCANA",
        baseAddr        : "CSL_MSS_MCANA_MSG_RAM_U_BASE",
        intrNum         : 44,
        clockIds        : [ "SOC_RcmPeripheralId_MSS_MCANA" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_MSS_MCANA",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2",
                clkRate : mcan_func_clk,
            },
        ],          
    },
    {
        name            : "MSS_MCANB",
        baseAddr        : "CSL_MSS_MCANB_MSG_RAM_U_BASE",
        intrNum         : 48,
        clockIds        : [ "SOC_RcmPeripheralId_MSS_MCANB" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_MSS_MCANB",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2",
                clkRate : mcan_func_clk,
            },
        ],
    },

];

function getConfigArr() {
    let mcan_config;

    mcan_config = mcan_config_r5fss;

    return mcan_config;
}

function getInterfaceName(inst) {

    return "MSS_MCAN";
}

exports = {
    getConfigArr,
    getInterfaceName,
};
