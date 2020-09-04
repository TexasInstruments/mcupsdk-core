let common = system.getScript("/common");

let mcan_func_clk = 80 * 1000 * 1000;

const mcan_config_r5fss = [
    {
        name            : "MCAN0",
        baseAddr        : "CSL_MCAN0_MSG_RAM_U_BASE",
        intrNum         : 27,
        clockIds        : [ "SOC_RcmPeripheralId_MCAN0" ],
        clockFrequencies: [ 
            { 
                moduleId: "SOC_RcmPeripheralId_MCAN0",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0",
                clkRate : mcan_func_clk,
            },
        ],            
    },
    {
        name            : "MCAN1",
        baseAddr        : "CSL_MCAN1_MSG_RAM_U_BASE",
        intrNum         : 30,
        clockIds        : [ "SOC_RcmPeripheralId_MCAN1" ],
        clockFrequencies: [ 
            { 
                moduleId: "SOC_RcmPeripheralId_MCAN1",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0",
                clkRate : mcan_func_clk,
            },
        ],
    },
        {
        name            : "MCAN2",
        baseAddr        : "CSL_MCAN2_MSG_RAM_U_BASE",
        intrNum         : 33,
        clockIds        : [ "SOC_RcmPeripheralId_MCAN2" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_MCAN2",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0",
                clkRate : mcan_func_clk,
            },
        ],
    },
    {
        name            : "MCAN3",
        baseAddr        : "CSL_MCAN3_MSG_RAM_U_BASE",
        intrNum         : 36,
        clockIds        : [ "SOC_RcmPeripheralId_MCAN3" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_MCAN3",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0",
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

function getInterfaceName(instance) {
    return "MCAN";
}

exports = {
    getConfigArr,
    getInterfaceName,
};
