let common = system.getScript("/common");


let mcasp_input_clk_freq = 48000000;

const mcasp_config = [
    {
        name                : "RCSS_MCASPA",
        regBaseAddr         : "CSL_RCSS_MCASP_A_U_BASE",
        dataRegBaseAddr     : "CSL_RCSS_MCASPA_DATA_U_BASE",
        numSerializers      : 16,
        inputClkFreq        : mcasp_input_clk_freq,
        intr0Num            : 52,
        intr1Num            : 55,
        edmaChTx            : 48,
        edmaChRx            : 51,
        clockIds        : [ "SOC_RcmPeripheralId_RCSS_MCASPA_AUX" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_RCSS_MCASPA_AUX",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1",
                clkRate : mcasp_input_clk_freq,
            },
        ],
    },
    {
        name                : "RCSS_MCASPB",
        regBaseAddr         : "CSL_RCSS_MCASP_B_U_BASE",
        dataRegBaseAddr     : "CSL_RCSS_MCASPB_DATA_U_BASE",
        numSerializers      : 6,
        inputClkFreq        : mcasp_input_clk_freq,
        intr0Num            : 53,
        intr1Num            : 56,
        edmaChTx            : 49,
        edmaChRx            : 52,
        clockIds        : [ "SOC_RcmPeripheralId_RCSS_MCASPB_AUX" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_RCSS_MCASPB_AUX",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1",
                clkRate : mcasp_input_clk_freq,
            },
        ],
    },
    {
        name                : "RCSS_MCASPC",
        regBaseAddr         : "CSL_RCSS_MCASP_C_U_BASE",
        dataRegBaseAddr     : "CSL_RCSS_MCASPC_DATA_U_BASE",
        numSerializers      : 6,
        inputClkFreq        : mcasp_input_clk_freq,
        intr0Num            : 54,
        intr1Num            : 57,
        edmaChTx            : 50,
        edmaChRx            : 53,
        clockIds        : [ "SOC_RcmPeripheralId_RCSS_MCASPC_AUX" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_RCSS_MCASPC_AUX",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1",
                clkRate : mcasp_input_clk_freq,
            },
        ],
    },
];

function getConfigArr() {
    return mcasp_config;
}

exports = {
    getConfigArr,
    mcasp_input_clk_freq,
};