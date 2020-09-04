let qspi_input_clk_freq = 80000000;

const qspi_config_r5fss = [
    {
        name            : "QSPI0",
        baseAddr        : "CSL_MSS_QSPI_U_BASE",
        memMapBaseAddr  : "CSL_EXT_FLASH_U_BASE",
        inputClkFreq    : qspi_input_clk_freq,
        intrNum         : 35,
        baudRateDiv     : 0,
        wrdLen          : 8,
        clockIds        : [ "SOC_RcmPeripheralId_MSS_QSPI" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_MSS_QSPI",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_DSP_HSDIV0_CLKOUT2",
                clkRate : qspi_input_clk_freq,
            },
        ],
    },
];

function getQspiPinName(inst, qspiPinName) {
    let pinName;
    switch (qspiPinName)
    {
        case "D0":
            pinName = "0";
            break;
        case "D1":
            pinName = "1";
            break;
        case "D2":
            pinName = "2";
            break;
        case "D3":
            pinName = "3";
            break;
        case "CLK":
            pinName = "CLK";
            break;
        case "CS0":
        case "CS1":
        case "CS2":
        case "CS3":
            pinName = "CS";
            break;
    }
    return pinName;
}

function getInterfaceName(inst) {
    let interfaceName = "QSPI";
    return interfaceName;
}

function getDefaultConfig()
{
    return qspi_config_r5fss[0];
}

function getConfigArr() {
    return qspi_config_r5fss;
}

exports = {
    getDefaultConfig,
    getConfigArr,
    getInterfaceName,
    getQspiPinName,
};
