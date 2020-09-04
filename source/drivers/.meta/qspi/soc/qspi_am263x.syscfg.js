let qspi_input_clk_freq = 80000000;

const qspi_config_r5fss = [
    {
        name            : "QSPI",
        baseAddr        : "CSL_QSPI0_U_BASE",
        memMapBaseAddr  : "CSL_EXT_FLASH0_U_BASE",
        inputClkFreq    : qspi_input_clk_freq,
        intrNum         : 54,
        baudRateDiv     : 0,
        wrdLen          : 8,
        clockIds        : [ "SOC_RcmPeripheralId_QSPI0" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_QSPI0",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0",
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
            pinName = "QSPI_D0";
            break;
        case "D1":
            pinName = "QSPI_D1";
            break;
        case "D2":
            pinName = "QSPI_D2";
            break;
        case "D3":
            pinName = "QSPI_D3";
            break;
        case "CLK":
            pinName = "QSPI_CLK";
            break;
        case "CS0":
            pinName = "QSPI_CSn0";
            break;
        case "CS1":
            pinName = "QSPI_CSn1";
            break;
        case "CS2":
            pinName = "QSPI_CSn0";
            break;
        case "CS3":
            pinName = "QSPI_CSn1";
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
