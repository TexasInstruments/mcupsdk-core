let common = system.getScript("/common");


let mibspi_input_clk_freq = 150000000;

const mibspi_config_r5fss = [
    {
        name                : "MIBSPIA",
        regBaseAddr         : "CSL_MSS_SPIA_U_BASE",
        ramBaseAddr         : "CSL_MSS_SPIA_RAM_U_BASE",
        inputClkFreq        : mibspi_input_clk_freq,
        intr0Num            : 31,
        intr1Num            : 32,
        edmaInst            : "EDMA_MSS_A",
        edmaEvtRx           : [0, 2, 4],
        edmaEvtTx           : [1, 3, 5],
        instanceID          : [ "MIBSPI_INST_ID_MSS_SPIA" ],
        clockIds        : [ "SOC_RcmPeripheralId_MSS_SPIA" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_MSS_SPIA",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : mibspi_input_clk_freq,
            },
        ],
    },
    {
        name                : "MIBSPIB",
        regBaseAddr         : "CSL_MSS_SPIB_U_BASE",
        ramBaseAddr         : "CSL_MSS_SPIB_RAM_U_BASE",
        inputClkFreq        : mibspi_input_clk_freq,
        intr0Num            : 33,
        intr1Num            : 34,
        edmaInst            : "EDMA_MSS_A",
        edmaEvtRx           : [6, 8, 10],
        edmaEvtTx           : [7, 9, 11],
        instanceID          : [ "MIBSPI_INST_ID_MSS_SPIB" ],
        clockIds        : [ "SOC_RcmPeripheralId_MSS_SPIB" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_MSS_SPIB",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : mibspi_input_clk_freq,
            },
        ],
    },
];

function getConfigArr() {
    let mibspi_config;

    mibspi_config = mibspi_config_r5fss;

    return mibspi_config;
}

function getInterfaceName(inst) {
    return "MIBSPI";
}

function getDefaultInterfaceName() {
    return "MIBSPI";
}

exports = {
    getConfigArr,
    getInterfaceName,
    getDefaultInterfaceName,
};
