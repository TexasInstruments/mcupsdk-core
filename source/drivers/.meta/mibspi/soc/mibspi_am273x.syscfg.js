let common = system.getScript("/common");


let mibspi_input_clk_freq = 200000000;

const mibspi_config_r5fss = [
    {
        name                : "MSS_MIBSPIA",
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
        name                : "MSS_MIBSPIB",
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
    {
        name                : "RCSS_MIBSPIA",
        regBaseAddr         : "CSL_RCSS_SPIA_U_BASE",
        ramBaseAddr         : "CSL_RCSS_SPIA_RAM_U_BASE",
        inputClkFreq        : mibspi_input_clk_freq,
        intr0Num            : 147,
        intr1Num            : 148,
        edmaInst            : "EDMA_RCSS_A",
        edmaEvtRx           : [0, 2, 4],
        edmaEvtTx           : [1, 3, 5],
        instanceID          : [ "MIBSPI_INST_ID_RCSS_SPIA" ],
        clockIds        : [ "SOC_RcmPeripheralId_RCSS_SPIA" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_RCSS_SPIA",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : mibspi_input_clk_freq,
            },
        ],
    },
    {
        name                : "RCSS_MIBSPIB",
        regBaseAddr         : "CSL_RCSS_SPIB_U_BASE",
        ramBaseAddr         : "CSL_RCSS_SPIB_RAM_U_BASE",
        inputClkFreq        : mibspi_input_clk_freq,
        intr0Num            : 149,
        intr1Num            : 150,
        edmaInst            : "EDMA_RCSS_A",
        edmaEvtRx           : [6, 8, 10],
        edmaEvtTx           : [7, 9, 11],
        instanceID          : [ "MIBSPI_INST_ID_RCSS_SPIB" ],
        clockIds        : [ "SOC_RcmPeripheralId_RCSS_SPIB" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_RCSS_SPIB",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : mibspi_input_clk_freq,
            },
        ],
    },
];

const mibspi_config_c66ss = [
   {
        name                : "RCSS_MIBSPIA",
        regBaseAddr         : "CSL_RCSS_SPIA_U_BASE",
        ramBaseAddr         : "CSL_RCSS_SPIA_RAM_U_BASE",
        inputClkFreq        : mibspi_input_clk_freq,
        intr0Num            : 45,
        intr1Num            : 46,
        edmaInst            : "EDMA_RCSS_A",
        edmaEvtRx           : [0, 2, 4],
        edmaEvtTx           : [1, 3, 5],
        instanceID          : [ "MIBSPI_INST_ID_RCSS_SPIA" ],
        clockIds        : [ "SOC_RcmPeripheralId_RCSS_SPIA" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_RCSS_SPIA",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : mibspi_input_clk_freq,
            },
        ],
    },
    {
        name                : "RCSS_MIBSPIB",
        regBaseAddr         : "CSL_RCSS_SPIB_U_BASE",
        ramBaseAddr         : "CSL_RCSS_SPIB_RAM_U_BASE",
        inputClkFreq        : mibspi_input_clk_freq,
        intr0Num            : 47,
        intr1Num            : 48,
        edmaInst            : "EDMA_RCSS_A",
        edmaEvtRx           : [6, 8, 10],
        edmaEvtTx           : [7, 9, 11],
        instanceID          : [ "MIBSPI_INST_ID_RCSS_SPIB" ],
        clockIds        : [ "SOC_RcmPeripheralId_RCSS_SPIB" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_RCSS_SPIB",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : mibspi_input_clk_freq,
            },
        ],
    },

];

function getConfigArr() {
    let mibspi_config;

    if(common.getSelfSysCfgCoreName().includes("c66")) {
        mibspi_config = mibspi_config_c66ss;
    }
    else {
        mibspi_config = mibspi_config_r5fss;
    }

    return mibspi_config;
}

function getInterfaceName(inst) {
    let interfaceName;

    if((inst.instance == "MSS_MIBSPIA") || (inst.instance == "MSS_MIBSPIB"))
    {
        interfaceName = "MSS_MIBSPI";
    }

    if((inst.instance == "RCSS_MIBSPIA") || (inst.instance == "RCSS_MIBSPIB"))
    {
        interfaceName = "RCSS_MIBSPI";
    }
    return interfaceName;
}

function getDefaultInterfaceName() {
    return "MSS_MIBSPI";
}

exports = {
    getConfigArr,
    getInterfaceName,
    getDefaultInterfaceName,
};