let common = system.getScript("/common");

let uart_input_clk_freq = 150000000;

const uart_config_r5fss = [
    {
        name            : "UARTA",
        baseAddr        : "CSL_MSS_SCIA_U_BASE",
        inputClkFreq    : uart_input_clk_freq,
        intrNum         : 53,
        rxDmaEvt        : 57,
        txDmaEvt        : 58,
        clockIds        : [ "SOC_RcmPeripheralId_MSS_SCIA" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_MSS_SCIA",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : uart_input_clk_freq,
            },
        ],
    },
    {
        name            : "UARTB",
        baseAddr        : "CSL_MSS_SCIB_U_BASE",
        inputClkFreq    : uart_input_clk_freq,
        intrNum         : 55,
        rxDmaEvt        : 59,
        txDmaEvt        : 60,
        clockIds        : [ "SOC_RcmPeripheralId_MSS_SCIB" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_MSS_SCIB",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : uart_input_clk_freq,
            },
        ],
    }
];

const uart_config_c66ss = [
    {
        name            : "DSS_UARTA",
        baseAddr        : "CSL_DSS_SCIA_U_BASE",
        inputClkFreq    : uart_input_clk_freq,
        intrNum         : 37,
        rxDmaEvt        : 14,
        txDmaEvt        : 15,
        clockIds        : [ "SOC_RcmPeripheralId_DSS_SCIA" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_DSS_SCIA",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : uart_input_clk_freq,
            },
        ],
    },
];

function getConfigArr() {
    let uart_config;

    if(common.getSelfSysCfgCoreName().includes("c66")) {
        uart_config = uart_config_c66ss;
    }
    else {
        uart_config = uart_config_r5fss;
    }

    return uart_config;
}

function getInterfaceName(inst) {
    if(common.getSelfSysCfgCoreName().includes("c66")) {
        return "DSS_UART";
    }
    return "UART";
}

exports = {
    getConfigArr,
    getInterfaceName,
};
