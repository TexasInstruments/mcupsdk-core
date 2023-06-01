let common = system.getScript("/common");

let uart_input_clk_freq = 48000000;

const uart_config_r5fss = [
    {
        name            : "UART0",
        baseAddr        : "CSL_UART0_U_BASE",
        inputClkFreq    : uart_input_clk_freq,
        intrNum         : 38,
        clockIds        : [ "SOC_RcmPeripheralId_LIN0_UART0" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_LIN0_UART0",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1",
                clkRate : uart_input_clk_freq,
            },
        ],
    },
    {
        name            : "UART1",
        baseAddr        : "CSL_UART1_U_BASE",
        inputClkFreq    : uart_input_clk_freq,
        intrNum         : 39,
        clockIds        : [ "SOC_RcmPeripheralId_LIN1_UART1" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_LIN1_UART1",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1",
                clkRate : uart_input_clk_freq,
            },
        ],
    },
    {
        name            : "UART2",
        baseAddr        : "CSL_UART2_U_BASE",
        inputClkFreq    : uart_input_clk_freq,
        intrNum         : 40,
        clockIds        : [ "SOC_RcmPeripheralId_LIN2_UART2" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_LIN2_UART2",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1",
                clkRate : uart_input_clk_freq,
            },
        ],
    },
    {
        name            : "UART3",
        baseAddr        : "CSL_UART3_U_BASE",
        inputClkFreq    : uart_input_clk_freq,
        intrNum         : 41,
        clockIds        : [ "SOC_RcmPeripheralId_LIN3_UART3" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_LIN3_UART3",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1",
                clkRate : uart_input_clk_freq,
            },
        ],
    },
    {
        name            : "UART4",
        baseAddr        : "CSL_UART4_U_BASE",
        inputClkFreq    : uart_input_clk_freq,
        intrNum         : 42,
        clockIds        : [ "SOC_RcmPeripheralId_LIN4_UART4" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_LIN4_UART4",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1",
                clkRate : uart_input_clk_freq,
            },
        ],
    },
    {
        name            : "UART5",
        baseAddr        : "CSL_UART5_U_BASE",
        inputClkFreq    : uart_input_clk_freq,
        intrNum         : 43,
        clockIds        : [ "SOC_RcmPeripheralId_LIN5_UART5" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_LIN5_UART5",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1",
                clkRate : uart_input_clk_freq,
            },
        ],
    },
];

function getConfigArr() {
    let uart_config;

    if(common.getSelfSysCfgCoreName().includes("r5f")) {
        uart_config = uart_config_r5fss;
    }
    else {
        uart_config = system.getScript(`/imports/drivers/uart/soc/uart_${common.getSocName()}_hsm.syscfg.js`).uart_config_m4f;
    }

    return uart_config;
}

function getDefaultClkRate() {
    return uart_input_clk_freq;
}

function getClockSourceOptions() {
    return [
        {name: "SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1"},
        {name: "SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT0"},
    ];
}

function getClockOptions(clkSrc) {
    let res = []
    if(clkSrc === "SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1") {
        res = [
            {name: 48000000, displayName: "48 MHz"},
            {name: 96000000, displayName: "96 MHz"},
            {name: 192000000, displayName: "192 MHz"},
        ];
    } else if (clkSrc === "SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT0") {
        res = [{name: 160000000, displayName: "160 MHz"}];
    } else {
        /* Bad clk source */
    }

    return res;
}

exports = {
    getConfigArr,
    getDefaultClkRate,
    getClockSourceOptions,
    getClockOptions
};
