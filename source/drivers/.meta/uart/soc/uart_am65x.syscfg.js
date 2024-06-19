let common = system.getScript("/common");

let uart_input_clk_freq = 48000000;

const uart_config_r5fss = [
    {
        name            : "MCU_USART0",
        baseAddr        : "CSL_MCU_UART0_BASE",
        inputClkFreq    : uart_input_clk_freq,
        intrNum         : 30,
        clockIds        : [ "TISCI_DEV_MCU_UART0" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_MCU_UART0",
                clkId   : "TISCI_DEV_MCU_UART0_BUS_FCLK_CLK",
                clkRate : uart_input_clk_freq,
            },
        ],
        udmaPdmaChannels: [
            {
                txCh    : "UDMA_PDMA_CH_MCU_UART0_TX",
                rxCh    : "UDMA_PDMA_CH_MCU_UART0_RX",
            },
        ],
    },
    {
        name            : "USART0",
        baseAddr        : "CSL_UART0_BASE",
        inputClkFreq    : uart_input_clk_freq,
        intrNum         : 168, /* main2mcu_lvl_introuter_main_0_outl_8*/
        clockIds        : [ "TISCI_DEV_UART0" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_UART0",
                clkId   : "TISCI_DEV_UART0_BUS_FCLK_CLK",
                clkRate : uart_input_clk_freq,
            },
        ],
    },
    {
        name            : "USART1",
        baseAddr        : "CSL_UART1_BASE",
        inputClkFreq    : uart_input_clk_freq,
        intrNum         : 169, /* main2mcu_lvl_introuter_main_0_outl_9*/
        clockIds        : [ "TISCI_DEV_UART1" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_UART1",
                clkId   : "TISCI_DEV_UART1_BUS_FCLK_CLK",
                clkRate : uart_input_clk_freq,
            },
        ],
    },
    {
        name            : "USART2",
        baseAddr        : "CSL_UART2_BASE",
        inputClkFreq    : uart_input_clk_freq,
        intrNum         : 170, /* main2mcu_lvl_introuter_main_0_outl_10*/
        clockIds        : [ "TISCI_DEV_UART2" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_UART2",
                clkId   : "TISCI_DEV_UART2_BUS_FCLK_CLK",
                clkRate : uart_input_clk_freq,
            },
        ],
    },
];

function getConfigArr() {
    let uart_config;

    uart_config = uart_config_r5fss;

    return uart_config;
}

function getDefaultClkRate() {
    return uart_input_clk_freq;
}

function getClockOptions() {
    return [
        {name: 48000000, displayName: "48 MHz"},
        {name: 96000000, displayName: "96 MHz"},
        {name: 192000000, displayName: "192 MHz"},
        {name: 160000000, displayName: "160 MHz"},
    ]
}

exports = {
    getConfigArr,
    getDefaultClkRate,
    getClockOptions
};
