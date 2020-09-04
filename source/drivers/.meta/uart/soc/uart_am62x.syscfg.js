let common = system.getScript("/common");

/*
 * MCU_UART and WKUP_UART are using same PLL.
 * So CLK frequecy should be matching between both.
 * DM firmware is using 48000000 as the frequency for WKUP_UART.
 * Same should be used for MCU_UART.
 */
let uart_input_clk_freq_mcu = 48000000;


/* On M4F, interrupt number as specified in TRM is input to the NVIC but from M4 point of view there are 16 internal interrupts
 * and then the NVIC input interrupts start, hence we need to add +16 to the value specified by TRM */
const uart_config_m4f = [
    {
        name            : "MCU_USART0",
        baseAddr        : "CSL_MCU_UART0_BASE",
        inputClkFreq    : uart_input_clk_freq_mcu,
        intrNum         : 24 + 16,
        clockIds        : [ "TISCI_DEV_MCU_UART0" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_MCU_UART0",
                clkId   : "TISCI_DEV_MCU_UART0_FCLK_CLK",
                clkRate : uart_input_clk_freq_mcu,
            },
        ],
    },
];

function getConfigArr() {
    let uart_config;

    if(common.getSelfSysCfgCoreName().includes("m4f")) {
        uart_config = uart_config_m4f;
    }

    return uart_config;
}

function getDefaultClkRate() {
    let uart_input_clk_freq;
    if(common.getSelfSysCfgCoreName().includes("m4f")) {
        uart_input_clk_freq = uart_input_clk_freq_mcu;
    }
    return uart_input_clk_freq;
}

exports = {
    getConfigArr,
    getDefaultClkRate
};
