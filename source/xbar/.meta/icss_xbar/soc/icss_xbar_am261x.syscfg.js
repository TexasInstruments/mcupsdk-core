let common = system.getScript("/common");
let xbarSoc = system.getScript(`/xbar/soc/xbar_${common.getSocName()}`);

const internal_list = [
    {   name: "ICSS_XBAR_LIN0_INTR_REQ0", displayName: "LIN0_INTR_REQ0", path: "icss_xbar" },
    {   name: "ICSS_XBAR_LIN0_INTR_REQ1", displayName: "LIN0_INTR_REQ1", path: "icss_xbar" },
    {   name: "ICSS_XBAR_LIN1_INTR_REQ0", displayName: "LIN1_INTR_REQ0", path: "icss_xbar" },
    {   name: "ICSS_XBAR_LIN1_INTR_REQ1", displayName: "LIN1_INTR_REQ1", path: "icss_xbar" },
    {   name: "ICSS_XBAR_LIN2_INTR_REQ0", displayName: "LIN2_INTR_REQ0", path: "icss_xbar" },
    {   name: "ICSS_XBAR_LIN2_INTR_REQ1", displayName: "LIN2_INTR_REQ1", path: "icss_xbar" },
    {   name: "ICSS_XBAR_LIN3_INTR_REQ0", displayName: "LIN3_INTR_REQ0", path: "icss_xbar" },
    {   name: "ICSS_XBAR_LIN3_INTR_REQ1", displayName: "LIN3_INTR_REQ1", path: "icss_xbar" },
    {   name: "ICSS_XBAR_LIN4_INTR_REQ0", displayName: "LIN4_INTR_REQ0", path: "icss_xbar" },
    {   name: "ICSS_XBAR_LIN4_INTR_REQ1", displayName: "LIN4_INTR_REQ1", path: "icss_xbar" },
    {   name: "ICSS_XBAR_USART0_IRQ", displayName: "USART0_IRQ", path: "icss_xbar" },
    {   name: "ICSS_XBAR_USART1_IRQ", displayName: "USART1_IRQ", path: "icss_xbar" },
    {   name: "ICSS_XBAR_USART2_IRQ", displayName: "USART2_IRQ", path: "icss_xbar" },
    {   name: "ICSS_XBAR_USART3_IRQ", displayName: "USART3_IRQ", path: "icss_xbar" },
    {   name: "ICSS_XBAR_USART4_IRQ", displayName: "USART4_IRQ", path: "icss_xbar" },
    {   name: "ICSS_XBAR_USART5_IRQ", displayName: "USART5_IRQ", path: "icss_xbar" },
    {   name: "ICSS_XBAR_I2C0_IRQ", displayName: "I2C0_IRQ", path: "icss_xbar" },
    {   name: "ICSS_XBAR_I2C1_IRQ", displayName: "I2C1_IRQ", path: "icss_xbar" },
    {   name: "ICSS_XBAR_I2C2_IRQ", displayName: "I2C2_IRQ", path: "icss_xbar" },
    {   name: "ICSS_XBAR_I2C3_IRQ", displayName: "I2C3_IRQ", path: "icss_xbar" },
    {   name: "ICSS_XBAR_SPI0_IRQ", displayName: "SPI0_IRQ", path: "icss_xbar" },
    {   name: "ICSS_XBAR_SPI1_IRQ", displayName: "SPI1_IRQ", path: "icss_xbar" },
    {   name: "ICSS_XBAR_SPI2_IRQ", displayName: "SPI2_IRQ", path: "icss_xbar" },
    {   name: "ICSS_XBAR_SPI3_IRQ", displayName: "SPI3_IRQ", path: "icss_xbar" },
    {   name: "ICSS_XBAR_SPI4_IRQ", displayName: "SPI4_IRQ", path: "icss_xbar" },
    {   name: "ICSS_XBAR_QSPI_INTR", displayName: "QSPI_INTR", path: "icss_xbar" },
    {   name: "ICSS_XBAR_TPCC_INTG", displayName: "TPCC_INTG", path: "icss_xbar" },
    {   name: "ICSS_XBAR_TPCC_INT0", displayName: "TPCC_INT0", path: "icss_xbar" },
    {   name: "ICSS_XBAR_TPCC_INT1", displayName: "TPCC_INT1", path: "icss_xbar" },
    {   name: "ICSS_XBAR_TPCC_INT2", displayName: "TPCC_INT2", path: "icss_xbar" },
    {   name: "ICSS_XBAR_TPCC_INT3", displayName: "TPCC_INT3", path: "icss_xbar" },
    {   name: "ICSS_XBAR_TPCC_INT4", displayName: "TPCC_INT4", path: "icss_xbar" },
    {   name: "ICSS_XBAR_TPCC_INT5", displayName: "TPCC_INT5", path: "icss_xbar" },
    {   name: "ICSS_XBAR_TPCC_INT6", displayName: "TPCC_INT6", path: "icss_xbar" },
    {   name: "ICSS_XBAR_TPCC_INT7", displayName: "TPCC_INT7", path: "icss_xbar" },
    {   name: "ICSS_XBAR_TPCC_ERRINT", displayName: "TPCC_ERRINT", path: "icss_xbar" },
    {   name: "ICSS_XBAR_TPCC_MPINT", displayName: "TPCC_MPINT", path: "icss_xbar" },
    {   name: "ICSS_XBAR_TPCC_ERINT_0", displayName: "TPCC_ERINT_0", path: "icss_xbar" },
    {   name: "ICSS_XBAR_TPCC_ERINT_1", displayName: "TPCC_ERINT_1", path: "icss_xbar" },
    {   name: "ICSS_XBAR_MCANSS0_​EXT_​TS_​ROLLOVER_​LVL_​INT", displayName: "MCANSS0_​EXT_​TS_​ROLLOVER_​LVL_​INT", path: "icss_xbar" },
    {   name: "ICSS_XBAR_MCANSS0_​MCAN_​LVL_​INT_0", displayName: "MCANSS0_​MCAN_​LVL_​INT_0", path: "icss_xbar" },
    {   name: "ICSS_XBAR_MCANSS0_​MCAN_​LVL_​INT_1", displayName: "MCANSS0_​MCAN_​LVL_​INT_1", path: "icss_xbar" },
    {   name: "ICSS_XBAR_MCANSS1_​EXT_​TS_​ROLLOVER_​LVL_​INT", displayName: "MCANSS1_​EXT_​TS_​ROLLOVER_​LVL_​INT", path: "icss_xbar" },
    {   name: "ICSS_XBAR_MCANSS1_​MCAN_​LVL_​INT_0", displayName: "MCANSS1_​MCAN_​LVL_​INT_0", path: "icss_xbar" },
    {   name: "ICSS_XBAR_MCANSS1_​MCAN_​LVL_​INT_1", displayName: "MCANSS1_​MCAN_​LVL_​INT_1", path: "icss_xbar" },
    {   name: "ICSS_XBAR_MCANSS2_​EXT_​TS_​ROLLOVER_​LVL_​INT", displayName: "MCANSS2_​EXT_​TS_​ROLLOVER_​LVL_​INT", path: "icss_xbar" },
    {   name: "ICSS_XBAR_MCANSS2_​MCAN_​LVL_​INT_0", displayName: "MCANSS2_​MCAN_​LVL_​INT_0", path: "icss_xbar" },
    {   name: "ICSS_XBAR_MCANSS2_​MCAN_​LVL_​INT_1", displayName: "MCANSS2_​MCAN_​LVL_​INT_1", path: "icss_xbar" },
    {   name: "ICSS_XBAR_MCANSS3_​EXT_​TS_​ROLLOVER_​LVL_​INT", displayName: "MCANSS3_​EXT_​TS_​ROLLOVER_​LVL_​INT", path: "icss_xbar" },
    {   name: "ICSS_XBAR_MCANSS3_​MCAN_​LVL_​INT_0", displayName: "MCANSS3_​MCAN_​LVL_​INT_0", path: "icss_xbar" },
    {   name: "ICSS_XBAR_MCANSS3_​MCAN_​LVL_​INT_1", displayName: "MCANSS3_​MCAN_​LVL_​INT_1", path: "icss_xbar" },
    {   name: "ICSS_XBAR_MAILBOX_PRU_REQ_0", displayName: "MAILBOX_PRU_REQ_0", path: "icss_xbar" },
    {   name: "ICSS_XBAR_MAILBOX_PRU_REQ_1", displayName: "MAILBOX_PRU_REQ_1", path: "icss_xbar" },
    {   name: "ICSS_XBAR_MAILBOX_PRU_ACK_0", displayName: "MAILBOX_PRU_ACK_0", path: "icss_xbar" },
    {   name: "ICSS_XBAR_MAILBOX_PRU_ACK_1", displayName: "MAILBOX_PRU_ACK_1", path: "icss_xbar" },
];

let xbarProperties = {
    masterXbarList: ["gpio_int_xbar"],
    outputInstanceList: [
        { name: "ICSS_XBAR_ICSS_MODULE", count: 16},
    ],
    duplicatesPresent: false,
    moduleString: "icss_xbar",
}

function getOptionList(calledBy) {
    return xbarSoc.getOptionListSoc(calledBy, xbarProperties, internal_list);
}

function getConfigArr() {
    return xbarSoc.getXbarInstanceConfig(xbarProperties);
}

function supportXbarConfig(outputSelected, instance) {
    return xbarSoc.supportXbarConfigSoc(outputSelected, instance, xbarProperties);
}

exports = {
    getConfigArr,
    getOptionList,
    supportXbarConfig,
};
