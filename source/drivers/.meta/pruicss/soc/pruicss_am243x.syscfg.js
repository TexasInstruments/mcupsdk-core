
let common = system.getScript("/common");

let pru_icss_config = [
    {
        name            : "ICSSG0",
        mdioBaseAddr    : "CSL_PRU_ICSSG0_PR1_MDIO_V1P7_MDIO_BASE",
        clockIds        : [ "TISCI_DEV_PRU_ICSSG0" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_PRU_ICSSG0",
                clkId   : "TISCI_DEV_PRU_ICSSG0_CORE_CLK",
                clkRate : 200000000,
            },
            {
                moduleId: "TISCI_DEV_PRU_ICSSG0",
                clkId   : "TISCI_DEV_PRU_ICSSG0_UCLK_CLK",
                clkRate : 192000000,
            },
            {
                moduleId: "TISCI_DEV_PRU_ICSSG0",
                clkId   : "TISCI_DEV_PRU_ICSSG0_IEP_CLK",
                clkRate : 200000000,
            },
        ],
    },
    {
        name            : "ICSSG1",
        mdioBaseAddr    : "CSL_PRU_ICSSG1_PR1_MDIO_V1P7_MDIO_BASE",
        clockIds        : [ "TISCI_DEV_PRU_ICSSG1" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_PRU_ICSSG1",
                clkId   : "TISCI_DEV_PRU_ICSSG1_CORE_CLK",
                clkRate : 200000000,
            },
            {
                moduleId: "TISCI_DEV_PRU_ICSSG1",
                clkId   : "TISCI_DEV_PRU_ICSSG1_UCLK_CLK",
                clkRate : 192000000,
            },
            {
                moduleId: "TISCI_DEV_PRU_ICSSG1",
                clkId   : "TISCI_DEV_PRU_ICSSG1_IEP_CLK",
                clkRate : 200000000,
            },
        ],
    },
];

function getConfigArr() {

    return pru_icss_config;
}

exports = {
    getConfigArr,
};
