let common = system.getScript("/common");

let mcan_func_clk = 80 * 1000 * 1000;

const mcan_config_m4fss = [
    {
        name            : "MCU_MCAN0",
        baseAddr        : "CSL_MCU_MCAN0_MSGMEM_RAM_BASE",
        intrNum         : 43 + 16,
        clockIds        : [ "TISCI_DEV_MCU_MCAN0" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_MCU_MCAN0",
                clkId   : "TISCI_DEV_MCU_MCAN0_MCANSS_CCLK_CLK",
                clkRate : mcan_func_clk,
            },
        ],
    },
    {
        name            : "MCU_MCAN1",
        baseAddr        : "CSL_MCU_MCAN1_MSGMEM_RAM_BASE",
        intrNum         : 46 + 16,
        clockIds        : [ "TISCI_DEV_MCU_MCAN1" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_MCU_MCAN1",
                clkId   : "TISCI_DEV_MCU_MCAN1_MCANSS_CCLK_CLK",
                clkRate : mcan_func_clk,
            },
        ],
    },
];

function getConfigArr() {
    let mcan_config;

    mcan_config = mcan_config_m4fss;

    return mcan_config;
}

function getInterfaceName(instance) {
    return "MCU_MCAN";
}

exports = {
    getConfigArr,
    getInterfaceName,
};
