
let common = system.getScript("/common");

const crc_config_r5f = [
    {
        name: "MCU_MCRC0",
        baseAddr: "CSL_MCU_MCRC64_0_REGS_BASE",
        intrNum: 192,
        clockIds: [ "TISCI_DEV_MCU_MCRC64_0_CLK" ],
    },
];

/* On M4F, interrupt number as specified in TRM is input to the NVIC but from M4 point of view there are 16 internal interrupts
 * and then the NVIC input interrupts start, hence we need to add +16 to the value specified by TRM */
const crc_config_m4f = [
    {
        name: "MCU_MCRC0",
        baseAddr: "CSL_MCU_MCRC64_0_REGS_BASE",
        intrNum: 16 + 16,
        clockIds: [ "TISCI_DEV_MCU_MCRC64_0_CLK" ],
    },
];

function getConfigArr() {
    let crc_config;

    if(common.getSelfSysCfgCoreName().includes("m4f")) {
        crc_config = crc_config_m4f;
    }
    else {
        crc_config = crc_config_r5f;
    }

    return crc_config;
}

exports = {
    getConfigArr,
};
