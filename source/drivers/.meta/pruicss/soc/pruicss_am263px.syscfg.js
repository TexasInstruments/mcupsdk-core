
let common = system.getScript("/common");

let pru_icss_config = [
    {
        name            : "ICSSM0",
        mdioBaseAddr    : "(CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_PR1_MDIO_V1P7_MDIO_REGS_BASE)",
    },
];

function getConfigArr() {

    return pru_icss_config;
}

exports = {
    getConfigArr,
};
