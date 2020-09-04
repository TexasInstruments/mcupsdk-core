
let common = system.getScript("/common");

const staticConfig = [
    {
        name: "FSIRX0",
        baseAddr: "CSL_CONTROLSS_FSI_RX0_U_BASE",
    },
    {
        name: "FSIRX1",
        baseAddr: "CSL_CONTROLSS_FSI_RX1_U_BASE",
    },
    {
        name: "FSIRX2",
        baseAddr: "CSL_CONTROLSS_FSI_RX2_U_BASE",
    },
    {
        name: "FSIRX3",
        baseAddr: "CSL_CONTROLSS_FSI_RX3_U_BASE",
    },
];

function getStaticConfigArr() {
    return staticConfig;
}

function getInterfaceName(inst) {
    return "FSIRX";
}

let soc = {
    getStaticConfigArr,
    getInterfaceName,
    interruptXbarConfig: true,
};

exports = soc;
