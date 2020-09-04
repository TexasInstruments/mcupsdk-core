
let common = system.getScript("/common");

const staticConfig = [
    {
        name: "FSI_RX0",
        baseAddr: "CSL_FSIRX0_CFG_BASE",
        intrNum1: 16,
        intrNum2: 17,
    },
    {
        name: "FSI_RX1",
        baseAddr: "CSL_FSIRX1_CFG_BASE",
        intrNum1: 18,
        intrNum2: 19,
    },
    {
        name: "FSI_RX2",
        baseAddr: "CSL_FSIRX2_CFG_BASE",
        intrNum1: 20,
        intrNum2: 21,
    },
    {
        name: "FSI_RX3",
        baseAddr: "CSL_FSIRX3_CFG_BASE",
        intrNum1: 22,
        intrNum2: 23,
    },
    {
        name: "FSI_RX4",
        baseAddr: "CSL_FSIRX4_CFG_BASE",
        intrNum1: 24,
        intrNum2: 25,
    },
    {
        name: "FSI_RX5",
        baseAddr: "CSL_FSIRX5_CFG_BASE",
        intrNum1: 26,
        intrNum2: 27,
    },
];

function getStaticConfigArr() {
    return staticConfig;
}

function getInterfaceName(inst) {
    return "FSI_RX";
}

let soc = {
    getStaticConfigArr,
    getInterfaceName,
    interruptXbarConfig: false,
};

exports = soc;
