
let common = system.getScript("/common");

const cbuff_config_r5f = [
    {
        name                    : "CBUFF0",
        baseAddr                : "CSL_DSS_CBUFF_U_BASE",
        fifoBaseAddr            : "CSL_DSS_CBUFF_FIFO_U_BASE",
        maxLVDSLanesSupported   : 4,
        errorIntrNum            : 144,
        intrNum                 : 143,
        cbuffChannelId          : [
            18,
            19,
            20,
            21,
            22,
            23,
            24,
            25,
        ],
    },
];

const cbuff_config_c66 = [
    {
        name                    : "CBUFF0",
        baseAddr                : "CSL_DSS_CBUFF_U_BASE",
        fifoBaseAddr            : "CSL_DSS_CBUFF_FIFO_U_BASE",
        maxLVDSLanesSupported   : 4,
        errorIntrNum            : 42,
        intrNum                 : 41,
        cbuffChannelId : [
            18,
            19,
            20,
            21,
            22,
            23,
            24,
            25,
        ],
    },
];

function getConfigArr() {
    let cbuff_config;

    if(common.getSelfSysCfgCoreName().includes("c66")) {
        cbuff_config = cbuff_config_c66;
    }
    else {
        cbuff_config = cbuff_config_r5f;
    }

    return cbuff_config;
}

exports = {
    getConfigArr,
};
