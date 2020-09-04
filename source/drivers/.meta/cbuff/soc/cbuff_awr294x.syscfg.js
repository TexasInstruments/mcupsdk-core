
let common = system.getScript("/common");

const cbuff_config_r5f = [
    {
        name                    : "CBUFF0",
        baseAddr                : "CSL_DSS_CBUFF_U_BASE",
        fifoBaseAddr            : "CSL_DSS_CBUFF_FIFO_U_BASE",
        adcBufBaseAddr          : "CSL_RSS_ADCBUF_READ_U_BASE",
        maxLVDSLanesSupported   : 2,
        errorIntrNum            : 144,
        intrNum                 : 143,
        chirpModeStartIndex     : 1,
        chirpModeEndIndex       : 8,
        cpSingleChirpInterleavedAddr : [
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG4",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG4 + 4",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG4 + 8",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG4 + 12",
        ],
        cpSingleChirpNonInterleavedAddr : [
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG0",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG0 + 4",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG0 + 8",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG0 + 12",
        ],
        cpMultipleChirpNonInterleavedAddr : [
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH0CPREG0",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH1CPREG0",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH2CPREG0",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH3CPREG0",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH4CPREG0",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH5CPREG0",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH6CPREG0",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH7CPREG0",
        ],
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
        adcBufBaseAddr          : "CSL_RSS_ADCBUF_READ_U_BASE",
        maxLVDSLanesSupported   : 2,
        errorIntrNum            : 42,
        intrNum                 : 41,
        chirpModeStartIndex     : 1,
        chirpModeEndIndex       : 8,
        cpSingleChirpInterleavedAddr : [
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG4",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG4 + 4",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG4 + 8",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG4 + 12",
        ],
        cpSingleChirpNonInterleavedAddr : [
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG0",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG0 + 4",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG0 + 8",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG0 + 12",
        ],
        cpMultipleChirpNonInterleavedAddr : [
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH0CPREG0",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH1CPREG0",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH2CPREG0",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH3CPREG0",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH4CPREG0",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH5CPREG0",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH6CPREG0",
            "CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH7CPREG0",
        ],
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
