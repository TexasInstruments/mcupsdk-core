
let common = system.getScript("/common");

const adcbuf_config_r5f = [
    {
        name: "ADCBUF0",
        baseAddr: "CSL_RSS_CTRL_U_BASE",
        interruptNum: 159,
        adcbufBaseAddr: "CSL_RSS_ADCBUF_READ_U_BASE",
        cqbufBaseAddr: "CSL_BSS_DFE_CQ1_U_BASE",
    },
];

const adcbuf_config_c66 = [
    {
        name: "ADCBUF0",
        baseAddr: "CSL_RSS_CTRL_U_BASE",
        interruptNum: 57,
        adcbufBaseAddr: "CSL_RSS_ADCBUF_READ_U_BASE",
        cqbufBaseAddr: "CSL_BSS_DFE_CQ1_U_BASE",
    },
];

function getConfigArr() {
    let adcbuf_config;

    if(common.getSelfSysCfgCoreName().includes("c66")) {
        adcbuf_config = adcbuf_config_c66;
    }
    else {
        adcbuf_config = adcbuf_config_r5f;
    }

    return adcbuf_config;
}

exports = {
    getConfigArr,
};
