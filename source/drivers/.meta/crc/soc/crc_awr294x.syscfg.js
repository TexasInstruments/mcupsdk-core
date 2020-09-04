
let common = system.getScript("/common");

const crc_config_r5f = [
    {
        name: "MSS_MCRC0",
        baseAddr: "CSL_MSS_MCRC_U_BASE",
        intrNum: 24,
        clockIds: [ "0" ],    //TODO
    },
];

const crc_config_c66 = [
    {
        name: "DSS_MCRC0",
        baseAddr: "CSL_DSS_MCRC_U_BASE",
        intrNum: 34,
        clockIds: [ "0" ],    //TODO
    },
];

function getConfigArr() {
    let crc_config;

    if(common.getSelfSysCfgCoreName().includes("c66")) {
        crc_config = crc_config_c66;
    }
    else {
        crc_config = crc_config_r5f;
    }

    return crc_config;
}

exports = {
    getConfigArr,
};
