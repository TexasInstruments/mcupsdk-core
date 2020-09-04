let common = system.getScript("/common");

const gpadc_config_r5fss = [
    {
        name                : "MSS_GPADC",
        baseAddr            : "CSL_MSS_GPADC_REG_U_BASE",
    },
];

function getConfigArr() {
    let gpadc_config;

    gpadc_config = gpadc_config_r5fss;

    return gpadc_config;
}

exports = {
    getConfigArr,
};