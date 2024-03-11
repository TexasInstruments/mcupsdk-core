
let common = system.getScript("/common");

let pmic_devices = [
    {
        name        : "TPS653860xx",
        type        : "MCSPI",
        instance    : "SPI1",
        deviceType  : "PMIC_DEV_BB_TPS65386X",
        instType    : "PMIC_MAIN_INST",
        commMode    : "PMIC_INTF_SPI",
    },
];

function getConfigArr() {
    return pmic_devices;
}

exports = {
    getConfigArr,
};