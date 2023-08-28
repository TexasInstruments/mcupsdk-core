
let common = system.getScript("/common");

let pmic_devices = [
    {
        name        : "TPS653860xx",
        type        : "MCSPI",
        instance    : "SPI1",
    },
];

function getConfigArr() {
    return pmic_devices;
}

exports = {
    getConfigArr,
};
