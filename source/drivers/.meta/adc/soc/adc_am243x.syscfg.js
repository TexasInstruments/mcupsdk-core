
let common = system.getScript("/common");

const staticConfig = [
    {
        name: "ADC0",
        baseAddr: "CSL_ADC0_BASE",
        intrNum: 128,
        clockIds: [ "TISCI_DEV_ADC0_ADC_CLK" ],
        numBits: 12,
    },
];

function getStaticConfigArr() {
    return staticConfig;
}

let soc = {
    getStaticConfigArr,
};

exports = soc;
