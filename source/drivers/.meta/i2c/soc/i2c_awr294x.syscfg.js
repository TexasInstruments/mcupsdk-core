
let common = system.getScript("/common");

let i2c_func_clk = 150000000;

const staticConfig_r5f = [
    {
        name: "I2CA",
        baseAddr: "CSL_MSS_I2C_U_BASE",
        intNum: 52,
        eventId: 0,
        funcClk: i2c_func_clk,
        clockIds: [ "SOC_RcmPeripheralId_MSS_I2C" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_MSS_I2C",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : i2c_func_clk,
            },
        ],
    },
];

function getStaticConfigArr() {

    let staticConfigArr;

    staticConfigArr = staticConfig_r5f;

    return staticConfigArr;
}

function getInterfaceName(inst) {
    return "I2C";
}

function isMakeInstanceRequired() {
    return false;
}

function isFrequencyDefined()
{
    return true;
}

let soc = {

    getStaticConfigArr,
    getInterfaceName,
    isMakeInstanceRequired,
    isFrequencyDefined,
};

exports = soc;
