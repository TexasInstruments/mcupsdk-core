
let common = system.getScript("/common");

let i2c_func_clk = 96000000;

const staticConfig_r5f = [
    {
        name: "I2C0",
        baseAddr: "CSL_I2C0_U_BASE",
        intNum: 44,
        eventId: 0,
        funcClk: i2c_func_clk,
        clockIds: [ "SOC_RcmPeripheralId_I2C" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_I2C",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1",
                clkRate : i2c_func_clk,
            },
        ],
    },
    {
        name: "I2C1",
        baseAddr: "CSL_I2C1_U_BASE",
        intNum: 45,
        eventId: 0,
        funcClk: i2c_func_clk,
        clockIds: [ "SOC_RcmPeripheralId_I2C" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_I2C",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1",
                clkRate : i2c_func_clk,
            },
        ],
    },
    {
        name: "I2C2",
        baseAddr: "CSL_I2C2_U_BASE",
        intNum: 46,
        eventId: 0,
        funcClk: i2c_func_clk,
        clockIds: [ "SOC_RcmPeripheralId_I2C" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_I2C",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1",
                clkRate : i2c_func_clk,
            },
        ],
    },
    {
        name: "I2C3",
        baseAddr: "CSL_I2C3_U_BASE",
        intNum: 47,
        eventId: 0,
        funcClk: i2c_func_clk,
        clockIds: [ "SOC_RcmPeripheralId_I2C" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_I2C",
                clkId   : "SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1",
                clkRate : i2c_func_clk,
            },
        ],
    },

];

function getStaticConfigArr() {

    let cpu = common.getSelfSysCfgCoreName();
    let staticConfigArr = staticConfig_r5f;

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

