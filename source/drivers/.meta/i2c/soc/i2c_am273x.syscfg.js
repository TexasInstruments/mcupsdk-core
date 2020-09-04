
let common = system.getScript("/common");

let i2c_func_clk = 200000000;

const staticConfig_r5f = [
    {
        name: "MSS_I2CA",
        baseAddr: "CSL_MSS_I2C_U_BASE",
        intNum: "CSL_MSS_INTR_MSS_I2C_INT",
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
    {
        name: "RCSS_I2CA",
        baseAddr: "CSL_RCSS_I2CA_U_BASE",
        intNum: "CSL_MSS_INTR_RCSS_I2CA_INT",
        eventId: 0,
        funcClk: i2c_func_clk,
        clockIds: [ "SOC_RcmPeripheralId_RCSS_I2CA" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_RCSS_I2CA",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : i2c_func_clk,
            },
        ],
    },
    {
        name: "RCSS_I2CB",
        baseAddr: "CSL_RCSS_I2CB_U_BASE",
        intNum: "CSL_MSS_INTR_RCSS_I2CB_INT",
        eventId: 0,
        funcClk: i2c_func_clk,
        clockIds: [ "SOC_RcmPeripheralId_RCSS_I2CB" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_RCSS_I2CB",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : i2c_func_clk,
            },
        ],
    },
];

const staticConfig_c66f = [
    {
        name: "RCSS_I2CA",
        baseAddr: "CSL_RCSS_I2CA_U_BASE",
        intNum: "CSL_DSS_INTR_RCSS_I2CA_INT",
        eventId: 0,
        funcClk: i2c_func_clk,
        clockIds: [ "SOC_RcmPeripheralId_RCSS_I2CA" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_RCSS_I2CA",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : i2c_func_clk,
            },
        ],
    },
    {
        name: "RCSS_I2CB",
        baseAddr: "CSL_RCSS_I2CB_U_BASE",
        intNum: "CSL_DSS_INTR_RCSS_I2CB_INT",
        eventId: 0,
        funcClk: i2c_func_clk,
        clockIds: [ "SOC_RcmPeripheralId_RCSS_I2CB" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_RCSS_I2CB",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : i2c_func_clk,
            },
        ],
    },
    {
        name: "MSS_I2CA",
        baseAddr: "CSL_MSS_I2C_U_BASE",
        intNum: 0,
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

    if(common.getSelfSysCfgCoreName().includes("c66")) {
        staticConfigArr = staticConfig_c66f;
    }
    else {
        staticConfigArr = staticConfig_r5f;
    }

    return staticConfigArr;
}

function getInterfaceName(inst) {

    let interfaceName;
    if((inst.instance == "MSS_I2CA"))
    {
        interfaceName = "MSS_I2C";
         return interfaceName;
    }

    if((inst.instance == "RCSS_I2CA") || (inst.instance == "RCSS_I2CB"))
    {
        interfaceName = "RCSS_I2C";
        return interfaceName;
    }
}

function isMakeInstanceRequired() {
    return true;
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
