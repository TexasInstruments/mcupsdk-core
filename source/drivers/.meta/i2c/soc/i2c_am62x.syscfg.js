
let common = system.getScript("/common");

let i2c_func_clk = 96000000;

/* On M4F, interrupt number as specified in TRM is input to the NVIC but from M4 point of view there are 16 internal interrupts
   and then the NVIC input interrupts start, hence we need to add +16 to the value specified by TRM */
const staticConfig_m4f = [
    {
        name: "MCU_I2C0",
        baseAddr: "CSL_MCU_I2C0_CFG_BASE",
        intNum: 17+16,
        eventId: 0,
        funcClk: i2c_func_clk,
        clockIds: [ "TISCI_DEV_MCU_I2C0" ],
    }
];

function getStaticConfigArr() {
    return staticConfig_m4f;
}

let soc = {

    getStaticConfigArr,
};

exports = soc;

