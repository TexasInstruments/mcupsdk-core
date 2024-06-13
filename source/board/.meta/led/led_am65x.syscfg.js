
let common = system.getScript("/common");

let led_devices = [
    {
        name        : "TPIC2810",
        type        : "I2C",
        i2cAddress  : 0x60,   /* Default address */
        ioIndex     : 0,
    },
];

function getConfigArr() {
    return led_devices;
}

exports = {
    getConfigArr,
};
