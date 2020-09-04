
let common = system.getScript("/common");

let eeprom_devices = [
    {
        name: "AT24C",
        i2cAddress: 0x50,   /* Default address */
    },
];

function getConfigArr() {
    return eeprom_devices;
}

exports = {
    getConfigArr,
};
