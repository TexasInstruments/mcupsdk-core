
let common = system.getScript("/common");

const driverVer = {
    "flash": {
        version: "v0",
    },
};

const topModules = [
    "/board/eeprom/eeprom",
    "/board/ethphy/ethphy",
    "/board/flash/flash",
    "/board/led/led",

];

exports = {
    getTopModules: function() {
        return topModules;
    },
    getDriverVer: function(driverName) {
        return driverVer[driverName].version;
    },
};
