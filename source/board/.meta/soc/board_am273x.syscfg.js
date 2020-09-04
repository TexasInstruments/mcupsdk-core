let common = system.getScript("/common");

const driverVer = {
    "flash": {
        version: "v1",
    },
};

const topModules = [
    "/board/led/led",
    "/board/eeprom/eeprom",
    "/board/flash/flash",
];

exports = {
    getTopModules: function() {
        return topModules;
    },
    getDriverVer: function(driverName) {
        return driverVer[driverName].version;
    },
};
