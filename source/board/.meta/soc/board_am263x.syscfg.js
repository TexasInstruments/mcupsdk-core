
let common = system.getScript("/common");

const driverVer = {
    "flash": {
        version: "v1",
    },
    "psram": {
        version: "v0",
    },
    "parallelRam": {
        version: "v0",
    },
    "ram": {
        version: "v0",
    }
};

const topModules_main = [
      "/board/eeprom/eeprom",
      "/board/led/led",
      "/board/flash/flash",
      "/board/ethphy/ethphy",
      "/board/ram/ram",
];
const topModules_mcu = [
];

exports = {
    getTopModules: function() {

        let topModules = topModules_main;

        if(common.getSelfSysCfgCoreName().includes("hsm")) {
            topModules = topModules_mcu;
        }

        return topModules;
    },
    getDriverVer: function(driverName) {
        return driverVer[driverName].version;
    },
    getDriverInstanceValid: function(driverName) {
        let valid = false;
        if(driverName in driverVer)
        {
            valid = true;
        }
        return valid;
    }
};
