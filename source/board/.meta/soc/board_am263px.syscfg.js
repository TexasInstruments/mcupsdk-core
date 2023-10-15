
let common = system.getScript("/common");

const driverVer = {
    "flash": {
        version: "v2",
    },
};

const topModules_main = [
      "/board/eeprom/eeprom",
      "/board/led/led",
      "/board/flash/flash",
      "/board/ethphy/ethphy",
      "/board/pmic/pmic"
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
};
