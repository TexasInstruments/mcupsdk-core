
let common = system.getScript("/common");

const topModules_main = [
    "/security/firewall_service/firewall_service",
    "/security/keyring_import/keyring_import",
];

const driverVer = {
    "firewall_service": {
        version: "v0",
    },
    "keyring_import": {
        version: "v0",
    },
};

const topModules_mcu = [

];

exports = {
    getTopModules: function() {
        let topModules = topModules_main;
        if(common.getSelfSysCfgCoreName().includes("m4f")) {
            topModules = topModules_mcu;
        }
        return topModules;
    },
    getDriverVer: function(driverName) {
        return driverVer[driverName].version;
    },
};
