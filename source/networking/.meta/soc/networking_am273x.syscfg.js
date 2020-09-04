
let common = system.getScript("/common");

const topModules_main = [
    "/networking/enet_cpsw/enet_cpsw",
];

const driverVer = {
    "enet_cpsw": {
        version: "v1",
    },
}

exports = {
    getTopModules: function() {
        return common.getSelfSysCfgCoreName().includes("r5f")||common.getSelfSysCfgCoreName().includes("c66")?topModules_main:[];
    },
    getDriverVer: function(driverName) {
        return driverVer[driverName].version;
    },
};
