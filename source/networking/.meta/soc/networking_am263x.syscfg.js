
let common = system.getScript("/common");

const topModules = [
    "/networking/enet_cpsw/enet_cpsw",
    "/networking/icss_emac/icss_emac",
];

const driverVer = {
    "enet_cpsw": {
        version: "v3",
    },
}

exports = {
    getTopModules: function() {
        return common.getSelfSysCfgCoreName().includes("r5f")?topModules:[];
    },
    getDriverVer: function(driverName) {
        return driverVer[driverName].version;
    },
};
