
let common = system.getScript("/common");

const topModules_main = [
    "/industrial_comms/ethercat/ethercat",
];

exports = {
    getTopModules: function() {
        let topModules = topModules_main;
        return common.getSelfSysCfgCoreName().includes("r5f")?topModules_main:[];
    },
};
