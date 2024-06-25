
let common = system.getScript("/common");

const topModules_main = [
    "/optiflash/RL2/RL2",
    "/optiflash/FLC/FLC",
    "/optiflash/RAT/RAT",
];

const driverVersion = {
    "rl2":
    {
        version: "v0"
    },
    'flc':
    {
        version: 'v0'
    },
    'rat':
    {
        version: 'v0'
    }
};


exports = {
    getTopModules: function() {
        return common.getSelfSysCfgCoreName().includes("r5f") ? topModules_main:[];
    },
    getDriverVer: function (driverName)
    {
        return driverVersion[driverName].version;
    }
};
