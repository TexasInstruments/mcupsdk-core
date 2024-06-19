
let common = system.getScript("/common");

const driverVer = {
    "gpio": {
        version: "v0_1",
    },
    "i2c": {
        version: "v0",
    },
    "uart": {
        version: "v0",
    },
    "udma": {
        version: "v0",
    },
};

const topModules = [
    "/drivers/gpio/gpio",
    "/drivers/i2c/i2c",
    "/drivers/uart/uart",
    "/drivers/udma/udma",
];

function getCpuID() {
    let corename_map = {
        "r5fss0-0" : "CSL_CORE_ID_R5FSS0_0",
        "r5fss0-1" : "CSL_CORE_ID_R5FSS0_1",
    };

    return corename_map[common.getSelfSysCfgCoreName()];
}

function getSelfCoreID() {
    let corename_map = {
        "r5fss0-0" : "R5FSS0_CORE0",
        "r5fss0-1" : "R5FSS0_CORE1",
    };

    return corename_map[common.getSelfSysCfgCoreName()];
}

exports = {
    getTopModules: function() {
        return topModules;
    },
    getDriverVer: function(driverName) {
        return driverVer[driverName].version;
    },
    getCpuID,
    getSelfCoreID,
};
