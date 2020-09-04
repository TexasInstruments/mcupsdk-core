
let common = system.getScript("/common");

const driverVer = {
    "gpio":{
        version:"v0",
    },
    "i2c": {
        version: "v0",
    },
    "ipc": {
        version: "v0",
    },
    "mcan": {
        version: "v0",
    },
    "sciclient": {
        version: "v0",
    },
    "uart": {
        version: "v0",
    },
    "mcspi": {
        version: "v0",
    },
};

const topModules_main = [
];

const topModules_mcu = [
    "/drivers/gpio/gpio",
    "/drivers/i2c/i2c",
    "/drivers/ipc/ipc",
    "/drivers/mcan/mcan",
    "/drivers/uart/uart",
    "/drivers/mcspi/mcspi",
];

function getCpuID() {
    let corename_map = {
        "m4fss0-0" : "CSL_CORE_ID_M4FSS0_0",
        "r5fss0-0" : "CSL_CORE_ID_R5FSS0_0",
        "a53ss0-0" : "CSL_CORE_ID_A53SS0_0",
        "a53ss0-1" : "CSL_CORE_ID_A53SS0_1",
        "a53ss1-0" : "CSL_CORE_ID_A53SS1_0",
        "a53ss1-1" : "CSL_CORE_ID_A53SS1_1",
    };

    return corename_map[common.getSelfSysCfgCoreName()];
}

function getSelfCoreID() {
    let corename_map = {
        "m4fss0-0" : "M4FSS0_CORE0",
        "r5fss0-0" : "R5FSS0_CORE0",
        "a53ss0-0" : "A53SS0_CORE0",
        "a53ss0-1" : "A53SS0_CORE1",
        "a53ss1-0" : "A53SS1_CORE0",
        "a53ss1-1" : "A53SS1_CORE1",
    };

    return corename_map[common.getSelfSysCfgCoreName()];
}

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
    getCpuID,
    getSelfCoreID,
};
