
let common = system.getScript("/common");

const driverVer = {
    "adc": {
        version: "v0",
    },
    "bootloader": {
        version: "v0",
    },
    "crc": {
        version: "v0",
    },
    "ddr": {
        version: "v0",
    },
    "ecap": {
        version: "v0",
    },
    "epwm": {
        version: "v0",
    },
    "eqep": {
        version: "v0",
    },
    "esm": {
        version: "v1",
    },
    "firewall": {
        version: "v0",
    },
    "fsi_rx": {
        version: "v0",
    },
    "fsi_tx": {
        version: "v0",
    },
    "gpio": {
        version: "v0_1",
    },
    "gpmc": {
        version: "v0",
    },
    "gtc": {
        version: "v0",
    },
    "i2c": {
        version: "v0",
    },
    "ipc": {
        version: "v0",
    },
    "ddr": {
        version: "v0",
    },
    "mcan": {
        version: "v0",
    },
    "mcspi": {
        version: "v0",
    },
    "mmcsd": {
        version: "v0",
    },
    "ospi": {
        version: "v0",
    },
    "pcie": {
        version: "v0",
    },
    "pruicss": {
        version: "g_v0",
    },
    "sciclient": {
        version: "v0",
    },
    "uart": {
        version: "v0",
    },
    "udma": {
        version: "v0",
    },
    "watchdog": {
        version: "v0",
    },
};

const topModules_main = [
    "/drivers/adc/adc",
    "/drivers/bootloader/bootloader",
    "/drivers/crc/crc",
    "/drivers/ddr/ddr",
    "/drivers/ecap/ecap",
    "/drivers/epwm/epwm",
    "/drivers/eqep/eqep",
    "/drivers/esm/esm",
    "/drivers/firewall/firewall",
    "/drivers/fsi_rx/fsi_rx",
    "/drivers/fsi_tx/fsi_tx",
    "/drivers/gpio/gpio",
    "/drivers/gpmc/gpmc",
    "/drivers/gtc/gtc",
    "/drivers/i2c/i2c",
    "/drivers/ipc/ipc",
    "/drivers/mcan/mcan",
    "/drivers/mcspi/mcspi",
    "/drivers/mmcsd/mmcsd",
    "/drivers/ospi/ospi",
    "/drivers/pcie/pcie",
    "/drivers/pruicss/pruicss",
    "/drivers/uart/uart",
    "/drivers/udma/udma",
    "/drivers/watchdog/watchdog",
];

const topModules_mcu = [
    "/drivers/crc/crc",
    "/drivers/gpio/gpio",
    "/drivers/i2c/i2c",
    "/drivers/ipc/ipc",
    "/drivers/mcspi/mcspi",
    "/drivers/uart/uart",
];

const topModules_a53 = [
    "/drivers/gpio/gpio",
    "/drivers/gtc/gtc",
    "/drivers/i2c/i2c",
    "/drivers/ipc/ipc",
    "/drivers/uart/uart",
    "/drivers/udma/udma",
    "/drivers/watchdog/watchdog",
];

function getCpuID() {
    let corename_map = {
        "m4fss0-0" : "CSL_CORE_ID_M4FSS0_0",
        "r5fss0-0" : "CSL_CORE_ID_R5FSS0_0",
        "r5fss0-1" : "CSL_CORE_ID_R5FSS0_1",
        "r5fss1-0" : "CSL_CORE_ID_R5FSS1_0",
        "r5fss1-1" : "CSL_CORE_ID_R5FSS1_1",
        "a53ss0-0" : "CSL_CORE_ID_A53SS0_0",
        "a53ss0-1" : "CSL_CORE_ID_A53SS0_1",
    };

    return corename_map[common.getSelfSysCfgCoreName()];
}

function getSelfCoreID() {
    let corename_map = {
        "m4fss0-0" : "M4FSS0_CORE0",
        "r5fss0-0" : "R5FSS0_CORE0",
        "r5fss0-1" : "R5FSS0_CORE1",
        "r5fss1-0" : "R5FSS1_CORE0",
        "r5fss1-1" : "R5FSS1_CORE1",
        "a53ss0-0" : "A53SS0_CORE0",
        "a53ss0-1" : "A53SS0_CORE1",
    };

    return corename_map[common.getSelfSysCfgCoreName()];
}

exports = {
    getTopModules: function() {

        let topModules = topModules_main;

        if(common.getSelfSysCfgCoreName().includes("m4f")) {
            topModules = topModules_mcu;
        }
        if (common.getSelfSysCfgCoreName().match(/a53*/)) {
            topModules = topModules_a53;
        }

        return topModules;
    },
    getDriverVer: function(driverName) {
        return driverVer[driverName].version;
    },
    getCpuID,
    getSelfCoreID,
};
