
let common = system.getScript("/common");

const driverVer = {
    "adc": {
        version: "v1",
    },
    "bootloader": {
        version: "v0",
    },
    "cmpss": {
        version: "v0",
    },
    "dac": {
        version: "v0",
    },
    "ecap": {
        version: "v1",
    },
    "edma": {
        version: "v0",
    },
    "epwm": {
        version: "v1",
    },
    "eqep": {
        version: "v1",
    },
    "fsi_rx": {
        version: "v0",
    },
    "fsi_tx": {
        version: "v0",
    },
    "gpio": {
        version: "v0",
    },
    "gpmc": {
        version: "v1",
    },
    "hsmclient": {
        version: "v0",
    },
    "i2c": {
        version: "v1",
    },
    "ipc": {
        version: "v0",
    },
    "lin": {
        version: "v0",
    },
    "mcan": {
        version: "v0",
    },
    "mcu_bist": {
        version: "v0",
    },
    "mcspi": {
        version: "v1",
    },
    "mmcsd": {
        version: "v1",
    },
    "pruicss": {
        version: "m_v0",
    },
    "qspi": {
        version: "v0",
    },
    "rti": {
        version: "v0",
    },
    "sdfm": {
        version: "v0",
    },
    "uart": {
        version: "v2",
    },
    "watchdog": {
        version: "v0",
    },
};

const topModules_main = [
    "/drivers/adc/adc",
    "/drivers/bootloader/bootloader",
    "/drivers/cmpss/cmpss",
    "/drivers/dac/dac",
    "/drivers/ecap/ecap",
    "/drivers/edma/edma",
    "/drivers/epwm/epwm",
    "/drivers/eqep/eqep",
    "/drivers/fsi_rx/fsi_rx",
    "/drivers/fsi_tx/fsi_tx",
    "/drivers/gpio/gpio",
    "/drivers/gpmc/gpmc",
    "/drivers/hsmclient/hsmclient",
    "/drivers/i2c/i2c",
    "/drivers/ipc/ipc",
    "/drivers/lin/lin",
    "/drivers/mcan/mcan",
    "/drivers/mcu_bist/mcu_bist",
    "/drivers/mcspi/mcspi",
    "/drivers/mmcsd/mmcsd",
    "/drivers/pruicss/pruicss",
    "/drivers/qspi/qspi",
    "/drivers/rti/rti",
    "/drivers/sdfm/sdfm",
    "/drivers/uart/uart",
    "/drivers/watchdog/watchdog",
];

const topModules_mcu = [
    "/drivers/uart/uart",

];

function getCpuID() {
    let corename_map = {
        "r5fss0-0" : "CSL_CORE_ID_R5FSS0_0",
        "r5fss0-1" : "CSL_CORE_ID_R5FSS0_1",
        "r5fss1-0" : "CSL_CORE_ID_R5FSS1_0",
        "r5fss1-1" : "CSL_CORE_ID_R5FSS1_1",
    };
    if(common.getSelfSysCfgCoreName().includes("hsm")) {
            corename_map = system.getScript(`/imports/drivers/soc/drivers_${common.getSocName()}_hsm.syscfg.js`).corename_map_hsm;
        }

    return corename_map[common.getSelfSysCfgCoreName()];
}

function getSelfCoreID() {
    let corename_map = {
        "r5fss0-0" : "R5FSS0_CORE0",
        "r5fss0-1" : "R5FSS0_CORE1",
        "r5fss1-0" : "R5FSS1_CORE0",
        "r5fss1-1" : "R5FSS1_CORE1",
    };

    return corename_map[common.getSelfSysCfgCoreName()];
}

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
    getCpuID,
    getSelfCoreID,
};
