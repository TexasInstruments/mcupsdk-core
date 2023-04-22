
let common = system.getScript("/common");

const driverVer = {
    "adcbuf": {
        version: "v0",
    },
    "bootloader": {
        version: "v0",
    },
    "cbuff": {
        version: "v0",
    },
    "crc": {
        version: "v1",
    },
    "csirx": {
        version: "v0",
    },
    "edma": {
        version: "v0",
    },
    "epwm": {
        version: "v2",
    },
    "esm": {
        version: "v0",
    },
    "gpio": {
        version: "v1",
    },
    "hsmclient": {
        version: "v0",
    },
    "gpadc": {
        version: "v0",
    },
    "hwa": {
        version: "v0",
    },
    "i2c": {
        version: "v1",
     },
    "ipc": {
        version: "v0",
    },
    "mcan": {
        version: "v0",
    },
    "qspi": {
        version: "v0",
    },
    "mibspi": {
        version: "v0",
    },
    "rti": {
        version: "v0",
    },
    "uart": {
        version: "v1",
    },
    "watchdog": {
        version: "v0",
    }
};

const topModules_r5f = [
    "/drivers/adcbuf/adcbuf",
    "/drivers/bootloader/bootloader",
    "/drivers/cbuff/cbuff",
    "/drivers/crc/crc",
    "/drivers/csirx/csirx",
    "/drivers/edma/edma",
    "/drivers/epwm/epwm",
    "/drivers/esm/esm",
    "/drivers/gpadc/gpadc",
    "/drivers/gpio/gpio",
    "/drivers/hsmclient/hsmclient",
    "/drivers/hwa/hwa",
    "/drivers/i2c/i2c",
    "/drivers/ipc/ipc",
    "/drivers/mcan/mcan",
    "/drivers/mibspi/mibspi",
    "/drivers/qspi/qspi",
    "/drivers/rti/rti",
    "/drivers/uart/uart",
    "/drivers/watchdog/watchdog",
];

const topModules_c66 = [
    "/drivers/adcbuf/adcbuf",
    "/drivers/cbuff/cbuff",
    "/drivers/crc/crc",
    "/drivers/csirx/csirx",
    "/drivers/edma/edma",
    "/drivers/esm/esm",
    "/drivers/hsmclient/hsmclient",
    "/drivers/hwa/hwa",
    "/drivers/ipc/ipc",
    "/drivers/rti/rti",
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
        "c66ss0"   : "CSL_CORE_ID_C66SS0",
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
        "c66ss0"   : "C66SS0",
    };
    return corename_map[common.getSelfSysCfgCoreName()];
}

exports = {
    getTopModules: function() {

        let topModules = topModules_r5f;

        if(common.getSelfSysCfgCoreName().includes("c66")) {
            topModules = topModules_c66;
        }

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
