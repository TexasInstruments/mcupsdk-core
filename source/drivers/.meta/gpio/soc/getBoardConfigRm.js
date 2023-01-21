// Include the fs module
const fs = require('fs');
var path = require('path')

//To pick the directory of the script file
var dir = __dirname

let tisciId = {
    "m4fss0-0": "TISCI_HOST_ID_M4_0",
    "r5fss0-0": "TISCI_HOST_ID_MAIN_0_R5_1",
    "r5fss0-1": "TISCI_HOST_ID_MAIN_0_R5_3",
    "r5fss1-0": "TISCI_HOST_ID_MAIN_1_R5_1",
    "r5fss1-1": "TISCI_HOST_ID_MAIN_1_R5_3",
    "a53ss0-0": "TISCI_HOST_ID_A53_2",
};

//To get the TISCI Id of the core
function getTisciId(core) {
    return tisciId[core];
}

//To get the Router Id of the Core
function getRouterID(core) {
    let mcuCore = ["m4fss0-0"]
    if (mcuCore.includes(core)) {
        return "TISCI_DEV_MCU_MCU_GPIOMUX_INTROUTER0"
    }
    return "TISCI_DEV_MAIN_GPIOMUX_INTROUTER0"
}

//To extract number from the string
function extractNumber(data) {
    return data.match(/\d+/)[0]
}

//Create config object for the core
function getCoreConfig(start, num) {
    let config = {};
    config["outPinCfg"] = [];
    for (let i = start; i < start + num; i++) {
        tmp = {}
        tmp["name"] = `${i}`;
        tmp["displayName"] = `ROUTER ${i}`
        config["outPinCfg"].push(tmp)
    }
    return config;
}

//To get K3 board config soc name
function getSocName(soc) {
    if (soc === "am64x" || soc === "am243x") {
        return "am64x_am243x"
    }
    else if (soc === "am62x") {
        return "am62x"
    }
    else {
        process.exit(1);
    }
}

//To parse data from the C file
function parseData(boardCfg, config, routerId, core, soc) {
    let arr = boardCfg.split(/{?}/);
    arr.forEach((line, idx) => {
        if (
            line.includes(routerId) &&
            line.includes(getTisciId(core))
        ) {
            let tempArr = line.split(/\r?\n/);
            let startArr, numArr, numResource, startResource = 0;
            //if SOC is am62x the start resources and num resources array changes
            switch (soc) {
                case "am62x":
                    startArr = tempArr[3].split(/[, ]+/);
                    numArr = tempArr[4].split(/[, ]+/);
                    numResource = parseInt(extractNumber(numArr[2]));
                    startResource = parseInt(extractNumber(startArr[2]));
                    config[core] = getCoreConfig(startResource, numResource);
                    break;
                case "am64x_am243x":
                    numArr = tempArr[2].split(/[, ]+/);
                    startArr = tempArr[4].split(/[, ]+/);
                    numResource = parseInt(numArr[3]);
                    startResource = parseInt(startArr[3]);
                    config[core] = getCoreConfig(startResource, numResource);
                    break;
                default:
                    break;
            }

        }
    });

}

//Arguments passed from SysConfig
outFile = process.argv[2];
soc = getSocName(process.argv[3]);

//Local File in the SDK
var boardCfgFile = path.resolve(dir, 'k3BoardConfig.json')

var coreList = ["m4fss0-0", "r5fss0-0", "r5fss0-1", "r5fss1-0", "r5fss1-1", "a53ss0-0", "a53ss0-1"]

const data = fs.readFileSync(boardCfgFile, "utf8", { encoding: 'utf8', flag: 'r' });
var boardCfg = JSON.parse(data);

try {
    var cfg = path.resolve(dir, `../../../sciclient/sciclient_default_boardcfg/${soc}/sciclient_defaultBoardcfg_rm.c`)
    var cfgData = fs.readFileSync(cfg, "utf-8");
} catch (err) {
    console.log("Board Cfg File Read Error !!");
    return;
}

let config = {};
for (const core of coreList) {
    routerId = getRouterID(core)
    parseData(cfgData, config, routerId, core, soc);
}
boardCfg[soc] = config


var json = JSON.stringify(boardCfg)

fs.writeFile(outFile, json, 'utf8', (err) => {
    if (err) throw err;
    console.log('File write complete');
}
);

fs.writeFile(boardCfgFile, json, 'utf8', (err) => {
    if (err) throw err;
    console.log('File write complete');
}
);
