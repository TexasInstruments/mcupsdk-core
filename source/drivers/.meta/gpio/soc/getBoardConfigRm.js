// Include the fs module
const fs = require('fs');
var path = require('path')


//To get the TISCI Id of the core
function getTisciId(core) {
    return tisciId[core];
}

//To get the Router Id of the Core
function getRouterID(core) {
    if(soc === "am65x"){
        let mcuCore = ["r5fss0-0", "r5fss0-1"]
        if (mcuCore.includes(core)) {
            return "TISCI_DEV_WKUP_GPIOMUX_INTRTR0"
        }
        return "TISCI_DEV_GPIOMUX_INTRTR0"
    }
    else{
        let mcuCore = ["m4fss0-0"]
        if (mcuCore.includes(core)) {
            return "TISCI_DEV_MCU_MCU_GPIOMUX_INTROUTER0"
        }
        return "TISCI_DEV_MAIN_GPIOMUX_INTROUTER0"
    }
}

//To extract number from the string
function extractNumber(data) {
    try {
        let num = data.match(/\d+/)[0];
        return num
    } catch (err) {
        writeErrorLog("Number Parsing Error !!\n" + err)
    }
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
    if (soc === "am65x" || soc === "am64x" || soc === "am243x" || soc === "am62x") {
        return soc
    }
    else {
        writeErrorLog("Device not found !!\n")
    }
}

//To parse data from the C file
function parseData(boardCfg, config, routerId, core, soc) {
    let arr = boardCfg.split(/{?}/);
    arr.forEach((line, idx) => {
        try {
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
                    case "am64x":
                    case "am243x":
                        numArr = tempArr[2].split(/[, ]+/);
                        startArr = tempArr[4].split(/[, ]+/);
                        numResource = parseInt(numArr[3]);
                        startResource = parseInt(startArr[3]);
                        config[core] = getCoreConfig(startResource, numResource);
                        break;
                    case "am65x":
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
        } catch (err) {
            writeErrorLog("File Parsing Error !!\n" + err)
        }
    });
}

function writeErrorLog(log) {
    fs.writeFile(outFile, log, (err) => {
        if (err) {
            return console.log(err);
        }
        console.log("Error Logged !!");
        process.exit(1);
    });
}

//Arguments passed from SysConfig
outFile = process.argv[2];
try {
    soc = getSocName(process.argv[3]);
} catch (err) {
    writeErrorLog("Device argument missing !!\n" + err)
}

//To pick the directory of the script file
var dir = __dirname

var tisciId = {};
if(soc === "am65x"){
    var tisciId = {
        "r5fss0-0": "TISCI_HOST_ID_R5_0",
        "r5fss0-1": "TISCI_HOST_ID_R5_2",
    };
}
else{
    var tisciId = {
        "m4fss0-0": "TISCI_HOST_ID_M4_0",
        "r5fss0-0": "TISCI_HOST_ID_MAIN_0_R5_1",
        "r5fss0-1": "TISCI_HOST_ID_MAIN_0_R5_3",
        "r5fss1-0": "TISCI_HOST_ID_MAIN_1_R5_1",
        "r5fss1-1": "TISCI_HOST_ID_MAIN_1_R5_3",
        "a53ss0-0": "TISCI_HOST_ID_A53_2",
    };
}

//Local k3BoardConfig.json in the SDK
try {
    var boardCfgFile = path.resolve(dir, 'k3BoardConfig.json')
    const data = fs.readFileSync(boardCfgFile, "utf8", { encoding: 'utf8', flag: 'r' });
    var boardCfg = JSON.parse(data);
} catch (err) {
    writeErrorLog("Default configuration Read Error !!\n" + err)
}

try {
    var cfg = path.resolve(dir, `../../../sciclient/sciclient_default_boardcfg/${soc}/sciclient_defaultBoardcfg_rm.c`)
    var cfgData = fs.readFileSync(cfg, "utf-8");
} catch (err) {
    writeErrorLog("Board Cfg File Read Error !!\n" + err);
    return;
}

let config = {};
if(soc === "am65x"){
    var coreList = ["r5fss0-0", "r5fss0-1"]
}
else{
    var coreList = ["m4fss0-0", "r5fss0-0", "r5fss0-1", "r5fss1-0", "r5fss1-1", "a53ss0-0", "a53ss0-1"]
}
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
