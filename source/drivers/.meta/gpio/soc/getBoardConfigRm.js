// Include the fs module
const fs = require('fs');
var path = require('path')

//To pick the directory of the script file
var dir = __dirname

let tisci_id = {
    "m4fss0-0": "TISCI_HOST_ID_M4_0",
    "r5fss0-0": "TISCI_HOST_ID_MAIN_0_R5_1",
    "r5fss0-1": "TISCI_HOST_ID_MAIN_0_R5_3",
    "r5fss1-0": "TISCI_HOST_ID_MAIN_1_R5_1",
    "r5fss1-1": "TISCI_HOST_ID_MAIN_1_R5_3",
    "a53ss0-0": "TISCI_HOST_ID_A53_2",
};

//To get the TISCI Id of the core
function getTisciId(core) {
    return tisci_id[core];
}

//To get the Router Id of the Core
function getRouterID(core) {
    let mcu_core = ["m4fss0-0"]
    if (mcu_core.includes(core)) {
        return "TISCI_DEV_MCU_MCU_GPIOMUX_INTROUTER0"
    }
    return "TISCI_DEV_MAIN_GPIOMUX_INTROUTER0"
}

//To extract number from the string
function extractNumber(data){
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

//To parse data from the C file
function parseData(boardConfig, router_id, core,soc) {
    let arr = data.split(/{?}/);
    arr.forEach((line, idx) => {
        if (
            line.includes(router_id) &&
            line.includes(getTisciId(core))
        ) {
            let tempArr = line.split(/\r?\n/);
            //if SOC is am62x the start resources and num resources array changes
            if(soc=="am62x"){
                let start_array = tempArr[3].split(/[, ]+/);
                let num_array = tempArr[4].split(/[, ]+/);
                let num_resources = parseInt(extractNumber(num_array[2]));
                let start_resources = parseInt(extractNumber(start_array[2]));
                boardConfig[core] =  getCoreConfig(start_resources, num_resources)
                return
            }
            let num_array = tempArr[2].split(/[, ]+/);
            let start_array = tempArr[4].split(/[, ]+/);
            let num_resources = parseInt(num_array[3]);
            let start_resources = parseInt(start_array[3]);
            boardConfig[core] =  getCoreConfig( start_resources, num_resources)
            return
        }
    });

}

var boardConfigSocList = ["am64x_am243x","am62x"]
var coreList = ["m4fss0-0", "r5fss0-0", "r5fss0-1", "r5fss1-0", "r5fss1-1", "a53ss0-0", "a53ss0-1"]
let boardConfig = {}
for (const soc of boardConfigSocList){
    var cfg = path.resolve(dir,`../../../sciclient/sciclient_default_boardcfg/${soc}/sciclient_defaultBoardcfg_rm.c`)
    var data = fs.readFileSync(cfg,"utf-8");
    let Config={}
    for (const core of coreList) {
        router_id = getRouterID(core)
        parseData(Config, router_id, core,soc);
    }
    boardConfig[soc]=Config
}

outFile = process.argv[2]
//outFile = "k3BoardConfig.json"
var json = JSON.stringify(boardConfig)
fs.writeFile(outFile, json, 'utf8', function (err) {
    if (err) throw err;
    console.log('File write complete');
}
);
