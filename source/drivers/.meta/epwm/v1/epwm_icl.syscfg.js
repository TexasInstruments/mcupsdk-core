let common   = system.getScript("/common");
let device_peripheral = system.getScript(`/drivers/epwm/soc/epwm_${common.getSocName()}.syscfg.js`);

function onChangeuseICLA(inst, ui)
{
   if(inst.epwmICL_useICLA)
   {
        for(let uiConfigIndex = 0; uiConfigIndex < config.length; uiConfigIndex++)
        {
            let configName = config[uiConfigIndex].name;
                if (configName.startsWith("ICL_BLOCKA"))
                {
                    let groupLen= config[uiConfigIndex].config.length;
                    for(let i = 1; i < groupLen; i++)
                        {
                            let subConfigName = config[uiConfigIndex].config[i].name;
                            ui[subConfigName].hidden = false;
                        }
                    break;
                }
        }
   }
   else{
        for(let uiConfigIndex = 0; uiConfigIndex < config.length; uiConfigIndex++)
        {
            let configName = config[uiConfigIndex].name;
                if (configName.startsWith("ICL_BLOCKA"))
                {
                    let groupLen= config[uiConfigIndex].config.length;
                    for(let i = 1; i < groupLen; i++)
                        {
                            let subConfigName = config[uiConfigIndex].config[i].name;
                            ui[subConfigName].hidden = true;
                            inst[subConfigName] = "0";
                        }
                        break;
                }
        }
   }
}

function onChangeuseICLB(inst, ui)
{
   if(inst.epwmICL_useICLB)
   {
        for(let uiConfigIndex = 0; uiConfigIndex < config.length; uiConfigIndex++)
        {
            let configName = config[uiConfigIndex].name;
                if (configName.startsWith("ICL_BLOCKB"))
                {
                    let groupLen= config[uiConfigIndex].config.length;
                    for(let i = 1; i < groupLen; i++)
                        {
                            let subConfigName = config[uiConfigIndex].config[i].name;
                            ui[subConfigName].hidden = false;
                        }
                        break;
                }
        }
   }
   else{
        for(let uiConfigIndex = 0; uiConfigIndex < config.length; uiConfigIndex++)
        {
            let configName = config[uiConfigIndex].name;
                if (configName.startsWith("ICL_BLOCKB"))
                {
                    let groupLen= config[uiConfigIndex].config.length;
                    for(let i = 1; i < groupLen; i++)
                        {
                            let subConfigName = config[uiConfigIndex].config[i].name;
                            ui[subConfigName].hidden = true;
                            inst[subConfigName] = "0";
                        }
                    break;
                }
        }
   }
}

let config = [];

let blockA_config = [];
blockA_config = blockA_config.concat([
    {
        name: "ICL_BLOCKA",
        displayName: "BLOCK A",
        description: "",
        longDescription: "",
        config:
            [
                {
                    name: "epwmICL_useICLA",
                    displayName : "Enable ICL Block A",
                    description : 'Enable illegal combo logic block A',
                    hidden      : false,
                    default     : false,
                    onChange    : onChangeuseICLA
                },
                {
                    name: "epwmICLA_IN3source",
                    displayName : "Select IN3",
                    description : 'Xbar signal to be fed into ICL block A.',
                    hidden      : true,
                    default     : "0",
                    options     : [
                        {name:"0", displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT0"},
                        {name:"1", displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT1"},
                        {name:"2", displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT2"},
                        {name:"3", displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT3"},
                        {name:"4", displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT4"},
                        {name:"5", displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT5"},
                        {name:"6", displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT6"},
                        {name:"7", displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT7"},
                        {name:"8", displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT8"},
                        {name:"9", displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT9"},
                        {name:"10",displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT10"},
                        {name:"11",displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT11"},
                        {name:"12",displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT12"},
                        {name:"13",displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT13"},
                        {name:"14",displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT14"},
                        {name:"15",displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT15"},
                    ]
                },
            ]
    }
]);

config = config.concat(blockA_config);
for (let i = 0; i < 8; i++)
{
    let j = i & (1<<0);
    let k = (i & (1<<1)) >> 1;
    let l = (i & (1<<2)) >> 2;

    let BLOCKA_LUTDECX = {
        name: "epwmICLA_LUTDEC_"+i,
        displayName : "IN3=" + l + " IN2=" + k + " IN1="+ j,
        description : "LUTDEC_" + i +" for block A value",
        hidden      : true,
        default     : "0",
        options     : [
            {name:"0", displayName: "Force 0"},
            {name:"1", displayName: "Force 1"},
        ]
    }
    config[0].config.push(BLOCKA_LUTDECX);
}

let blockB_config = [];
blockB_config = blockB_config.concat([
    {
        name: "ICL_BLOCKB",
        displayName: "BLOCK B",
        description: "",
        longDescription: "",
        config:
            [
                {
                    name: "epwmICL_useICLB",
                    displayName : "Enable ICL Block B",
                    description : 'Enable illegal combo logic block B',
                    hidden      : false,
                    default     : false,
                    onChange    : onChangeuseICLB
                },
                {
                    name: "epwmICLB_IN3source",
                    displayName : "Select IN3",
                    description : 'Xbar signal to be fed into ICL block B.',
                    hidden      : true,
                    default     : "0",
                    options     : [
                        {name:"0", displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT0"},
                        {name:"1", displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT1"},
                        {name:"2", displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT2"},
                        {name:"3", displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT3"},
                        {name:"4", displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT4"},
                        {name:"5", displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT5"},
                        {name:"6", displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT6"},
                        {name:"7", displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT7"},
                        {name:"8", displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT8"},
                        {name:"9", displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT9"},
                        {name:"10",displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT10"},
                        {name:"11",displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT11"},
                        {name:"12",displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT12"},
                        {name:"13",displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT13"},
                        {name:"14",displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT14"},
                        {name:"15",displayName: "EPWM_ICL_IN3_SRC_ICLXBAR_OUT15"},
                    ]
                },
            ]
    }
]);

config = config.concat(blockB_config);

for (let i = 0; i < 8; i++)
{
    let j = i & (1<<0);
    let k = (i & (1<<1)) >> 1;
    let l = (i & (1<<2)) >> 2;

    let BLOCKB_LUTDECX = {
        name: "epwmICLB_LUTDEC_"+i,
        displayName : "IN3=" + l + " IN2=" + k + " IN1="+ j,
        description : "LUTDEC_" + i +" for block B value",
        hidden      : true,
        default     : "0",
        options     : [
            {name:"0", displayName: "Force 0"},
            {name:"1", displayName: "Force 1"},
        ]
    }
    config[1].config.push(BLOCKB_LUTDECX);
}

let epwmIclSubmodule = {
    displayName: "EPWM ICL",
    // maxInstances: common.peripheralCount("EPWM"),
    defaultInstanceName: "EPWM_ICL",
    description: "Enhanced Pulse Width Modulator Illegal combo logic",
    config: config,
    templates: {
        boardc : "",
        boardh : ""
    },
};


exports = epwmIclSubmodule;