let common   = system.getScript("/common");

function onChangeuseMDLA(inst, ui)
{
   if(inst.epwmMDB_useMDLA)
   {
        for(let uiConfigIndex = 0; uiConfigIndex < config.length; uiConfigIndex++)
        {
            let configName = config[uiConfigIndex].name;
                if (configName.startsWith("MDL_BLOCKA"))
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
                if (configName.startsWith("MDL_BLOCKA"))
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

function onChangeuseMDLB(inst, ui)
{
   if(inst.epwmMDB_useMDLB)
   {
        for(let uiConfigIndex = 0; uiConfigIndex < config.length; uiConfigIndex++)
        {
            let configName = config[uiConfigIndex].name;
                if (configName.startsWith("MDL_BLOCKB"))
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
                if (configName.startsWith("MDL_BLOCKB"))
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
        name: "MDL_BLOCKA",
        displayName: "BLOCK A",
        description: "",
        longDescription: "",
        config:
            [
                {
                    name: "epwmMDB_useMDLA",
                    displayName : "Enable MDL Block A",
                    description : 'Enable minimum deadband block A',
                    hidden      : false,
                    default     : false,
                    onChange    : onChangeuseMDLA
                },
                {
                    name: "epwmMDB_refSignalMDLA",
                    displayName : "Select Reference Signal MDL Block A",
                    description : 'Select Reference Signal for MDL Block A',
                    hidden      : true,
                    default     : "0",
                    options     : [
                        {name: "0", displayName: "DEPWMA"},
                        {name: "1", displayName: "MDLXBAR_OUT1"},
                        {name: "2", displayName: "MDLXBAR_OUT2"},
                        {name: "3", displayName: "MDLXBAR_OUT3"},
                        {name: "4", displayName: "MDLXBAR_OUT4"},
                        {name: "5", displayName: "MDLXBAR_OUT5"},
                        {name: "6", displayName: "MDLXBAR_OUT6"},
                        {name: "7", displayName: "MDLXBAR_OUT7"},
                        {name: "8", displayName: "MDLXBAR_OUT8"},
                        {name: "9", displayName: "MDLXBAR_OUT9"},
                        {name: "10", displayName: "MDLXBAR_OUT10"},
                        {name: "11", displayName: "MDLXBAR_OUT11"},
                        {name: "12", displayName: "MDLXBAR_OUT12"},
                        {name: "13", displayName: "MDLXBAR_OUT13"},
                        {name: "14", displayName: "MDLXBAR_OUT14"},
                        {name: "15", displayName: "MDLXBAR_OUT15"},
                    ]
                },
                {
                    name: "epwmMDB_invertMDLA",
                    displayName : "Invert Reference Signal",
                    description : 'Invert Reference Signal for minimum deadband block A',
                    hidden      : true,
                    default     : "0",
                    options     : [
                        {name:"0", displayName:"No inversion on the selected reference signal"},
                        {name:"1", displayName:" Invert the selected reference signal"}
                    ]
                },
                {
                    name: "epwmMDB_delayMDLA",
                    displayName : "Set Delay On PWMA",
                    description : 'Set Delay on INA for minimum deadband block A',
                    hidden      : true,
                    default     : "0",
                },
                {
                    name: "epwmMDB_blocksignalMDLA",
                    displayName : "Select Blocking Signal On PWMA",
                    description : 'Select Blocking signal on PWMA for minimum deadband block A',
                    hidden      : true,
                    default     : "0",
                    options     : [
                        {name:"0", displayName:"BLOCKA as the blocking signal on PWMA"},
                        {name:"1", displayName:"BLOCKB as the blocking signal on PWMA"}
                    ]
                },
                {
                    name: "epwmMDB_polMDLA",
                    displayName : "Blocking Signal AND/OR With PWMA",
                    description : 'Select Blocking signal to be AND-ed or OR-ed with PWMA for minimum deadband block A',
                    hidden      : true,
                    default     : "0",
                    options     : [
                        {name:"0", displayName:"Blocking signal is inverted and ANDed with PWMA"},
                        {name:"1", displayName:"Blocking signal is ORed with PWMA"}
                    ]
                },
            ]
    }
]);

let blockB_config = [];
blockB_config = blockB_config.concat([
    {
        name: "MDL_BLOCKB",
        displayName: "BLOCK B",
        description: "",
        longDescription: "",
        config:
            [
                {
                    name: "epwmMDB_useMDLB",
                    displayName : "Enable MDL Block B",
                    description : 'Enable minimum deadband block B',
                    hidden      : false,
                    default     : false,
                    onChange    : onChangeuseMDLB
                },
                {
                    name: "epwmMDB_refSignalMDLB",
                    displayName : "Select Reference Signal MDL Block B",
                    description : 'Select Reference Signal for MDL Block B',
                    hidden      : true,
                    default     : "0",
                    options     : [
                        {name: "0", displayName: "DEPWMB"},
                        {name: "1", displayName: "MDLXBAR_OUT1"},
                        {name: "2", displayName: "MDLXBAR_OUT2"},
                        {name: "3", displayName: "MDLXBAR_OUT3"},
                        {name: "4", displayName: "MDLXBAR_OUT4"},
                        {name: "5", displayName: "MDLXBAR_OUT5"},
                        {name: "6", displayName: "MDLXBAR_OUT6"},
                        {name: "7", displayName: "MDLXBAR_OUT7"},
                        {name: "8", displayName: "MDLXBAR_OUT8"},
                        {name: "9", displayName: "MDLXBAR_OUT9"},
                        {name: "10", displayName: "MDLXBAR_OUT10"},
                        {name: "11", displayName: "MDLXBAR_OUT11"},
                        {name: "12", displayName: "MDLXBAR_OUT12"},
                        {name: "13", displayName: "MDLXBAR_OUT13"},
                        {name: "14", displayName: "MDLXBAR_OUT14"},
                        {name: "15", displayName: "MDLXBAR_OUT15"},
                    ]
                },
                {
                    name: "epwmMDB_invertMDLB",
                    displayName : "Invert Reference Signal",
                    description : 'Invert Reference Signal for minimum deadband block B',
                    hidden      : true,
                    default     : "0",
                    options     : [
                        {name:"0", displayName:"No inversion on the selected reference signal"},
                        {name:"1", displayName:" Invert the selected reference signal"}
                    ]
                },
                {
                    name: "epwmMDB_delayMDLB",
                    displayName : "Set Delay On PWMB",
                    description : 'Set Delay on INB for minimum deadband block B',
                    hidden      : true,
                    default     : "0",
                },
                {
                    name: "epwmMDB_blocksignalMDLB",
                    displayName : "Select Blocking Signal On PWMB",
                    description : 'Select Blocking signal on PWMB for minimum deadband block B',
                    hidden      : true,
                    default     : "0",
                    options     : [
                        {name:"0", displayName:"BLOCKB as the blocking signal on PWMB"},
                        {name:"1", displayName:"BLOCKA as the blocking signal on PWMB"}
                    ]
                },
                {
                    name: "epwmMDB_polMDLB",
                    displayName : "Blocking Signal AND/OR With PWMB",
                    description : 'Select Blocking signal to be AND-ed or OR-ed with PWMB for minimum deadband block B',
                    hidden      : true,
                    default     : "0",
                    options     : [
                        {name:"0", displayName:"Blocking signal is inverted and ANDed with PWMB"},
                        {name:"1", displayName:"Blocking signal is Ored with PWMB"}
                    ]
                },
            ]
    }
]);

config = config.concat(blockA_config);
config = config.concat(blockB_config);


let epwmMdlSubmodule = {
    displayName: "EPWM MDL",
    // maxInstances: common.peripheralCount("EPWM"),
    defaultInstanceName: "EPWM_MDL",
    description: "Enhanced Pulse Width Modulator Mininum Deadband Logic",
    config: config,
    templates: {
        boardc : "",
        boardh : ""
    },
};


exports = epwmMdlSubmodule;