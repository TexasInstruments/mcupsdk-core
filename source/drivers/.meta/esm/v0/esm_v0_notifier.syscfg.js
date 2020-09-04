let common = system.getScript("/common");
let soc = system.getScript(`/drivers/mibspi/soc/mibspi_${common.getSocName()}`);

function getConfigArr() {
    return soc.getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let configArr = getConfigArr();
    let config = configArr.find( o => o.name === moduleInstance.instance);

     return {
        ...config,
        ...moduleInstance,
     };
};

function getConfigurables()
{
    let config = [];

    config.push(/* Notifier attributes */
        {
            name: "groupNumber",
            displayName: "Error Group",
            default: 1,
            options: [
                {
                    name: 1,
                },
                {
                    name: 2,
                },
                {
                    name: 3,
                },
            ],
            description: "Group number to which the error number belongs",
            onChange: function (inst, ui) {
                if(inst.groupNumber == 1) {
                    ui.setIntrPriorityLvl.hidden = false;
                    ui.enableInfluenceOnErrPin.hidden = false;
                }
                else
                {
                    ui.setIntrPriorityLvl.hidden = true;
                    ui.enableInfluenceOnErrPin.hidden = true;
                }
            },
              
        },
        {
            name: "errorNumber",
            displayName: "Error Number",
            default: 0,
            description: "Error number for which the notify function is registered",
        },
        {
            name: "setIntrPriorityLvl",
            displayName: "Interrupt Priority Level",
            hidden: false,
            default: "HIGH",
            options: [
                {
                    name: "HIGH",
                    displayName: "High"
                },
                {
                    name: "LOW",
                    displayName: "Low"
                },
            ],
            description: "Set the interrupt priority level high or low. Applicable to Group 1 errors only",
        },
        {
            name: "enableInfluenceOnErrPin",
            displayName: "Enable Failure Influence",
            hidden: false,
            default: true,
            description: "Configurable output generation on ERROR pin. Applicable to Group 1 errors only",
        },
        {
            name: "notify",
            displayName: "Notify Callback",
            hidden: false,
            default: "NULL",
            description: "Notify callback function",
        },
    );
    return config;
}

let esm_notifier_module_name = "/drivers/esm/v0/esm_v0_notifier";

let esm_notifier_module = {
    displayName: "ESM Notifier Configuration",
    defaultInstanceName: "CONFIG_ESM_NOTIFY",
    config: getConfigurables(),
    validate : validate,
    getInstanceConfig,
};

/*
 *  ======== validate ========
 */
function validate(inst, report) {
  
  if(inst.groupNumber == 1)
  {
      common.validate.checkNumberRange(inst, report, "errorNumber", 0, 127, "dec");
  }
  else
  {
      common.validate.checkNumberRange(inst, report, "errorNumber", 0, 31, "dec");
  }
  
  if((inst.notify == "NULL") ||(inst.notify == "")) 
  {
      report.logError("Callback function MUST be provided ", inst, "notify");
  }
}

exports = esm_notifier_module;

