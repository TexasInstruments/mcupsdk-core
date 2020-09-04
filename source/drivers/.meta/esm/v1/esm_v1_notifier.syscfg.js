let common = system.getScript("/common");
let soc = system.getScript(`/drivers/esm/soc/esm_${common.getSocName()}`);

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
            description: "Set the interrupt priority level high or low.",
        },
        {
            name: "enableInfluenceOnErrPin",
            displayName: "Enable Failure Influence",
            hidden: false,
            default: false,
            description: "Configurable output generation on ERROR pin.",
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

let esm_notifier_module_name = "/drivers/esm/v0/esm_v1_notifier";

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

    common.validate.checkNumberRange(inst, report, "errorNumber", 0, 1024, "dec");

    if((inst.notify == "NULL") ||(inst.notify == ""))
    {
        report.logError("Callback function MUST be provided ", inst, "notify");
    }
}

exports = esm_notifier_module;