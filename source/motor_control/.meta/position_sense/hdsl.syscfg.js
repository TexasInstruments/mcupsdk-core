
let common = system.getScript("/common");
let hdsl_endat_pins = system.getScript("/motor_control/position_sense/hdsl_endat_pins.js");
let device = common.getDeviceName();
let hdsl_module_name = "/motor_control/position_sense/hdsl";

function onValidate(inst, validation) {

    for (let instance_index in inst.$module.$instances)
    {
        /* validation for booster pack */
        if((device!="am243x-lp")&&(instance.Booster_Pack))
        {
            validation.logError("Select only when using Booster Pack with LP",inst,"Booster_Pack");
        }
    }

}

let hdsl_module = {

    displayName: "HDSL Position Encoder",
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/motor_control/position_sense/templates/hdsl_templates.h.xdt",
            moduleName: hdsl_module_name,
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: hdsl_module_name,
        },
    },
    defaultInstanceName: "CONFIG_HDSL",
    config: [
        {
            name: "instance",
            displayName: "Instance",
            default: "ICSSG0",
            options: [
                {
                    name: "ICSSG0",
                },
                {
                    name: "ICSSG1",
                }
            ],
        },
        {
            name: "coreClk",
            displayName: "PRU-ICSS Core Clk (Hz)",
            default: 225*1000000,
            options: [
                {
                    name: 225*1000000,
                },

                {
                    name: 300*1000000,
                },

            ],
        },
        {
            name: "Booster_Pack",
            displayName: "Booster Pack",
            description: "Only for Booster Pack",
            default: false,
        },
        {
            name: "Mode",
            displayName: "Select Mode",
            description: "Mode selection",
            default: "FREE_RUN",
            options: [
                {
                    name: "FREE_RUN",
                },
                {
                    name: "SYNC",
                },
            ],
        },
        {
            name: "channel_0",
            displayName: "Select Channel 0",
            description: "Channel 0 Selection ",
            default: true,

        },
        {
            name: "channel_1",
            displayName: "Select Channel 1",
            description: "Channel 1 Selection ",
            default: false,

        },
    ],
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    pinmuxRequirements: hdsl_endat_pins.pinmuxRequirements,
    getInterfaceName: hdsl_endat_pins.getInterfaceName,
    getPeripheralPinNames: hdsl_endat_pins.getPeripheralPinNames,
    sharedModuleInstances: sharedModuleInstances,
};

function sharedModuleInstances(instance) {
    let modInstances = new Array();

    modInstances.push({
        name: "pru",
        displayName: "PRU ICSS Configuration",
        moduleName: '/drivers/pruicss/pruicss',
        requiredArgs: {
            instance: instance.instance,
            coreClk: instance.coreClk,
            iepSyncMode: true,
        },
    });
    if(device == "am243x-lp")
    {
       modInstances.push({
            name: "ENC1_EN",
            displayName: "Booster Pack Ch0 Enable Pin",
            moduleName: "/drivers/gpio/gpio",
            requiredArgs: {
                pinDir: "OUTPUT"
            },
        });
    }

    return (modInstances);
}

exports = hdsl_module;
