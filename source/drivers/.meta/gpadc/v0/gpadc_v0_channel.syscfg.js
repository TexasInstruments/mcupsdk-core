let common = system.getScript("/common");
let soc = system.getScript(`/drivers/gpadc/soc/gpadc_${common.getSocName()}`);

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

    config.push(/* Channel attributes */
        {
            name: "ChannelIndex",
            displayName: "Channel Index(1-9)",
            description: "Set Channel number between 1 to 9",
            default: 1,
        },
        {
            name: "isConfigured",
            displayName: "Configure Channel",
            default: "TRUE",
            options: [
                {
                    name: "FALSE",
                    displayName: "FALSE"
                },
                {
                    name: "TRUE",
                    displayName: "TRUE"
                },
            ],
            description: "This determines whether the Channel is configured",
        },
        {
            name: "isBufferedMode",
            displayName: "Buffer Mode",
            default: "FALSE",
            options: [
                {
                    name: "FALSE",
                    displayName: "FALSE"
                },
                {
                    name: "TRUE",
                    displayName: "TRUE"
                },
            ],
            description: "This determines whether the Channel uses Buffer Mode",
        },
        {
            name: "useLuTable",
            displayName: "LUT Table",
            default: "TRUE",
            options: [
                {
                    name: "FALSE",
                    displayName: "FALSE"
                },
                {
                    name: "TRUE",
                    displayName: "TRUE"
                },
            ],
            description: "This determines whether the Channel uses LUT Table",
        },
     );

    return config;
}


function validate(inst, report) {
        common.validate.checkNumberRange(inst, report, "ChannelIndex", 1, 9, "dec");
}

let gpadc_ch_module_name = "/drivers/gpadc/v0/gpadc_v0_channel";

let gpadc_ch_module = {
    displayName: "GPADC Channel Configuration",
    defaultInstanceName: "CONFIG_GPADC_CH",
    config: getConfigurables(),
    validate : validate,
    getInstanceConfig,
};

exports = gpadc_ch_module;
