
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/edma/soc/edma_${common.getSocName()}`);

function getInstanceConfig(moduleInstance) {
    return {
        ...moduleInstance,
    };
};

var config = [];

let CHANNEL_TRIGGER_LIST = soc.getOptionList("INTERNAL");
let CHANNEL_TRIGGERS = JSON.parse(JSON.stringify(CHANNEL_TRIGGER_LIST));
CHANNEL_TRIGGERS.map(function(item) {
    delete item.path;
    return item;
});

var channelTrigger = {
    name        : "channelTrigger",
    displayName : "Channel Trigger",
    description : 'This determines the hardware trigger for a channel',
    hidden      : false,
    default     : CHANNEL_TRIGGERS[0].name,
    options     : CHANNEL_TRIGGERS,
}

config.push(channelTrigger);

let edma_ch_module = {
    displayName: "EDMA Channel Configuration",
    longDescription: `This adds and configures a DMA channel.`,
    alwaysShowLongDescription: true,
    defaultInstanceName: "CONFIG_EDMA_CH",
    config: config,
    moduleInstances: moduleInstances,
    getInstanceConfig,
};

function moduleInstances(instance) {
    return soc.supportXbarConfig(CHANNEL_TRIGGER_LIST.find(o => o.name == instance.channelTrigger), instance);
}

exports = edma_ch_module;
