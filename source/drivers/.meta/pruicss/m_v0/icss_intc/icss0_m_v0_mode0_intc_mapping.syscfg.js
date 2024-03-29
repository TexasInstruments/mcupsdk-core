let common = system.getScript("/common");
let soc = system.getScript(`/drivers/pruicss/soc/icss_intc_${common.getSocName()}`);

function getInstanceConfig(moduleInstance) {
    return {
        ...moduleInstance
    }
}

function onValidate(inst, validation) {
    let channelHostMap = new Map();
    let events = new Array();
    for (let instance_index in inst.$module.$instances)
    {
        let instance = inst.$module.$instances[instance_index];

        /* event can only be selected once */
        if (events.includes(instance.event))
            validation.logError(
                "The same event number/signal can't be remapped",
                inst, "event");
                /* TODO: Will there be a case that we want to use same event number from 2 PRU cores */
        else
            events.push(instance.event);

        if (!channelHostMap.has(instance.channel))
            channelHostMap.set(instance.channel, instance.host);
        else if (channelHostMap.get(instance.channel) !== instance.host)
            validation.logError(
                "The same channel can't be mapped to multiple host events." +
                `Other existing mapping: Channel:${instance.channel} to Host Event:${channelHostMap.get(instance.channel)}`,
                inst, "host");
    }
}

let icss_intc_mapping_top_module_name = "/drivers/pruicss/m_v0/icss_intc/icss0_m_v0_mode0_intc_mapping";

let icss_intc_mapping_top_module = {
    displayName: "ICSSM0 INTC MODE0 Signals Mapping",

    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/pruicss/m_v0/icss_intc/templates/icss_intc.h.xdt",
            moduleName: icss_intc_mapping_top_module_name,
        },
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/pruicss/m_v0/icss_intc/templates/icss_intc_config.c.xdt",
            moduleName: icss_intc_mapping_top_module_name,
        },
    },

    defaultInstanceName: "CONFIG_ICSS0_INTC_MODE0_MAPPING",
    config: [
        {
            name: "icssInstance",
            displayName: "ICSSM Instance",
            default: "ICSSM0",
            readOnly: true,
        },
        {
            name: "INTC MODE",
            displayName: "INTC MODE",
            default:"ICSSM0_MII_RT_EVENT_DISABLE",
            readOnly: true,
        },
        {
            name: "event",
            displayName: "INTC Event Signal",
            default: "16",
            options: soc.getEventConfigOptions("ICSSM0_MODE0"),
            hidden:false,
            getDisabledOptions: (inst) => JSON.parse(inst.eventDisabledOptions),
        },
        {
            name: "channel",
            displayName: "INTC Channel",
            default: "2",
            options: soc.getChannelConfigOptions(),
            getDisabledOptions: (inst) => JSON.parse(inst.channelDisabledOptions),
            onChange: (inst) => inst.host = inst.channel,
        },
        {
            name: "host",
            displayName: "INTC Host Interrupt",
            default: "2",
            options: soc.getHostConfigOptions(),
            getDisabledOptions: (inst) => JSON.parse(inst.hostDisabledOptions),
        },
        {
            name: "sysEventPolarity",
            displayName: "System Event Polarity",
            default: "HIGH",
            options: [
                {
                    name: "HIGH",
                },
                {
                    name: "LOW",
                }
            ],
        },
        {
            name: "sysEventType",
            displayName: "System Event Type",
            default: "PULSE",
            options: [
                {
                    name: "PULSE",
                },
                {
                    name: "EDGE",
                }
            ],
        },
        {
            name: "eventDisabledOptions",
            default: `[]`,
            hidden: true,
        },
        {
            name: "channelDisabledOptions",
            default: `[]`,
            hidden: true,
        },
        {
            name: "hostDisabledOptions",
            default: `[]`,
            hidden: true,
        },
    ],
    getInstanceConfig,
    validate: onValidate,
};


exports = icss_intc_mapping_top_module;
