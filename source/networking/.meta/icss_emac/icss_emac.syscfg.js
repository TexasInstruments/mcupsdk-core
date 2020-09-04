
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/networking/icss_emac/icss_emac_${common.getSocName()}`);

let iss_emac_module_name = "/networking/icss_emac/icss_emac";

function getInstanceConfig(inst)
{
    let config = soc.getStaticConfig();

    soc.updateConfig(inst, config);

    return {
        ...config,
        ...inst,
    };
}

function getPktBufSizes(module)
{
    return soc.getPktBufSizes(module);
}

function checkSameFieldName(instance, iccsInstance, fieldname, report)
{
    let moduleInstances = instance.$module.$instances;

    for (let i = 0; i < moduleInstances.length; i++) {
        if (instance[fieldname] === moduleInstances[i][fieldname] &&
            instance !== moduleInstances[i] && instance.instance === iccsInstance) {
            report.logError(`Same ${fieldname} cannot be selected`, instance, fieldname);
        }
    }
}

function validate(instance, report) {
    common.validate.checkNumberRange(instance, report, "phyAddr0", 0, 31);
    common.validate.checkNumberRange(instance, report, "phyAddr1", 0, 31);
    common.validate.checkNumberRange(instance, report, "pktBufSizeKB", 16, 1024);
    common.validate.checkNumberRange(instance, report, "linkTaskPriority", 0, 31);
    common.validate.checkNumberRange(instance, report, "rxTaskPriority", 0, 31);
    common.validate.checkNumberRange(instance, report, "txTaskPriority", 0, 31);
}

function getConfigurables()
{
    let config = [];
    let icssConfig = soc.getIcssInstancesArr();
    let phyToMacInterfaceConfig = soc.getPhyToMacInterfacesArr();

    config.push(
        common.ui.makeConfig(icssConfig, "instance", "ICSS Instance"),
        {
            name: "mode",
            displayName: "EMAC Mode",
            default: "SWITCH",
            options: [
                {
                    name: "SWITCH",
                    displayName: "Switch",
                },
                {
                    name: "DUAL MAC | MAC1",
                    displayName: "MAC1 of DUAL EMAC",
                },
                {
                    name: "DUAL MAC | MAC2",
                    displayName: "MAC2 of DUAL EMAC",
                },
                {
                    name: "MAC1",
                    displayName: "MAC1",
                },
                {
                    name: "MAC2",
                    displayName: "MAC2",
                },
            ],
        },
        {
            name: "phyAddr0",
            description: "Phy Address of the port in single/dual EMAC mode or Port 0 in Switch mode. Value MUST be between 0 .. 31",
            displayName: "Phy Address 0",
            default: 15,
        },
        {
            name: "phyAddr1",
            description: "Phy Address of Port 1 in Switch mode. Value MUST be between 0 .. 31",
            displayName: "Phy Address 1",
            default: 15,
        },
        common.ui.makeConfig(phyToMacInterfaceConfig, "phyToMacInterfaceMode", "MII/RGMII"),
        {
            name: "queue",
            displayName: "RT/NRT Priority Separation Queue",
            description: "If packets are in Queue <= RT/NRT Priority separation queue, they will be forwarded to RT callback and others to NRT callback.",
            default: "QUEUE3",
            options: [
                { name: "QUEUE1", },
                { name: "QUEUE2", },
                { name: "QUEUE3", },
                { name: "QUEUE4", },
            ],
        },
        {
            name: "pktBufSizeKB",
            displayName: "Queue Buffer Size (KB)",
            description: "Value MUST be between 16 KB .. 1024 KB",
            default: 64,
        },
        {
            name: "linkTaskPriority",
            displayName: "Link Task Priority",
            description: "Value MUST be between 0 (lowest priority) .. 31 (highest priority)",
            default: 28,
        },
        {
            name: "rxTaskPriority",
            displayName: "RX Task Priority",
            description: "Value MUST be between 0 (lowest priority) .. 31 (highest priority)",
            default: 26,
        },
        {
            name: "txInterruptEnable",
            displayName: "TX Interrupt Enable",
            default: false,
        },
        {
            name: "txTaskPriority",
            displayName: "TX Task Priority",
            description: "Valid only if TX Interrupt Enable is checked. Value MUST be between 0 (lowest priority) .. 31 (highest priority)",
            default: 26,
        },
        {
            name: "halfDuplexEnable",
            displayName: "Half Duplex Enable",
            description: "Flag to enable Half duplex capability. Firmware support is also required.",
            default: false,
        },
        {
            name: "enableIntrPacing",
            displayName: "Interrupt Pacing Enable",
            default: false,
        },
        {
            name: "intrPacingMode",
            displayName: "Interrupt Pacing Mode",
            description: "Valid only if Interrupt Pacing Enable is checked",
            default: "MODE1",
            options: [
                {
                    name: "MODE1",
                    displayName : "Frame count based",
                },
            ],
        },
        {
            name: "pacingThreshold",
            displayName: "Interrupt Pacing Threshold Value",
            description: "Valid only if Interrupt Pacing Enable is checked",
            default: 100,
        },
        {
            name: "learningEnable",
            displayName: "Learning Enable",
            description: "Valid only for switch mode",
            default: false,
        },
        {
            name: "splitQueue",
            displayName: "Split Queue",
            longDescription: "Flag to enable Split Queue capability. If enabled, then out of the four queues, two queues are dedicated to each receiving port. ICSS_EMAC_QUEUE1 and ICSS_EMAC_QUEUE2 for Port 0 and ICSS_EMAC_QUEUE3 and ICSS_EMAC_QUEUE4 for Port 1.",
            default: false,
        },
    );

    return config;
}

let icss_emac_module = {

    displayName: "ICSS-EMAC",
	longDescription: "Driver for ICSS-EMAC which provide APIs for end point and switch that has been implemented on PRU-ICSS cores at at 100 Mbps.",
    defaultInstanceName: "CONFIG_ICSS_EMAC",
    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/networking/icss_emac/templates/icss_emac_config.c.xdt",
            driver_init: "/networking/icss_emac/templates/icss_emac_init.c.xdt",
            driver_deinit: "/networking/icss_emac/templates/icss_emac_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/networking/icss_emac/templates/icss_emac.h.xdt",
        },
    },
    config: getConfigurables(),
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    getInstanceConfig,
    getPktBufSizes,
    validate: validate,
};

exports = icss_emac_module;