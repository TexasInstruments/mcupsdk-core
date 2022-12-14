let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/firewall/soc/firewall_${common.getSocName()}`);

const firewallIdList = [ { name: 'SA2UL' },
                         { name: 'PSRAMECC0' },
                         { name: 'MCU_PSC0' },
                         { name: 'DDR16SS0' },
                         { name: 'MSRAM_256K0' },
                         { name: 'MSRAM_256K1' },
                         { name: 'MSRAM_256K2' },
                         { name: 'MSRAM_256K5' },
                         { name: 'MSRAM_256K4' },
                         { name: 'MSRAM_256K3' },
                         { name: 'DMASS0' },
                         { name: 'GICSS0' },
                         { name: 'MSRAM_256K6' },
                         { name: 'MSRAM_256K7' }
                       ];

function getConfigArr() {
    return soc.getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let configArr = soc.getFirewallConfig();
    let config = configArr.find( o => o.fwl_name === moduleInstance.fwlId);
    return {
        ...config,
        ...moduleInstance,
    };
};

function getInterfaceName(inst) {

    let interfaceName = "FIREWALL";
    return interfaceName;
}

function getMasterConfigArr(){
    return soc.getMasterConfigArr();
}

function getFirewallMaxRegions(inst){
    let fwlId = inst.fwlId;
    if(!fwlId) return 0
    let fwlArr = soc.getFirewallConfig();
    let config = fwlArr.find( o => o.fwl_name === fwlId);
    return config.fwl_regions;
}


function getConfigurables() {
    let config = [];
    config.push(
    {
        name: "fwlId",
        displayName: "Firewall ID",
        default: firewallIdList[0].name,
        description: "The Target name of the Firewall region",
        options: firewallIdList,
    }
    )
    return config;
}

let firewall_module_name = "/drivers/firewall/firewall";

let firewall_module = {
    displayName: "FIREWALL",
    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/firewall/templates/firewall_config.c.xdt",
            driver_init: "/drivers/firewall/templates/firewall_init.c.xdt",
            driver_deinit: "/drivers/firewall/templates/firewall_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/firewall/templates/firewall.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/firewall/templates/firewall_open_close_config.c.xdt",
            driver_open: "/drivers/firewall/templates/firewall_open.c.xdt",
            driver_close: "/drivers/firewall/templates/firewall_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/firewall/templates/firewall_open_close.h.xdt",
        },
    },
    defaultInstanceName: "CONFIG_FIREWALL",
    config: getConfigurables(),
    modules: function (inst) {
        return [{
            name: "system_common",
            moduleName: "/system_common",
        }]
    },
    validate: validate,
    maxInstances: soc.getFirewallConfig().length,
    moduleInstances: regionCfg,
    getInstanceConfig,
    getInterfaceName,
};

function regionCfg(instance) {
    let modInstances = new Array();

    modInstances.push({
        name: "regCfg",
        displayName: "Region Configuration",
        moduleName: "/drivers/firewall/v0/firewallRegionCfg",
        useArray: true,
        minInstanceCount: 0,
        defaultInstanceCount: 0,
        maxInstanceCount:getFirewallMaxRegions(instance),
    });

    return modInstances;
}

function validate(inst, report) {
    common.validate.checkSameFieldName(inst,"fwlId",report);
}


exports = firewall_module;
