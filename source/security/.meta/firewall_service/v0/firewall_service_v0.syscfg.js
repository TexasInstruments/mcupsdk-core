
let common = system.getScript("/common");

function getConfigArr() {
    return system.getScript(`/security/firewall_service/soc/mpu_firewall_config_${common.getSocName()}`).getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let configArr = getConfigArr();
    let config = configArr.find( o => o.name === moduleInstance.instance);

    return {
        ...config,
        ...moduleInstance,
    };
};

let firewall_service_module = {
    displayName: "SET FIREWALL",
    defaultInstanceName: "SET_MPU_FIREWALL_REQUEST_",
    config: [
    ],
    moduleInstances: moduleInstances,
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    getInstanceConfig,
};

/*
 *  ======== moduleInstances ========
 */
function moduleInstances(instance) {
    let modInstances = new Array();
    modInstances.push({
        name: "mpu_region",
        displayName: "MPU Region Configuration",
        moduleName: '/security/firewall_service/v0/firewall_service_v0_region',
        collapsed: false,
        useArray: true,
        maxInstanceCount: 16,
        defaultInstanceCount: 0,
    });

    return (modInstances);
}

exports = firewall_service_module;
