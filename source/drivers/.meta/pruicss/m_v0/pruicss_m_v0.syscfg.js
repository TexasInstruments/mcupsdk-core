let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/pruicss/soc/pruicss_${common.getSocName()}`);

function getConfigArr() {
    return soc.getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let configArr = getConfigArr();
    let config = configArr.find(o => o.name === moduleInstance.instance);

    return {
        ...config,
        ...moduleInstance,
    };
};

function getMdioBaseAddr(pruicssInstance)
{
    let configArr = getConfigArr();
    let config = configArr.find(o => o.name === pruicssInstance);

    return config.mdioBaseAddr;
}

function getConfigurables()
{
    let device = common.getDeviceName();
    let config=new Array();
    config.push(
        {
            name: "instance",
            displayName: "Instance",
            default: "ICSSM0",
            options: [
                {
                    name: "ICSSM0",
                },
            ],
        },
    )
    if(device==="am263x-cc" || device==="am263-lp" || device==="am263px-cc" || device==="am263px-lp"){
        config.push(
            {
                name: "INTC MODE",
                displayName: "INTC MODE",
                default:"mode1",
                options: [
                    {
                        name:"mode1",
                        displayName: "ICSSM0_MII_RT_EVENT_ENABLE",
                        description:'In this mode MII_RT_EVENTS are enabled PRU-ICSS Interrupt Controller lines 32 through 55 are mapped to internal events'
                    },
                    {
                        name:"mode0",
                        displayName: "ICSSM0_MII_RT_EVENT_DISABLE",
                        description:"In this mode MII_RT_EVENTS are NOT enabled PRU-ICSS Interrupt Controller lines 32 through 55 are mapped to external events"
                    },
                ],
            },
        )
    }
    return config
}

let pruicss_top_module_name = "/drivers/pruicss/pruicss";

let pruicss_top_module = {
    displayName: "PRU (ICSS)",

    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/pruicss/templates/pruicss_config.c.xdt",
            driver_init: "/drivers/pruicss/templates/pruicss_init.c.xdt",
            driver_deinit: "/drivers/pruicss/templates/pruicss_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/pruicss/templates/pruicss.h.xdt",
        },
    },

    defaultInstanceName: "CONFIG_PRU_ICSS",
    config: getConfigurables(),
    moduleInstances: moduleInstances,
    validate: validate,
    getInstanceConfig,
    getMdioBaseAddr,
};
function moduleInstances(instance) {
    let device = common.getDeviceName();
    let modInstances = new Array();
    if(device==="am263x-cc" || device==="am263-lp" || device==="am263px-cc" || device==="am263px-lp"){
        modInstances.push({
            name: "AdditionalICSSSettings",
            displayName: "Additional ICSS Settings",
            moduleName: '/drivers/pruicss/m_v0/pruicss_m_v0_gpio',
            useArray: true,
            minInstanceCount: 1,
            defaultInstanceCount: 1,
            maxInstanceCount: 1,
        });
        // Interrupt Mapping:
        let submodule = "/drivers/pruicss/m_v0/icss_intc/";
        if(instance["INTC MODE"] === "mode1")
        submodule += "icss0_m_v0_mode1_intc_mapping";
        else if(instance["INTC MODE"] === "mode0")
        submodule += "icss0_m_v0_mode0_intc_mapping";
        else
        submodule += "icss0_m_v0_mode1_intc_mapping";
        modInstances.push({
            name: "intcMapping",
            displayName: instance.instance + " INTC Internal Signals Mapping",
            moduleName: submodule,
            useArray: true,
            defaultInstanceCount: 0,
        });
        }
        return (modInstances);
}
function validate(inst, report) {
    common.validate.checkSameInstanceName(inst, report);
}

exports = pruicss_top_module;
