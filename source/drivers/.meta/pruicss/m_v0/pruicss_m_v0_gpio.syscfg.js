
let common = system.getScript("/common");

let pruicss_top_module_name = "/drivers/pruicss/m_v0/pruicss_m_v0_gpio";

let pruicss_top_module = {
    displayName: "PRU (ICSS) IO Settings",

    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: pruicss_top_module_name,
        },
    },

    defaultInstanceName: "CONFIG_PRU_ICSS_IO",
    moduleInstances,
};

function validate(inst, report) {
    common.validate.checkSameInstanceName(inst, report);
}

function moduleInstances(instance) {
    let device = common.getDeviceName();
    let modInstances = new Array();
    if(device==="am263x-cc" || device==="am263-lp" || device==="am263px-cc" || device==="am263px-lp"){
        modInstances.push({
            name: "PruGPIO",
            displayName: "PRU (ICSS) GPIO",
            moduleName: '/drivers/pruicss/m_v0/pruicss_m_v0_gpio_gp',
            useArray: true,
            minInstanceCount: 0,
            defaultInstanceCount: 0,
        });
        modInstances.push({
            name: "PruIepIO",
            displayName: "PRU (ICSS) IEP",
            moduleName: '/drivers/pruicss/m_v0/pruicss_m_v0_gpio_iep',
            useArray: true,
            minInstanceCount: 0,
            defaultInstanceCount: 0,
        });
        modInstances.push({
            name: "PruUartIO",
            displayName: "PRU (ICSS) UART",
            moduleName: '/drivers/pruicss/m_v0/pruicss_m_v0_gpio_uart',
            useArray: true,
            minInstanceCount: 0,
            defaultInstanceCount: 0,
        });
    }
    return (modInstances);
}

exports = pruicss_top_module;
