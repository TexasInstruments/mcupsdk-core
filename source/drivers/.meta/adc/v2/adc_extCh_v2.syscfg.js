let common   = system.getScript("/common");
let pinmux   = system.getScript("/drivers/pinmux/pinmux");

function getInstanceConfig(moduleInstance) {

}


/*
 *  ======== filterHardware ========
 *  Control RX, TX Pin usage by the user specified dataDirection.
 *
 *  param component - hardware object describing signals and
 *                     resources they're attached to
 *
 *  returns Boolean indicating whether or not to allow the component to
 *           be assigned to an instance's $hardware config
 */
function filterHardware(component)
{
    return (common.typeMatches(component.type, ["ADC"]));
}

function getInterfaceName(inst)
{
    return "ADC_EXTCH_XBAR";
}

function getPeripheralPinNames(inst)
{
    return ["CHANNEL0","CHANNEL1","CHANNEL2","CHANNEL3","CHANNEL4","CHANNEL5","CHANNEL6","CHANNEL7","CHANNEL8","CHANNEL9"];

}

function pinmuxRequirements(inst)
{
   let interfaceName = getInterfaceName(inst);

    let resources = [];

    for(let channel = 0; channel < 10; channel++)
    {
        resources.push( pinmux.getPinRequirements(interfaceName, "CHANNEL"+channel.toString(), "ADC Ext Channel select " +channel.toString()+ " Pin"));
    }

    let peripheral = {
        name: interfaceName,
        displayName: "ADC Ext Channel Select Xbar Instance",
        interfaceName: interfaceName,
        resources: resources,
    };

    return [peripheral];
}

let adcExtChModule = {
    peripheralName: "ADC_EXTCH_XBAR",
    displayName: "ADC_EXTCH_XBAR",
    defaultInstanceName: "CONFIG_ADC_EXTCH_XBAR",
    description: "ADC External Channel select XBAR",
    longDescription: "Use SOC APIs like SOC_selectAdcExtChXbar for configuring the ADC_EXTCH_SEL xbar to use specific ADC SOC-ExtChSel bits",
    filterHardware : filterHardware,
    config: [],
    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: "/drivers/adc/adc_extCh",
        },
        // "/drivers/system/system_config.h.xdt": {
        //     driver_config: "/drivers/adc/templates/adc.h.xdt",
        // },
        // "/drivers/system/system_config.c.xdt": {
        //     driver_init: "/drivers/adc/templates/adc_init.c.xdt",
        // },
        // "/drivers/system/drivers_open_close.h.xdt": {
        //     driver_open_close_config: "/drivers/adc/templates/adc_open_close_config.h.xdt",
        // },
        // "/drivers/system/drivers_open_close.c.xdt": {
        //     driver_open_close_config: "/drivers/adc/templates/adc_open_close_config.c.xdt",
        //     driver_open: "/drivers/adc/templates/adc_open.c.xdt",
        // },
    },
    // getInstanceConfig,
    getInterfaceName,
    getPeripheralPinNames,
    pinmuxRequirements,
};

exports = adcExtChModule;