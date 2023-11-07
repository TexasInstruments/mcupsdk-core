let common   = system.getScript("/common");
let pinmux   = system.getScript("/drivers/pinmux/pinmux");
let device_peripheral = system.getScript(`/drivers/adc/soc/adc_${common.getSocName()}.syscfg.js`);
let adc_sampletime_sysclk_ns = 1000/device_peripheral.ADC_Sysclk_Mhz;

let module = system.getScript("adc_v1");

function getStaticConfigArr() {
    return system.getScript(`/drivers/adc/soc/adc_${common.getSocName()}`).getStaticConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let solution = moduleInstance[getInterfaceName(moduleInstance)].$solution;
    let staticConfigArr = getStaticConfigArr();
    let staticConfig = staticConfigArr.find( o => o.name === solution.peripheralName);

    return {
        ...staticConfig,
        ...moduleInstance
    }
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
    return (common.typeMatches(component.type, ["ADC_R"]));
}

function pinmuxRequirements(inst)
{
   let interfaceName = getInterfaceName(inst);

    let resources = [];
    resources.push( pinmux.getPinRequirements(interfaceName, "AIN0", "ADC Input Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "AIN1", "ADC Input Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "AIN2", "ADC Input Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "AIN3", "ADC Input Pin"));

    let peripheral = {
        name: interfaceName,
        displayName: interfaceName + " Instance",
        interfaceName: interfaceName,
        resources: resources,
    };

    return [peripheral];
}


function getInterfaceName(inst)
{
    return device_peripheral.getInterfaceNameAdcR(inst);
}

function getPeripheralPinNames(inst)
{
    let interfaceName = getInterfaceName(inst);
    return [ "AIN0", "AIN1", "AIN2", "AIN3" ];

}
let adcRModule = JSON.parse(JSON.stringify(module));

adcRModule.peripheralName = "ADC_R";
adcRModule.displayName = "ADC_R";
adcRModule.defaultInstanceName = "CONFIG_ADC_R";
adcRModule.getInstanceConfig = getInstanceConfig;
adcRModule.getInterfaceName = getInterfaceName;
adcRModule.getPeripheralPinNames = getPeripheralPinNames;
adcRModule.pinmuxRequirements = pinmuxRequirements;
adcRModule.templates = {
            "/drivers/system/system_config.h.xdt": {
                driver_config: "/drivers/adc/templates/adc_r.h.xdt",
            },
            "/drivers/system/system_config.c.xdt": {
                driver_init: "/drivers/adc/templates/adc_r_init.c.xdt",
            },
            "/drivers/system/drivers_open_close.h.xdt": {
                driver_open_close_config: "/drivers/adc/templates/adc_r_open_close_config.h.xdt",
            },
            "/drivers/system/drivers_open_close.c.xdt": {
                driver_open_close_config: "/drivers/adc/templates/adc_r_open_close_config.c.xdt",
                driver_open: "/drivers/adc/templates/adc_r_open.c.xdt",
            },
        };

exports = adcRModule;