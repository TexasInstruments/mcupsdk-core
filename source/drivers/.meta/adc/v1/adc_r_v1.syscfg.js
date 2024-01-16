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


function onValidate(inst, validation) {

    //
    // Synchronous Mode
    //

    if (inst.$module.$static["synchronousModeCheck"])
    {
        //
        // Run this once
        //
        let triggers_for_adc_socs = {};

        let runStaticValidation = false;
        if (inst.$module.$instances.length > 0)
        {
            if (inst == inst.$module.$instances[0])
            {
                runStaticValidation = true;
            }
        }

        if (runStaticValidation)
        {
            let synchronousModeErrorFound = false;
            for (let instance_index in inst.$module.$instances)
            {
                let instance_obj = inst.$module.$instances[instance_index];

                console.log(instance_obj)

                if (instance_obj["adcClockPrescaler"] != inst["adcClockPrescaler"])
                {
                    validation.logError(
                        "In synchronous mode all clock prescalers must match! " + inst.$name + " and " + instance_obj.$name +
                        " do not match!",
                        instance_obj, "adcClockPrescaler");
                    synchronousModeErrorFound = true;
                }
                if (instance_obj["socHighPriorityMode"] != inst["socHighPriorityMode"])
                {
                    validation.logError(
                        "In synchronous mode all ADC high priority modes must match! " + inst.$name + " and " + instance_obj.$name +
                        " do not match!",
                        instance_obj, "socHighPriorityMode");
                    synchronousModeErrorFound = true;
                }
                if (instance_obj["enableBurstMode"] != inst["enableBurstMode"])
                {
                    validation.logError(
                        "In synchronous mode all burst modes must match! " + inst.$name + " and " + instance_obj.$name +
                        " do not match!",
                        instance_obj, "enableBurstMode");
                    synchronousModeErrorFound = true;
                }
                if (instance_obj["enableBurstMode"] && inst["enableBurstMode"])
                {
                    if (instance_obj["burstSize"] != inst["burstSize"])
                    {
                        validation.logError(
                            "In synchronous mode all burst sizes must match! " + inst.$name + " and " + instance_obj.$name +
                            " do not match!",
                            instance_obj, "burstSize");
                        synchronousModeErrorFound = true;
                    }
                    if (instance_obj["burstTrigger"] != inst["burstTrigger"])
                    {
                        validation.logError(
                            "In synchronous mode all burst triggers must match! " + inst.$name + " and " + instance_obj.$name +
                            " do not match!",
                            instance_obj, "burstTrigger");
                        synchronousModeErrorFound = true;
                    }
                }

            }

            if (!synchronousModeErrorFound)
            {
                for (let soci = 0; soci < 16; soci++)
                {
                    let socTriggers = [];
                    let socTriggerOwners = [];
                    let socSampleWindows = [];
                    for (let instance_index in inst.$module.$instances)
                    {
                        let instance_obj = inst.$module.$instances[instance_index];
                        let highPri = instance_obj.socHighPriorityMode;
                        let number_highpri_socs = device_peripheral.ADC_PriorityMode.findIndex(x => x.name == highPri);

                        if (soci < number_highpri_socs)
                        {
                            socTriggers.push(instance_obj["soc" + soci.toString() + "Trigger"])
                            socTriggerOwners.push("soc" + soci.toString() + "Trigger");
                            socSampleWindows.push(instance_obj["soc" + soci.toString() + "SampleWindow"]);
                        }
                        else
                        {
                            if (instance_obj.enableBurstMode)
                            {
                                socTriggers.push(instance_obj["burstTrigger"])
                                socTriggerOwners.push("burstTrigger");
                                socSampleWindows.push(instance_obj["soc" + soci.toString() + "SampleWindow"]);
                            }
                            else
                            {
                                socTriggers.push(instance_obj["soc" + soci.toString() + "Trigger"])
                                socTriggerOwners.push("soc" + soci.toString() + "Trigger");
                                socSampleWindows.push(instance_obj["soc" + soci.toString() + "SampleWindow"]);
                            }
                        }
                    }

                    let maxWindow = Math.max.apply(null, socSampleWindows);
                    for (let instance_index in inst.$module.$instances)
                    {
                        let instance_obj = inst.$module.$instances[instance_index];
                        for (let instance_indexCompare in inst.$module.$instances)
                        {
                            let instance_objCompare = inst.$module.$instances[instance_index];
                            if (socTriggers[instance_index] != socTriggers[instance_indexCompare])
                            {
                                validation.logError(
                                    "In synchronous mode the triggers for the same SOC numbers must match across different ADCs! " + instance_objCompare.$name + " and " + instance_obj.$name +
                                    " do not match!",
                                    instance_obj, socTriggerOwners[instance_index]);
                            }
                            if (socSampleWindows[instance_index] != socSampleWindows[instance_indexCompare])
                            {
                                validation.logError(
                                    "In synchronous mode the sampling window of the same SOC numbers must match across different ADCs! " + instance_objCompare.$name + " and " + instance_obj.$name +
                                    " do not match!" + " The maximum value found is " + maxWindow.toString(),
                                    instance_obj, "soc" + soci.toString() + "SampleWindow");
                            }
                        }
                    }
                }
            }
        }
    }

    //
    // End Synchronous Mode
    //

    //
    // Check channel selection for each SOC
    //
    for (let soci = 0; soci < 16; soci++)
    {
        let channel = inst["soc" + soci.toString() + "Channel"]
        let channel_numbers = channel.replace("ADC_CH_", "")
        let channels = channel_numbers.split("_")
        if (inst["adcSignalMode"] == "ADC_MODE_SINGLE_ENDED")
        {
            if (channels.length > 1)
            {
                validation.logError(
                    "SOC" + soci.toString() + " cannot use a differential input channel" +
                    " when the ADC module is in single ended mode",
                    inst, "soc" + soci.toString() + "Channel");
            }
        }
        else
        {
            if (channels.length < 2)
            {
                validation.logError(
                    "SOC" + soci.toString() + " cannot use a single-ended input channel" +
                    " when the ADC module is in differential mode",
                    inst, "soc" + soci.toString() + "Channel");
            }
        }
    }

    //
    // Burst Trigger warnings
    //

    let highPri = inst.socHighPriorityMode;
    let number_highpri_socs = device_peripheral.ADC_PriorityMode.findIndex(x => x.name == highPri);
    for (let soci = 0; soci < 16; soci++)
    {
        if (soci < number_highpri_socs)
        {

        }
        else
        {
            if (inst.enableBurstMode)
            {
                if (inst.burstTrigger != inst["soc" + soci.toString() + "Trigger"])
                {
                    validation.logWarning(
                        "SOC" + soci.toString() + " is configured for Round Robin. With burst mode enabled, the " +
                        " burst mode trigger will override the selected trigger for this SOC",
                        inst, "soc" + soci.toString() + "Trigger");
                }
            }
        }
    }

    //
    // Sample Window
    //
    for (let soci = 0; soci < 16; soci++)
    {
        if (inst["soc" + soci.toString() + "SampleWindow"] < 16 ||
            inst["soc" + soci.toString() + "SampleWindow"] > 512)
        {
            validation.logError(
                "SOC" + soci.toString() + " sample window must be between 16 and 512",
                inst, "soc" + soci.toString() + "SampleWindow");
        }
    }

    //
    // Cycle Offset
    //
    if (inst["interruptPulseMode"] == "ADC_PULSE_END_OF_ACQ_WIN")
    {
        if (inst["interruptCycleOffset"] < 0 || inst["interruptCycleOffset"] > 0xFFFF)
        {
            validation.logError(
                "Interupt cycle offset must be between 0 and 0xFFFF",
                inst, "interruptCycleOffset");
        }
    }

    //
    // PPB Check
    //
    for (let ppbi = 1; ppbi <= 4; ppbi++)
    {
        if (inst["ppb" + ppbi.toString() + "CalibrationOffset"] < -512 ||
            inst["ppb" + ppbi.toString() + "CalibrationOffset"] > 511)
        {
            validation.logError(
                "The calibration offset value must be between -512 and 511",
                inst,"ppb" + ppbi.toString() +  "CalibrationOffset");
        }

        if (inst["ppb" + ppbi.toString() + "ReferenceOffset"] < 0 ||
            inst["ppb" + ppbi.toString() + "CalibrationOffset"] > 0xFFFF)
        {
            validation.logError(
                "The reference offset value must be between 0 and 0xFFFF",
                inst,"ppb" + ppbi.toString() +  "CalibrationOffset");
        }

        if (inst["ppb" + ppbi.toString() + "HighTripLimit"] > 65535 ||
            inst["ppb" + ppbi.toString() + "HighTripLimit"] < -65536)
        {
            validation.logError(
                "The high trip limit value must be between 65535 and -65536",
                inst,"ppb" + ppbi.toString() +  "HighTripLimit");
        }

        if (inst["ppb" + ppbi.toString() + "LowTripLimit"] > 65535 ||
            inst["ppb" + ppbi.toString() + "LowTripLimit"] < -65536)
        {
            validation.logError(
                "The low trip limit value must be between 65535 and -65536",
                inst,"ppb" + ppbi.toString() +  "LowTripLimit");
        }
    }
}

/* ------------------------------------------------------------------------- */

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

adcRModule.validate = onValidate;

for(let soci = 0; soci < 16 ; soci ++)
{
    let configName  = "soc" + soci.toString() + "Channel"

    for (let config1 of adcRModule.config)
    {
        if (config1.name == "GROUP_SOC")
        {
            for (let config2 of config1.config)
            {
                if (config2.name == "GROUP_SOC"+soci.toString())
                {
                    for (let config3 of config2.config)
                    {
                        if (config3.name == configName)
                        {
                            config3.options = device_peripheral.ADC_R_Channel;
                            config3.default = device_peripheral.ADC_R_Channel[0].name;
                        }
                    }
                }
            }
        }
    }
}

exports = adcRModule;