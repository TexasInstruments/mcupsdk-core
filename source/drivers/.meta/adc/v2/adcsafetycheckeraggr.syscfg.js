let common   = system.getScript("/common");
let pinmux   = system.getScript("/drivers/pinmux/pinmux");
let device_peripheral = system.getScript(`/drivers/adc/soc/adc_${common.getSocName()}.syscfg.js`);

let module = system.getScript("adc_v2");

var ADC_SFTYCH_SOURCE =[];

for (var sftychkr in device_peripheral.ADC_Checker)
{
    for (var chkrslt in device_peripheral.ADC_SafetyCheckResult)
    {
        //var currentchkrslt= device_driverlib_memmap.ADCSAFETYCHKMemoryMap[sftychkr].name.replace("_BASE","").replace("ADC","")
        //var currentsftychkr = device_peripheral.ADC_SafetyCheckResult[chkrslt].name.replace("ADC_SAFETY_CHECK_","")

        ADC_SFTYCH_SOURCE.push(
            {name: device_peripheral.ADC_Checker[sftychkr].name.replace("ADC_","") + device_peripheral.ADC_SafetyCheckResult[chkrslt].name.replace("ADC_SAFETY_CHECK_",""),
            displayName: device_peripheral.ADC_Checker[sftychkr].name.replace("ADC_","") + " "+ device_peripheral.ADC_SafetyCheckResult[chkrslt].name.replace("ADC_SAFETY_CHECK_","")
        },
        )
    }

}

function getInterfaceName(inst)
{
    return device_peripheral.getInterfaceNameAdcSCTILE(inst);
}
function getStaticConfigArr() {
    return system.getScript(`/drivers/adc/soc/adc_${common.getSocName()}`).getStaticConfigArr();
}
function getInstanceConfig(moduleInstance) {
    let solution = moduleInstance["adcIntEvtBase"];
    let staticConfigArr = getStaticConfigArr();
    let staticConfig = staticConfigArr.find( o => o.name === solution);

    return {
        ...staticConfig,
        ...moduleInstance
    }
}

function onChangeSafetyCheckerEVNTs(inst, ui){
    for(var evntIndex in device_peripheral.ADC_SafetyCheckEvent)
    {
        var currentEvnt = device_peripheral.ADC_SafetyCheckEvent[evntIndex].name
        var evnti = (currentEvnt).replace(/[^0-9]/g,'')
        var sftyEVNTConfigs =
            [
                "chkEvent" + evnti.toString() + "Source",
            ]

        if (ui)
        {
            for (var sftyConfig of sftyEVNTConfigs)
            {
                if(inst["enableEvent"].includes(currentEvnt))
                {
                    ui[sftyConfig].hidden = false;
                }
                else
                {
                    ui[sftyConfig].hidden = true;
                }
            }
        }
    }
}
function onChangeSafetyCheckerINTs(inst, ui) {
    if (ui) {
        if((inst.enableInterrupt)) {
            ui["eventInterruptSource"].hidden = false;
        }
        else {
            ui["eventInterruptSource"].hidden = true;
        }
    }
}

var config = [
    {
        name: "$name",
        hidden: false,
        isCIdentifier: false
    },
    {
        name        : "adcIntEvtBase",
        displayName : "ADC INTEVT Instance",
        description : 'Instance of the ADC used.',
        hidden      : false,
        default     : device_peripheral.ADC_SafetyAggr_Instances[0].name,
        options     : device_peripheral.ADC_SafetyAggr_Instances,
    },
    {
        name            : "enableInterrupt",
        displayName     : "Enable Interrupt",
        description     : 'Each ADC result safety checker tile can generate an interrupt signal from out-of-tolerance (OOT) flag and result overflow flags OVF1 and OVF2',
        longDescription : 'Each ADC result safety checker tile generates an interrupt pulse when an enabled out-of-tolerance (OOT) or overflow OVF1 or OVF2 event occurs. No further interrupt pulses will be generated until the interrupt flag is cleared.',
        hidden          : false,
        default         : false,
        onChange        : onChangeSafetyCheckerINTs,
    },
    {
        name            : "eventInterruptSource",
        displayName     : "Select Source For Interrupt",
        description     : 'Each ADC result safety checker tile can generate an interrupt signal from out-of-tolerance (OOT) flag and result overflow flags OVF1 and OVF2',
        hidden          : true,
        minSelections   : 0,
        default         : [],
        options         : ADC_SFTYCH_SOURCE,
        onChange        : onChangeSafetyCheckerINTs,
    },
    {
        name            : "enableEvent",
        displayName     : "Enable Event",
        description     : 'Safety checker tiles can generate events that can be sent to the X-BAR, so that automatic hardware actions such as an ePWM trip can be generated',
        longDescription : '',
        hidden          : false,
        minSelections   : 0,
        default         : [],
        options         :device_peripheral.ADC_SafetyCheckEvent,
        onChange        : onChangeSafetyCheckerEVNTs,
    },

];

for(var evntIndex in device_peripheral.ADC_SafetyCheckEvent)
    {
        var currentEvnt = device_peripheral.ADC_SafetyCheckEvent[evntIndex].name
        var evnti = (currentEvnt).replace(/[^0-9]/g,'')
        var sfty_Configs = [
        {
            name: "chkEvent" + evnti.toString() + "Source",
            displayName : "Select Source",
            description : 'Select Source for this event',
            hidden      : true,
            minSelections   : 0,
            default         : [],
            options     : ADC_SFTYCH_SOURCE,
        },

    ]
    config= config.concat ([{
        name: "GROUP_EVENT" +evnti.toString(),
        displayName: "Safety Check Event " + evnti.toString(),
        description: "",
        longDescription: "",
        config: sfty_Configs
    }])
    }

function onValidate(inst, validation) {
    //
    // Check duplicate sources
    //
    var usedADCSCInsts = [];
    for (var instance_index in inst.$module.$instances)
    {
        var instance_obj = inst.$module.$instances[instance_index];
        usedADCSCInsts.push(instance_obj.adcIntEvtBase);
    }

    var otherContexts = common.getOtherContextNames()
    for (var cntx of otherContexts)
    {
        var onOtherCntx = common.getModuleForCore(inst.$module.$name, cntx);
        if (onOtherCntx)
        {
            for (var instance_index in onOtherCntx.$instances)
            {
                var instance_obj = onOtherCntx.$instances[instance_index];
                usedADCSCInsts.push(instance_obj.adcIntEvtBase);
            }
        }
    }
    var duplicatesResult = common.findDuplicates(usedADCSCInsts)

    if (duplicatesResult.duplicates.length != 0)
    {
        var allDuplicates = "";
        for (var duplicateNamesIndex in duplicatesResult.duplicates)
        {
            allDuplicates = allDuplicates + common.stringOrEmpty(allDuplicates, ", ")
                            + duplicatesResult.duplicates[duplicateNamesIndex];
        }
        validation.logError(
            "The ADC Safety Checker Aggregator Instance used. Duplicates: " + allDuplicates,
            inst, "adcIntEvtBase");
    }
}

let maxInstances = device_peripheral.ADC_SafetyAggr_Instances.length

var adcSCSubmodule = {
    displayName: "ADC SAFETY CHECKER INTEVT",
    defaultInstanceName: "CONFIG_ADCSC_AGGR",
    description: "ADC Safety Checker Interrupt Event Aggregator",
    maxInstances: maxInstances,
    config: config,
    validate: onValidate,
    templates: {
        //boardc : "",
        //boardh : ""
    },
    getInstanceConfig,

};

exports = adcSCSubmodule;
