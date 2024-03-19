let common   = system.getScript("/common");
let pinmux   = system.getScript("/drivers/pinmux/pinmux");
let device_peripheral = system.getScript(`/drivers/adc/soc/adc_${common.getSocName()}.syscfg.js`);

var ADCSC_INSTANCE =[];

for(var sftychkr in device_peripheral.ADC_Checker) {
    ADCSC_INSTANCE.push(
        { name: device_peripheral.ADC_Checker[sftychkr].name,
        displayName: device_peripheral.ADC_Checker[sftychkr].name.replace("ADC_","")
        },
    )
}

function getInterfaceName(inst)
{
    return device_peripheral.getInterfaceNameAdcSC(inst);
}
function getStaticConfigArr() {
    return system.getScript(`/drivers/adc/soc/adc_${common.getSocName()}`).getStaticConfigArr();
}
function getInstanceConfig(moduleInstance) {
    let solution = moduleInstance["adcscBase"];
    let staticConfigArr = getStaticConfigArr();
    let staticConfig = staticConfigArr.find( o => o.name === solution);

    return {
        ...staticConfig,
        ...moduleInstance
    }
}

var maxInstances = ADCSC_INSTANCE.length


var longDescriptionsafetychecker = `
For safety-critical applications, this device provides the ability to automatically compare ADC conversion results from multiple ADC modules against each other for consistency.
Each ADC checker tile captures conversion results from the associated ADCs
as soon as the conversions are complete, and compares the absolute value of the difference to the configured
tolerance. If the computed delta is out of range, the checker can generate a trip event signal that is sent to
an ePWM or output crossbar, and can also trigger an CPU interrupt. The structure and
operation of an ADC result safety checker tile are demonstrated in the figure:
![](../source/drivers/.meta/adc/images/safetychecker.png)

To configure an ADC result safety checker tile:
1) Select the first ADC instance to test. Then, in the Input field, select the source of the result, which can be any one of the SOCs, PPB modules or PPB sums. The result value should be given in the Selected Result field.
2) Select the second ADC instance to test. Then, in the Input field, select the source of the result, which can be any one of the SOCs, PPB modules or PPB sums. The result value should be given in the Selected Result field.
3) Configure the checker tolerance by assigning the value to Tolerance field.
4) Enable the checker tile by selecting Enable Safety Checker
`
var longDescriptionsafetycheckertolerance = `
Each ADC result safety checker tile can be configured to automatically compare two ADC conversion results
against a set tolerance value, and generate an interrupt if an out-of-tolerance (OOT) event occurs. The selected
results can be from the same ADC instance, or from different ADCs.
`
/* Array of ADC configurables that are common across device families */
let config = [
    {
        name: "$name",
        hidden: false,
        isCIdentifier: false
    },
    {
        name        : "adcscBase",
        displayName : "ADC Safety Checker Instance",
        longDescription : longDescriptionsafetychecker,
        hidden      : false,
        default     : ADCSC_INSTANCE[0].name,
        options     : ADCSC_INSTANCE,

    },
    {
        name            : "safetyCheckTolerance",
        displayName     : "Tolerance [LSB]",
        description     : 'Select tolerance in LSB for the difference between check result instancese',
        longDescription : longDescriptionsafetycheckertolerance,
        hidden          : false,
        default         : 0,
    },
];




function onChangesafetychecker(inst, ui)
{
    for(var rptrIndex in device_peripheral.ADC_SafetyCheckInst)
    {
        var currentRPTR = device_peripheral.ADC_SafetyCheckInst[rptrIndex].name
        var rptri = (currentRPTR).replace(/[^0-9]/g,'')
        var selectedsources=
        [
            "safetycheck"+ rptri.toString()+"usedSOC",
            // "safetycheck"+ rptri.toString()+"usedPPB",
            // "safetycheck"+ rptri.toString()+"usedPPBSUM",
        ];
        if (ui)
        {
            for (var selectedsource of selectedsources)
            {
                if ((inst["safetycheck"+ rptri.toString()+"inputSource"] != "ADC_SAFETY_CHECKER_INPUT_DISABLE"))
                {
                    ui["safetycheck"+ rptri.toString()+"usedSOC"].hidden = false;
                }
                else
                {
                    ui["safetycheck"+ rptri.toString()+"usedSOC"].hidden = true;
                }
            }
        }
    }
}

for (var rptrIndex in device_peripheral.ADC_SafetyCheckInst) {
    var currentRPTR = device_peripheral.ADC_SafetyCheckInst[rptrIndex].name
    let rptri = (currentRPTR).replace(/[^0-9]/g,'')
    config= config.concat ([
        {
            name:"safetychecker"+ rptri.toString(),
            displayName: "Selector "+ rptri.toString() + " Configurations",
            longDescription: "",
            collapsed: true,
            config:
            [

                {
                    name: "safetycheck"+ rptri.toString()+"ADC",
                    displayName :  "ADC Instance",
                    description : 'Select the ADC for this safety checker module',
                    default     : device_peripheral.ADC_Select[0].name,
                    options     : device_peripheral.ADC_Select,
                },
                {
                    name: "safetycheck"+ rptri.toString()+"ADCBase",
                    displayName :  "Selected ADC Base Address",
                    description : 'ADC module in syscfg',
                    // readOnly    : true,
                    default     : "",
                    getValue    : (inst)=>{
                        let adc =  system.modules['/drivers/adc/adc'];
                        let Base = "";
                        if (adc!=null)
                        {
                            for (let adc_inst of adc.$instances)
                            {
                                if (adc_inst[adc.getInterfaceName(adc_inst)].$solution.peripheralName ===
                                inst["safetycheck"+ rptri.toString()+"ADC"].replace("ADC_","ADC")){
                                    Base = adc_inst.$name+"_BASE_ADDR"
                                }
                            }
                        }
                        return Base
                    },
                },
                {
                    name: "safetycheck"+ rptri.toString()+"inputSource",
                    displayName: "Input Source"+ rptri.toString(),
                    hidden      : false,
                    default     : device_peripheral.ADC_SafetyCheckerInput[0].name,
                    options     : device_peripheral.ADC_SafetyCheckerInput,
                    onChange    : onChangesafetychecker,
                },
                {
                    name: "safetycheck"+ rptri.toString()+"ResultSelect",
                    displayName : "Result "+ rptri.toString(),
                    description : 'Select the result for the safety checker module',
                    default     : device_peripheral.ADC_ResultSelect[0].name,
                    options     : device_peripheral.ADC_ResultSelect
                },
                {
                    name: "safetycheck"+ rptri.toString()+"usedSOC",
                    displayName :  "Selected Source",
                    description : 'Selected SOCs for Selector '+ rptri.toString(),
                    hidden      : true,
                    default     :"",
                    getValue    : (inst) =>{
                        let adc =  system.modules['/drivers/adc/adc'];
                        var selectedsource= "";
                        if (adc!=null)
                            {
                                for ( var adc_inst of adc.$instances)
                                    {
                                        if (adc_inst[adc.getInterfaceName(adc_inst)].$solution.peripheralName ===
                                        inst["safetycheck"+ rptri.toString()+"ADC"].replace("ADC_","ADC")){
                                            for(var socIndex in device_peripheral.ADC_SOCNumber)
                                            {
                                                var currentSOC = device_peripheral.ADC_SOCNumber[socIndex].name
                                                var soci = (currentSOC).replace(/[^0-9]/g,'')
                                                var resi= inst["safetycheck"+ rptri.toString()+"ResultSelect"].replace(/[^0-9]/g,'')
                                                //return adc_inst.enabledSOCs.toString().replace("ADC_","").replace("_NUMBER"," number ").replace("SOC","SOC/EOC")
                                                // if((adc_inst.enabledPPBs).includes(currentPPB)){
                                                if(inst["safetycheck"+ rptri.toString()+"inputSource"]=="ADC_SAFETY_CHECKER_INPUT_SOCx")
                                                {
                                                    if (resi==soci)
                                                    {
                                                        selectedsource= "SOC"+soci.toString();
                                                    }
                                                }
                                            }

                                            for(var ppbIndex in device_peripheral.ADC_PPBNumber)
                                            {
                                                var currentPPB = device_peripheral.ADC_PPBNumber[ppbIndex].name
                                                var ppbi = (currentPPB).replace(/[^0-9]/g,'')
                                                var resi= inst["safetycheck"+ rptri.toString()+"ResultSelect"].replace(/[^0-9]/g,'')
                                                var highPPB = "";
                                                if((inst["safetycheck"+ rptri.toString()+"inputSource"]=="ADC_SAFETY_CHECKER_INPUT_PPBx")||(inst["safetycheck"+ rptri.toString()+"inputSource"]=="ADC_SAFETY_CHECKER_INPUT_PPBSUMx"))
                                                {
                                                    if (resi == adc_inst["ppb" + ppbi.toString() + "SOCNumber"].replace(/[^0-9]/g,''))
                                                    {
                                                    // highPPB+=adc_inst["ppb" + ppbi.toString() + "Name"];
                                                        highPPB+=adc_inst["ppb" + ppbi.toString() + "SOCNumber"]
                                                        selectedsource = adc_inst["ppb" + ppbi.toString() + "SOCNumber"].replace("ADC_","").replace("_NUMBER","")
                                                    }
                                                }
                                            }
                                        }
                                    }
                            }
                            return selectedsource;
                        }
                },
            ]
        },
    ])
}

config = config.concat([
            {
                name: "enableSafetychecker",
                displayName :  "Enable Safety Checker",
                description : 'Enable Safety Checker during initilization',
                hidden      : false,
                default     : false,
                // onChange    :onChangesafetychecker,
            },
        ])


function onValidateStatic(mod, stat)
{

}


function filterHardware(component)
{
    return (common.typeMatches(component.type, ["ADC_SC"]));
}

// function modules(inst)
// {
//     if (inst) {
//         return [
//             {
//                 name: "adcPullInTemplateDot",
//                 moduleName: "/driverlib/adc/templates/adc.dot.dynamic.js",
//             }
//         ];
//     }
//     return [];
// }

function onValidate(inst, validation) {

    //
    // Check copy safety Checker Instances
    //
    var usedADCSCInsts = [];
    for (var instance_index in inst.$module.$instances)
    {
        var instance_obj = inst.$module.$instances[instance_index];
        usedADCSCInsts.push(instance_obj.adcscBase);
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
                usedADCSCInsts.push(instance_obj.adcscBase);
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
            "The ADC Safety Checker Tile Instance used. Duplicates: " + allDuplicates,
            inst, "adcscBase");
    }
    //
    // Check duplicate ADC sources
    //
    let adc = system.modules["/drivers/.meta/adc/v2/adc_v2.syscfg.js"];

    if (adc!=null)
    {
        var usedADCInsts = [];
        for (var instance_index in adc.$instances)
        {
            var instance_obj = adc.$instances[instance_index];
            usedADCInsts.push(instance_obj.adcBase);
        }
        for(var rptrIndex in device_peripheral.ADC_SafetyCheckInst) {
            var currentRPTR = device_peripheral.ADC_SafetyCheckInst[rptrIndex].name
            let rptri = (currentRPTR).replace(/[^0-9]/g,'')
            if(!(usedADCInsts.includes(inst["safetycheck"+ rptri.toString()+"ADC"].replace("ADC_","ADC")+"_BASE"))){
            validation.logError(
                "This ADC Instance has not been configured. If you want to use this ADC, you can set it up via ADC module.",
                inst, "safetycheck"+ rptri.toString()+"ADC");
            }
        }
        for ( var adc_inst of adc.$instances)
        {
            if ((inst["safetycheck1usedSOC"]!=null)&&(inst["safetycheck2usedSOC"]!=null)) {
                var usedSOC1 =inst["safetycheck1ResultSelect"].replace(/[^0-9]/g,'')
                var usedSOC2 =inst["safetycheck2ResultSelect"].replace(/[^0-9]/g,'')
                var selectedSOC1 = inst["safetycheck1ResultSelect"].replace("RESULT","SOC_NUMBER")
                var selectedSOC2 = inst["safetycheck2ResultSelect"].replace("RESULT","SOC_NUMBER")
                //
                // Check the source for SOC
                //
                if ((adc_inst.enabledSOCs).includes(selectedSOC1)&&(adc_inst.enabledSOCs).includes(selectedSOC2)&&(inst["safetycheck1inputSource"]=="ADC_SAFETY_CHECKER_INPUT_SOCx")&&
                (inst["safetycheck2inputSource"]=="ADC_SAFETY_CHECKER_INPUT_SOCx")) {
                    //channel
                    if (adc_inst["soc"+ usedSOC1.toString()+"Channel"]==adc_inst["soc"+ usedSOC2.toString()+"Channel"])
                    {
                        validation.logWarning(
                            "Same channel has been selected for SOC of Selector 1 and Selector 2. For safety checking, you need to have different channels.",
                            inst, "safetycheck2usedSOC");
                    }
                    //trigger
                    if (!(adc_inst.enableBurstMode)) {
                        if ((adc_inst["soc"+ usedSOC1.toString()+"Triggermode"]=="singlemode")&&(adc_inst["soc"+ usedSOC2.toString()+"Triggermode"]=="singlemode"))
                        {
                            if (adc_inst["soc"+ usedSOC1.toString()+"Trigger"]!=adc_inst["soc"+ usedSOC2.toString()+"Trigger"])
                            {
                                validation.logWarning(
                                    "To get the accurate performance of Safety Checker, same SOC Trigger should be selected for Selector 1 and Selector 2.",
                                    inst, "safetycheck1usedSOC");
                            }
                        }
                        if (!((adc_inst["soc"+ usedSOC1.toString()+"Triggermode"]=="singlemode")^(adc_inst["soc"+ usedSOC2.toString()+"Triggermode"]=="repeatermode")) )
                        {
                            validation.logWarning(
                                "To get the accurate performance of Safety Checker, same Trigger Mode should be selected for SOC of Selector 1 and Selector 2.",
                                inst, "safetycheck1usedSOC");
                        }
                        if ((adc_inst["soc"+ usedSOC1.toString()+"Triggermode"]=="repeatermode")&&(adc_inst["soc"+ usedSOC2.toString()+"Triggermode"]=="repeatermode")) {
                            if (((adc_inst["soc"+ usedSOC1.toString()+"Trigger"]=="ADC_TRIGGER_REPEATER1" )&&(adc_inst["soc"+ usedSOC2.toString()+"Trigger"]=="ADC_TRIGGER_REPEATER2"))||
                            ((adc_inst["soc"+ usedSOC1.toString()+"Trigger"]=="ADC_TRIGGER_REPEATER2" )&&(adc_inst["soc"+ usedSOC2.toString()+"Trigger"]=="ADC_TRIGGER_REPEATER1"))) {
                                if (adc_inst["repeater1 Trigger"]!=adc_inst["repeater2 Trigger"])
                                {
                                    validation.logWarning(
                                        "To get the accurate performance of Safety Checker, you need to make sure same Triger is selected for repeater modules 1 and 2 associated with SOCs of Selector 1 and Selector 2.",
                                        inst, "safetycheck2usedSOC");
                                }
                            }
                        }
                    }

                    //sampling time
                    if (adc_inst["soc"+ usedSOC1.toString()+"CalculatedSampleTime"]!=adc_inst["soc"+ usedSOC2.toString()+"CalculatedSampleTime"])
                    {
                        validation.logWarning(
                            "To get the accurate performance of Safety Checker, the sample time should be identical for Selected SOC in Selector 1 and Selector 2.",
                            inst, "safetycheck2usedSOC");
                    }
                    if (adc_inst["soc"+ usedSOC1.toString()+"SampleTime"]!=adc_inst["soc"+ usedSOC2.toString()+"SampleTime"])
                    {
                        validation.logWarning(
                            "To get the accurate performance of Safety Checker, the sample time should be identical for Selected SOC in Selector 1 and Selector 2.",
                            inst, "safetycheck2usedSOC");
                    }
                }
            }
            //
            // Check the source for PPB
            //
            if ((inst["safetycheck1inputSource"]=="ADC_SAFETY_CHECKER_INPUT_PPBx")&&(inst["safetycheck2inputSource"]=="ADC_SAFETY_CHECKER_INPUT_PPBx")) {
                var res1= inst["safetycheck1ResultSelect"].replace(/[^0-9]/g,'')
                var res2= inst["safetycheck2ResultSelect"].replace(/[^0-9]/g,'')
                var ppbsoc4= adc_inst["ppb4SOCNumber"].replace(/[^0-9]/g,'')
                var ppbsoc3= adc_inst["ppb3SOCNumber"].replace(/[^0-9]/g,'')
                var ppbsoc2= adc_inst["ppb2SOCNumber"].replace(/[^0-9]/g,'')
                var ppbsoc1= adc_inst["ppb1SOCNumber"].replace(/[^0-9]/g,'')
                if ((inst.safetycheck1usedSOC)&&(inst.safetycheck2usedSOC)) {
                    if (res1!=res2) {
                        //
                        // To do: make a function to handle this section
                        //
                        // for (let xbari=1; xbari< 5; xbari++){
                        //     if ((adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER"+xbari)){

                        // }
                        if (((adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER1")&&(adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER2"))&&(((res1==ppbsoc1)&&(res2==ppbsoc2))||(res1==ppbsoc2)&&(res2==ppbsoc1))) {
                            if (adc_inst["ppb1ReferenceOffset"]!=adc_inst["ppb2ReferenceOffset"])
                            {
                                validation.logWarning(
                                    "Different Reference Offset values have been selected for PPB1 and PPB2. For safety checking, you need to make sure this setting is identical for PPB1 and PPB2",
                                    inst, "safetycheck1usedSOC");
                            }
                            if (adc_inst["ppb1EnableTwosComplement"]!=adc_inst["ppb2EnableTwosComplement"])
                            {
                                validation.logWarning(
                                    "Different settings related to Invert Output have been selected for PPB1 and PPB2. For safety checking, you need to make sure this setting is identical for PPB1 and PPB2.",
                                    inst, "safetycheck1usedSOC");
                            }
                        }
                        if (((adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER1")&&(adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER3"))&&(ppbsoc3!=ppbsoc2)&&(((res1==ppbsoc1)&&(res2==ppbsoc3))||(res1==ppbsoc3)&&(res2==ppbsoc1))) {
                            if (adc_inst["ppb3ReferenceOffset"]!=adc_inst["ppb1ReferenceOffset"])
                            {
                                validation.logWarning(
                                    "Different Reference Offset values have been selected for PPB1 and PPB3. For safety checking, you need to make sure this setting is identical for PPB1 and PPB3",
                                    inst, "safetycheck1usedSOC");
                            }
                            if (adc_inst["ppb3EnableTwosComplement"]!=adc_inst["ppb1EnableTwosComplement"])
                            {
                                validation.logWarning(
                                    "Different settings related to Invert Output have been selected for PPB1 and PPB3. For safety checking, you need to make sure this setting is identical for PPB1 and PPB3.",
                                    inst, "safetycheck1usedSOC");
                            }
                        }
                        if (((adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER1")&&(adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER4"))&&(ppbsoc2!=ppbsoc4)&&(ppbsoc3!=ppbsoc4)&&(((res1==ppbsoc1)&&(res2==ppbsoc4))||(res1==ppbsoc4)&&(res2==ppbsoc1))) {
                            if (adc_inst["ppb1ReferenceOffset"]!=adc_inst["ppb4ReferenceOffset"])
                            {
                                validation.logWarning(
                                    "Different Reference Offset values have been selected for PPB1 and PPB4. For safety checking, you need to make sure this setting is identical for PPB1 and PPB4",
                                    inst, "safetycheck1usedSOC");
                            }
                            if (adc_inst["ppb1EnableTwosComplement"]!=adc_inst["ppb4EnableTwosComplement"])
                            {
                                validation.logWarning(
                                    "Different settings related to Invert Output have been selected for PPB1 and PPB4. For safety checking, you need to make sure this setting is identical for PPB1 and PPB4.",
                                    inst, "safetycheck1usedSOC");
                            }
                        }
                        if (((adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER3")&&(adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER2"))&&(res1!=ppbsoc1)&&(res2!=ppbsoc1)&&(((res1==ppbsoc2)&&(res2==ppbsoc3))||(res1==ppbsoc3)&&(res2==ppbsoc2))) {
                            if (adc_inst["ppb3ReferenceOffset"]!=adc_inst["ppb2ReferenceOffset"])
                            {
                                validation.logWarning(
                                    "Different Reference Offset values have been selected for PPB3 and PPB2. For safety checking, you need to make sure this setting is identical for PPB3 and PPB2",
                                    inst, "safetycheck1usedSOC");
                            }
                            if (adc_inst["ppb3EnableTwosComplement"]!=adc_inst["ppb2EnableTwosComplement"])
                            {
                                validation.logWarning(
                                    "Different settings related to Invert Output have been selected for PPB3 and PPB2. For safety checking, you need to make sure this setting is identical for PPB3 and PPB2.",
                                    inst, "safetycheck1usedSOC");
                            }
                        }

                        if (((adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER2")&&(adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER4"))&&(res1!=ppbsoc1)&&(res2!=ppbsoc1)&&(res1!=ppbsoc3)
                        &&(res2!=ppbsoc3)&&(((res1==ppbsoc4)&&(res2==ppbsoc2))||(res1==ppbsoc2)&&(res2==ppbsoc4))) {
                            if (adc_inst["ppb2ReferenceOffset"]!=adc_inst["ppb4ReferenceOffset"])
                            {
                                validation.logWarning(
                                    "Different Reference Offset values have been selected for PPB2 and PPB4. For safety checking, you need to make sure this setting is identical for PPB2 and PPB4",
                                    inst, "safetycheck1usedSOC");
                            }
                            if (adc_inst["ppb2EnableTwosComplement"]!=adc_inst["ppb4EnableTwosComplement"])
                            {
                                validation.logWarning(
                                    "Different settings related to Invert Output have been selected for PPB2 and PPB4. For safety checking, you need to make sure this setting is identical for PPB2 and PPB4.",
                                    inst, "safetycheck1usedSOC");
                            }
                        }
                        if (((adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER3")&&(adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER4"))&&(res1!=ppbsoc1)&&(res2!=ppbsoc1)&&(res1!=ppbsoc2)&&
                        (res2!=ppbsoc2)&&(((res1==ppbsoc4)&&(res2==ppbsoc3))||(res1==ppbsoc3)&&(res2==ppbsoc4)))
                        {
                            if (adc_inst["ppb3ReferenceOffset"]!=adc_inst["ppb4ReferenceOffset"])
                            {
                                validation.logWarning(
                                    "Different Reference Offset values have been selected for PPB3 and PPB4. For safety checking, you need to make sure this setting is identical for PPB3 and PPB4.",
                                    inst, "safetycheck1usedSOC");
                            }
                            if (adc_inst["ppb3EnableTwosComplement"]!=adc_inst["ppb4EnableTwosComplement"])
                            {
                                validation.logWarning(
                                    "Different settings related to Invert Output have been selected for PPB3 and PPB4. For safety checking, you need to make sure this setting is identical for PPB3 and PPB4.",
                                    inst, "safetycheck1usedSOC");
                            }
                        }
                    }
                }
            }
            //
            // Check the source for PPBSum
            //
            if ((inst["safetycheck1inputSource"]=="ADC_SAFETY_CHECKER_INPUT_PPBSUMx")&&(inst["safetycheck2inputSource"]=="ADC_SAFETY_CHECKER_INPUT_PPBSUMx")) {
                var res1= inst["safetycheck1ResultSelect"].replace(/[^0-9]/g,'')
                var res2= inst["safetycheck2ResultSelect"].replace(/[^0-9]/g,'')
                var ppbsoc4= adc_inst["ppb4SOCNumber"].replace(/[^0-9]/g,'')
                var ppbsoc3= adc_inst["ppb3SOCNumber"].replace(/[^0-9]/g,'')
                var ppbsoc2= adc_inst["ppb2SOCNumber"].replace(/[^0-9]/g,'')
                var ppbsoc1= adc_inst["ppb1SOCNumber"].replace(/[^0-9]/g,'')
                if ((inst.safetycheck1usedSOC)&&(inst.safetycheck2usedSOC)) {
                    if (res1!=res2) {
                        if (((adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER1")&&(adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER2"))&&(((res1==ppbsoc1)&&(res2==ppbsoc2))||(res1==ppbsoc2)&&(res2==ppbsoc1))) {
                            if (adc_inst["ppb1ReferenceOffset"]!=adc_inst["ppb2ReferenceOffset"])
                            {
                                validation.logWarning(
                                    "Different Reference Offset values have been selected for PPB1 and PPB2. For safety checking, you need to make sure this setting is identical for PPB1 and PPB2",
                                    inst, "safetycheck1usedSOC");
                            }
                            if (adc_inst["ppb1EnableTwosComplement"]!=adc_inst["ppb2EnableTwosComplement"])
                            {
                                validation.logWarning(
                                    "Different settings related to Invert Output have been selected for PPB1 and PPB2. For safety checking, you need to make sure this setting is identical for PPB1 and PPB2.",
                                    inst, "safetycheck1usedSOC");
                            }
                            if (adc_inst["ppb1Rightshift"]!=adc_inst["ppb2Rightshift"])
                            {
                                validation.logWarning(
                                    "Different Number of Bits to Right Shift PSUM have been selected for PPB1 and PPB2.For safety checking, you need to make sure this setting is identical for PPB1 and PPB2.",
                                    inst, "safetycheck1usedSOC");
                            }
                        }
                        if (((adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER1")&&(adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER3"))&&(ppbsoc3!=ppbsoc2)&&(((res1==ppbsoc1)&&(res2==ppbsoc3))||(res1==ppbsoc3)&&(res2==ppbsoc1))) {
                            if (adc_inst["ppb3ReferenceOffset"]!=adc_inst["ppb1ReferenceOffset"])
                            {
                                validation.logWarning(
                                    "Different Reference Offset values have been selected for PPB1 and PPB3. For safety checking, you need to make sure this setting is identical for PPB1 and PPB3",
                                    inst, "safetycheck1usedSOC");
                            }
                            if (adc_inst["ppb3EnableTwosComplement"]!=adc_inst["ppb1EnableTwosComplement"])
                            {
                                validation.logWarning(
                                    "Different settings related to Invert Output have been selected for PPB1 and PPB3. For safety checking, you need to make sure this setting is identical for PPB1 and PPB3.",
                                    inst, "safetycheck1usedSOC");
                            }
                            if (adc_inst["ppb3Rightshift"]!=adc_inst["ppb1Rightshift"])
                            {
                                validation.logWarning(
                                    "Different Number of Bits to Right Shift PSUM have been selected for PPB1 and PPB3.For safety checking, you need to make sure this setting is identical for PPB1 and PPB3.",
                                    inst, "safetycheck1usedSOC");
                            }

                        }
                        if (((adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER1")&&(adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER4"))&&(ppbsoc2!=ppbsoc4)&&(ppbsoc3!=ppbsoc4)&&(((res1==ppbsoc1)&&(res2==ppbsoc4))||(res1==ppbsoc4)&&(res2==ppbsoc1))) {
                            if (adc_inst["ppb1ReferenceOffset"]!=adc_inst["ppb4ReferenceOffset"])
                            {
                                validation.logWarning(
                                    "Different Reference Offset values have been selected for PPB1 and PPB4. For safety checking, you need to make sure this setting is identical for PPB1 and PPB4",
                                    inst, "safetycheck1usedSOC");
                            }
                            if (adc_inst["ppb1EnableTwosComplement"]!=adc_inst["ppb4EnableTwosComplement"])
                            {
                                validation.logWarning(
                                    "Different settings related to Invert Output have been selected for PPB1 and PPB4. For safety checking, you need to make sure this setting is identical for PPB1 and PPB4.",
                                    inst, "safetycheck1usedSOC");
                            }
                            if (adc_inst["ppb1Rightshift"]!=adc_inst["ppb4Rightshift"])
                            {
                                validation.logWarning(
                                    "Different Number of Bits to Right Shift PSUM have been selected for PPB1 and PPB4.For safety checking, you need to make sure this setting is identical for PPB1 and PPB4.",
                                    inst, "safetycheck1usedSOC");
                            }
                        }
                        if (((adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER3")&&(adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER2"))&&(res1!=ppbsoc1)&&(res2!=ppbsoc1)&&(((res1==ppbsoc2)&&(res2==ppbsoc3))||(res1==ppbsoc3)&&(res2==ppbsoc2))) {
                            if (adc_inst["ppb3ReferenceOffset"]!=adc_inst["ppb2ReferenceOffset"])
                            {
                                validation.logWarning(
                                    "Different Reference Offset values have been selected for PPB3 and PPB2. For safety checking, you need to make sure this setting is identical for PPB3 and PPB2",
                                    inst, "safetycheck1usedSOC");
                            }
                            if (adc_inst["ppb3EnableTwosComplement"]!=adc_inst["ppb2EnableTwosComplement"])
                            {
                                validation.logWarning(
                                    "Different settings related to Invert Output have been selected for PPB3 and PPB2. For safety checking, you need to make sure this setting is identical for PPB3 and PPB2.",
                                    inst, "safetycheck1usedSOC");
                            }
                            if (adc_inst["ppb3Rightshift"]!=adc_inst["ppb2Rightshift"])
                            {
                                validation.logWarning(
                                    "Different Number of Bits to Right Shift PSUM have been selected for PPB3 and PPB2.For safety checking, you need to make sure this setting is identical for PPB3 and PPB2.",
                                    inst, "safetycheck1usedSOC");
                            }
                        }

                        if (((adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER2")&&(adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER4"))&&(res1!=ppbsoc1)&&(res2!=ppbsoc1)&&(res1!=ppbsoc3)&&(res2!=ppbsoc3)&&(((res1==ppbsoc4)&&(res2==ppbsoc2))||(res1==ppbsoc2)&&(res2==ppbsoc4))) {
                            if (adc_inst["ppb2ReferenceOffset"]!=adc_inst["ppb4ReferenceOffset"])
                            {
                                validation.logWarning(
                                    "Different Reference Offset values have been selected for PPB2 and PPB4. For safety checking, you need to make sure this setting is identical for PPB2 and PPB4",
                                    inst, "safetycheck1usedSOC");
                            }
                            if (adc_inst["ppb2EnableTwosComplement"]!=adc_inst["ppb4EnableTwosComplement"])
                            {
                                validation.logWarning(
                                    "Different settings related to Invert Output have been selected for PPB2 and PPB4. For safety checking, you need to make sure this setting is identical for PPB2 and PPB4.",
                                    inst, "safetycheck1usedSOC");
                            }
                            if (adc_inst["ppb2Rightshift"]!=adc_inst["ppb4Rightshift"])
                            {
                                validation.logWarning(
                                    "Different Number of Bits to Right Shift PSUM have been selected for PPB2 and PPB4.For safety checking, you need to make sure this setting is identical for PPB2 and PPB4.",
                                    inst, "safetycheck1usedSOC");
                            }
                        }
                        if (((adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER3")&&(adc_inst.enabledPPBs).includes("ADC_PPB_NUMBER4"))&&(res1!=ppbsoc1)&&(res2!=ppbsoc1)&&(res1!=ppbsoc2)&&(res2!=ppbsoc2)&&(((res1==ppbsoc4)&&(res2==ppbsoc3))||(res1==ppbsoc3)&&(res2==ppbsoc4)))
                        {
                            if (adc_inst["ppb3ReferenceOffset"]!=adc_inst["ppb4ReferenceOffset"])
                            {
                                validation.logWarning(
                                    "Different Reference Offset values have been selected for PPB3 and PPB4. For safety checking, you need to make sure this setting is identical for PPB3 and PPB4.",
                                    inst, "safetycheck1usedSOC");
                            }
                            if (adc_inst["ppb3EnableTwosComplement"]!=adc_inst["ppb4EnableTwosComplement"])
                            {
                                validation.logWarning(
                                    "Different settings related to Invert Output have been selected for PPB3 and PPB4. For safety checking, you need to make sure this setting is identical for PPB3 and PPB4.",
                                    inst, "safetycheck1usedSOC");
                            }
                            if (adc_inst["ppb3Rightshift"]!=adc_inst["ppb4Rightshift"])
                            {
                                validation.logWarning(
                                    "Different Number of Bits to Right Shift PSUM have been selected for PPB3 and PPB4.For safety checking, you need to make sure this setting is identical for PPB3 and PPB4.",
                                    inst, "safetycheck1usedSOC");
                            }
                        }
                    }
                }
            }

            for(var rptrIndex in device_peripheral.ADC_SafetyCheckInst) {
                var currentRPTR = device_peripheral.ADC_SafetyCheckInst[rptrIndex].name
                let rptri = (currentRPTR).replace(/[^0-9]/g,'')
                if ((inst["safetycheck"+ rptri.toString()+"usedSOC"]!="")&&(adc_inst.adcBase.replace("_BASE", "")==inst["safetycheck"+ rptri.toString()+"ADC"].replace("ADC_","ADC"))) {
                    if (inst["safetycheck"+ rptri.toString()+"inputSource"]=="ADC_SAFETY_CHECKER_INPUT_SOCx")
                    {
                        var selectedSOC = inst ["safetycheck"+rptri.toString()+"usedSOC"].replace(/[^0-9]/g,'')
                        validation.logInfo(
                            "You can change settings of Input Source for Selector "+ rptri.toString()+  " via"+ ": " + system.getReference(adc_inst,"soc"+selectedSOC.toString()+"Name"),
                            inst,  "safetycheck"+ rptri.toString()+"usedSOC");
                    }
                    if ((inst["safetycheck"+ rptri.toString()+"inputSource"]=="ADC_SAFETY_CHECKER_INPUT_PPBx")||(inst["safetycheck"+ rptri.toString()+"inputSource"]=="ADC_SAFETY_CHECKER_INPUT_PPBSUMx")) {
                        for(var ppbIndex in device_peripheral.ADC_PPBNumber) {
                            var currentPPB = device_peripheral.ADC_PPBNumber[ppbIndex].name
                            var ppbi = (currentPPB).replace(/[^0-9]/g,'')
                            var resi= inst["safetycheck"+ rptri.toString()+"ResultSelect"].replace(/[^0-9]/g,'')
                            let higherPPB="";
                            if (resi==adc_inst["ppb" + ppbi.toString() + "SOCNumber"].replace(/[^0-9]/g,'')){
                                if (adc_inst.enabledPPBs.includes("ADC_PPB_NUMBER1")) {
                                    if ((currentPPB=="ADC_PPB_NUMBER4")&&(adc_inst["ppb4SOCNumber"]!=adc_inst["ppb3SOCNumber"])&&(adc_inst["ppb4SOCNumber"]!=adc_inst["ppb2SOCNumber"])&&(adc_inst["ppb4SOCNumber"]!=adc_inst["ppb1SOCNumber"])) {
                                        higherPPB= "ppb4Name";
                                        validation.logInfo(
                                        "SOC"+resi.toString()+" is used in PPB4. You can change settings of Input Source for Selector "+ rptri.toString()+  " via"+ ": " + system.getReference(adc_inst,higherPPB),
                                        inst, "safetycheck"+ rptri.toString()+"usedSOC" );
                                    }
                                    if ((currentPPB=="ADC_PPB_NUMBER3")&&(adc_inst["ppb3SOCNumber"]!=adc_inst["ppb2SOCNumber"])&&(adc_inst["ppb3SOCNumber"]!=adc_inst["ppb1SOCNumber"])) {
                                        higherPPB= "ppb3Name";
                                        validation.logInfo(
                                        "SOC"+resi.toString()+" is used in PPB3. You can change settings of Input Source for Selector "+ rptri.toString()+  " via"+ ": " + system.getReference(adc_inst,higherPPB),
                                        inst, "safetycheck"+ rptri.toString()+"usedSOC" );
                                    }
                                    if ((currentPPB=="ADC_PPB_NUMBER2")&&(adc_inst["ppb1SOCNumber"]!=adc_inst["ppb2SOCNumber"])) {
                                        higherPPB= "ppb2Name";
                                        validation.logInfo(
                                        "SOC"+resi.toString()+" is used in PPB2. You can change settings of Input Source for Selector "+ rptri.toString()+  " via"+ ": " + system.getReference(adc_inst,higherPPB),
                                        inst, "safetycheck"+ rptri.toString()+"usedSOC" );
                                    }
                                    if ((currentPPB=="ADC_PPB_NUMBER1")) {
                                        higherPPB= "ppb1Name";
                                        validation.logInfo(
                                        "SOC"+resi.toString()+" is used in PPB1. You can change settings of Input Source for Selector "+ rptri.toString()+  " via"+ ": " + system.getReference(adc_inst,higherPPB),
                                        inst, "safetycheck"+ rptri.toString()+"usedSOC" );
                                    }
                                }
                                if (!(adc_inst.enabledPPBs.includes("ADC_PPB_NUMBER1"))) {
                                    if (adc_inst.enabledPPBs.includes("ADC_PPB_NUMBER2")) {
                                        if (currentPPB=="ADC_PPB_NUMBER2") {
                                        higherPPB= "ppb2Name";
                                        validation.logInfo(
                                        "SOC"+resi.toString()+" is used in PPB2. You can change settings of Input Source for Selector "+ rptri.toString()+  " via"+ ": " + system.getReference(adc_inst,higherPPB),
                                        inst, "safetycheck"+ rptri.toString()+"usedSOC" );
                                    }
                                    }
                                }

                                if (!(adc_inst.enabledPPBs.includes("ADC_PPB_NUMBER1"))) {
                                    if  (!(adc_inst.enabledPPBs.includes("ADC_PPB_NUMBER2"))||(resi!=adc_inst["ppb2" + "SOCNumber"].replace(/[^0-9]/g,''))) {
                                        if (adc_inst.enabledPPBs.includes("ADC_PPB_NUMBER3")){
                                            if (currentPPB=="ADC_PPB_NUMBER3")
                                            {
                                                higherPPB= "ppb3Name";
                                                validation.logInfo(
                                                "SOC "+resi.toString()+" is used in PPB3. You can change settings of Input Source for Selector "+ rptri.toString()+  " via"+ ": " + system.getReference(adc_inst,higherPPB),
                                                inst, "safetycheck"+ rptri.toString()+"usedSOC" );
                                            }
                                        }
                                    }
                                }

                                if (!(adc_inst.enabledPPBs.includes("ADC_PPB_NUMBER1"))) {
                                    if (!(adc_inst.enabledPPBs.includes("ADC_PPB_NUMBER3"))) {
                                        if (adc_inst.enabledPPBs.includes("ADC_PPB_NUMBER4")) {
                                            if (currentPPB=="ADC_PPB_NUMBER4") {
                                                higherPPB= "ppb4Name";
                                                validation.logInfo(
                                                "SOC"+resi.toString()+" is used in PPB4. You can change settings of Input Source for Selector "+ rptri.toString()+  " via"+ ": " + system.getReference(adc_inst,higherPPB),
                                                inst, "safetycheck"+ rptri.toString()+"usedSOC" );
                                            }
                                        }
                                    }

                                }

                            }
                        }
                    }
                }
            }
        }
        for(var rptrIndex in device_peripheral.ADC_SafetyCheckInst) {
            var currentRPTR = device_peripheral.ADC_SafetyCheckInst[rptrIndex].name
            let rptri = (currentRPTR).replace(/[^0-9]/g,'')
            if ((inst["safetycheck"+ rptri.toString()+"usedSOC"]=="") && (inst["safetycheck"+ rptri.toString()+"inputSource"] != "ADC_SAFETY_CHECKER_INPUT_DISABLE")) {
                validation.logWarning(
                    "There is no Input Source configured for this Result",
                    inst, "safetycheck"+ rptri.toString()+"usedSOC");
            }
        }

    }
    if ((inst["safetycheck1inputSource"] != "ADC_SAFETY_CHECKER_INPUT_DISABLE")||(inst["safetycheck2inputSource"] != "ADC_SAFETY_CHECKER_INPUT_DISABLE")){
        if (inst["safetycheck1inputSource"] != inst["safetycheck2inputSource"])
        {
            validation.logWarning(
                "Same Input Source should be selected for Selector 1 and Selector 2",
                inst, "safetycheck2inputSource");
        }
        if ((inst["safetycheck1ResultSelect"] == inst["safetycheck2ResultSelect"]) &&
            (inst["safetycheck1ADC"] == inst["safetycheck2ADC"]))
        {
            validation.logWarning(
                "Different Result should be selected for Selector 1 and Selector 2 when the same ADC instance is used for both",
                inst, "safetycheck1ResultSelect");
        }
    }
    if (inst["safetyCheckTolerance"] > 16777215)
    {
        validation.logError(
            "The max limit for Tolerance is 16777215",
            inst, "safetyCheckTolerance");
    }
    if (inst["safetyCheckTolerance"] < 0)
    {
        validation.logError(
            "Tolerance must be larger than 0",
            inst, "safetyCheckTolerance");
    }
    if (!Number.isInteger(inst["safetyCheckTolerance"]))
    {
    validation.logError(
        "Tolerance must be an integer value",
        inst, "safetyCheckTolerance");
    }
}

var adcscModule = {
    //peripheralName: "ADCSafetyChecker",
    displayName: "ADC SAFETY CHECKER",
    maxInstances: maxInstances,
    defaultInstanceName: "CONFIG_ADCSC_TILE",
    description: "ADC Safety Checker Module",
    //longDescription: (common.getCollateralFindabilityList("ADC")),
    filterHardware : filterHardware,
    config: config,
    templates: {
        //boardc : "/driverlib/adc/adcsc.board.c.xdt",
        //boardh : "/driverlib/adc/adcsc.board.h.xdt"
    },
    getInstanceConfig,

    //modules     : modules,
    validate    : onValidate,
};

exports = adcscModule;