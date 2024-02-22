let common   = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let device_peripheral = system.getScript(`/drivers/epwm/soc/epwm_${common.getSocName()}.syscfg.js`);
let epwm_validation = system.getScript("/drivers/epwm/v1/epwm_validation.syscfg.js");

device_peripheral.EPWM_CurrentLink.unshift(
    { name: "EPWM_LINK_WITH_DISABLE", displayName: "Disable Linking" }
);

function getStaticConfigArr() {
    return system.getScript(`/drivers/epwm/soc/epwm_${common.getSocName()}`).getStaticConfigArr();
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

/* Array of configurables that are common across device families */
let config = [];

config.splice.apply(config, [config.length - 1, 0].concat([
        {
            name: "epwmLock",
            displayName : "EPWM Lock Register",
            description : 'Lock EPWM registers',
            hidden      : false,
            minSelections: 0,
            default     : [],
            options     : device_peripheral.EPWM_LockRegisterGroup
        },
        {
            name: "epwmGroup",
            displayName: "EPWM Group",
            description: "EPWM Group For This Instance",
            hidden: false,
            default: device_peripheral.EPWM_GROUP[0].name,
            options: device_peripheral.EPWM_GROUP
        },
    ]));

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
    return (common.typeMatches(component.type, ["EPWM"]));
}

let submodulesComponents = [
    {
        moduleName: "/drivers/epwm/v1/epwm_timebase.syscfg.js",
        name: "epwmTimebase",
        displayName:"EPWM Time Base",
        description:"Time Base Submodule",
    },

    {
        moduleName: "/drivers/epwm/v1/epwm_counterCompare.syscfg.js",
        name: "epwmCounterCompare",
        displayName:"EPWM Counter Compare",
        description:"Counter Compare Submodule",
    },
    {
        moduleName: "/drivers/epwm/v1/epwm_actionQualifier.syscfg.js",
        name: "epwmActionQualifier",
        displayName:"EPWM Action Qualifier",
        description:"Action Qualifier Submodule",
    },
    {
        moduleName: "/drivers/epwm/v1/epwm_tripZone.syscfg.js",
        name: "epwmTripZone",
        displayName:"EPWM Trip Zone",
        description:"Trip Zone Submodule",
    },
    {
        moduleName: "/drivers/epwm/v1/epwm_digitalCompare.syscfg.js",
        name: "epwmDigitalCompare",
        displayName:"EPWM Digital Compare",
        description:"Digital Compare Submodule",
    },
    {
        moduleName: "/drivers/epwm/v1/epwm_deadband.syscfg.js",
        name: "epwmDeadband",
        displayName:"EPWM Dead-Band",
        description:"Dead-Band Submodule",
    },
    {
        moduleName: "/drivers/epwm/v1/epwm_chopper.syscfg.js",
        name: "epwmChopper",
        displayName:"EPWM Chopper",
        description:"Chopper Submodule",
    },
    {
        moduleName: "/drivers/epwm/v1/epwm_eventTrigger.syscfg.js",
        name: "epwmEventTrigger",
        displayName:"EPWM Event-Trigger",
        description:"Event-Trigger Submodule",
    },
    {
        moduleName: "/drivers/epwm/v1/epwm_xcmp.syscfg.js",
        name: "epwmXcmp",
        displayName:"EPWM XCMP",
        description:"XCMP Mode Operation",
    },
    {
        moduleName: "/drivers/epwm/v1/epwm_diodeEmulation.syscfg.js",
        name: "epwmDiodeEmulation",
        displayName:"EPWM Diode Emulation",
        description:"EPWM Diode Emulation",
    },
    {
        moduleName: "/drivers/epwm/v1/epwm_hrpwm.syscfg.js",
        name: "epwmHRpwm",
        displayName:"HRPWM",
        description:"EPWM High Resolution PWM",
    },
    {
        moduleName: "/drivers/epwm/v1/epwm_globalLoad.syscfg.js",
        name: "epwmGlobalLoad",
        displayName:"EPWM Global Load",
        description:"Global Loading",
    },
    {
        moduleName: "/drivers/epwm/v1/epwm_minimumDeadBand.syscfg.js",
        name: "epwmMdl",
        displayName:"EPWM MDL",
        description:"EPWM MDL",
    },
    {
        moduleName: "/drivers/epwm/v1/epwm_icl.syscfg.js",
        name: "epwmIcl",
        displayName:"EPWM ICL",
        description:"EPWM ICL",
    },
];

for (let submoduleComponent of submodulesComponents)
{
    let submodule = system.getScript(submoduleComponent.moduleName)
    config = config.concat([
        {
            name: "GROUP_" + submodule.defaultInstanceName,
            displayName: submodule.displayName,
            longDescription: submodule.description,
            description: "",
            config: submodule.config
        },
    ])
}

let moduleStatic = {
    name: "epwmGlobal",
    displayName: "EPWM Global Settings",
    config: [
        {
            name:"enableInfo",
            displayName: "Enable Info Mode",
            default : false
        },
        {
            name:"enableWarning",
            displayName: "Enable Warnings",
            default : false
        }
    ],
}

function onValidate(inst, validation)
{
    //
    // Static Variables
    //

    let useInfos = inst.$module.$static["enableInfo"]
    let useWarnings = inst.$module.$static["enableWarning"]

    //
    // End of static variables
    //

    let usedEPWMTemplateNamesInsts = [];
    for (let instance_index in inst.$module.$instances)
    {
        let instance_obj = inst.$module.$instances[instance_index];
        if (instance_obj.generateCodeTemplate)
        {
            usedEPWMTemplateNamesInsts.push(instance_obj.codeTemplateName);
        }
    }

    let duplicatesResult = common.findDuplicates(usedEPWMTemplateNamesInsts)

    if (duplicatesResult.duplicates.length != 0)
    {
        let allDuplicates = "";
        for (let duplicateNamesIndex in duplicatesResult.duplicates)
        {
            allDuplicates = allDuplicates + common.stringOrEmpty(allDuplicates, ", ")
                            + duplicatesResult.duplicates[duplicateNamesIndex];
        }
        validation.logError(
            "The EPWM template name is already used. Duplicates: " + allDuplicates,
            inst, "codeTemplateName");
    }

    for (let valIndex = 0; valIndex < epwm_validation.epwm_validation.length; valIndex++)
    {
        let valObject = epwm_validation.epwm_validation[valIndex];
        if (!valObject.devices.includes(common.getSocName()))
        {
            continue;
        }
        if (valObject.type == epwm_validation.validation_info && !useInfos)
        {
        	continue;
        }
        if (valObject.type == epwm_validation.validation_warning && !useWarnings)
        {
        	continue;
        }
        valObject.func(inst, validation, valObject.name);
    }
}

function getInterfaceName(inst)
{
    return device_peripheral.getInterfaceName(inst);
}

function getPeripheralPinNames(inst)
{
    return [ "A", "B" ];
}

function pinmuxRequirements(inst)
{
   let interfaceName = getInterfaceName(inst);

    let resources = [];
    let pinResource = {};

    pinResource = pinmux.getPinRequirements(interfaceName, "A", "EPWMxA Pin");
    pinmux.setConfigurableDefault( pinResource, "rx", false );
    resources.push(pinResource);
    pinResource = pinmux.getPinRequirements(interfaceName, "B", "EPWMxB Pin");
    pinmux.setConfigurableDefault( pinResource, "rx", false );
    resources.push(pinResource);

    let peripheral = {
        name: interfaceName,
        displayName: "EPWM Instance",
        interfaceName: interfaceName,
        resources: resources,
    };

    return [peripheral];
}

let epwm_module_name = "/drivers/epwm/epwm";

let epwmModule = {
    peripheralName: "EPWM",
    displayName: "EPWM",
    defaultInstanceName: "CONFIG_EPWM",
    description: "Enhanced Pulse Width Modulator Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/epwm/templates/epwm.h.xdt",
        },
        "/drivers/system/system_config.c.xdt": {
            driver_init: "/drivers/epwm/templates/epwm_init.c.xdt",
            driver_deinit: "/drivers/epwm/templates/epwm_deinit.c.xdt",
            moduleName: epwm_module_name,
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/epwm/templates/epwm_open_close_config.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/epwm/templates/epwm_open_close_config.c.xdt",
            driver_open: "/drivers/epwm/templates/epwm_open.c.xdt",
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: epwm_module_name,
        },
    },
    moduleInstances     : undefined,
    moduleStatic: moduleStatic,
    validate : onValidate,
    getInstanceConfig,
    getInterfaceName,
    getPeripheralPinNames,
    pinmuxRequirements,
    onMigrate,
};

function onMigrate(newInst, oldInst, oldSystem) {
    let pins = getPeripheralPinNames(oldInst)
    let interfaceName = getInterfaceName(oldInst)
    common.onMigrate(newInst, oldInst, oldSystem, pins, interfaceName)
}

exports = epwmModule;