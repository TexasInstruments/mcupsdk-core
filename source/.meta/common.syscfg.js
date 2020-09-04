
function getSelfSysCfgCoreName() {
    if(system.deviceData.core)
        return system.deviceData.core.name;

    /* when system.deviceData.core is not defined return a valid core for current SoC */
    switch(getSocName()) {
        case "am243x":
            return "r5fss0-0";
        default:
        case "am64x":
            return "r5fss0-0";
        case "am263x":
            return "r5fss0-0";
        case "am273x":
            return "r5fss0-0";
        case "awr294x":
            return "r5fss0-0";
        case "am62x":
            return "m4fss0-0";
    }
};

function isSciClientSupported() {
    switch(getSocName()) {
        case "am243x":
            return true;
        case "am64x":
            return true;
        case "am62x":
            return true;
        default:
            return false;
    }
};

function getSocName() {
    if(system.deviceData.device == "AM64x_beta")
        return "am64x";
    if((system.deviceData.device == "AM243x_ALV_beta") || (system.deviceData.device == "AM243x_ALX_beta"))
        return "am243x";
    if(system.deviceData.device == "AM263x_beta")
        return "am263x";
    if(system.deviceData.device == "AM273x")
        return "am273x";
    if((system.deviceData.device == "AWR294X") || (system.deviceData.device == "AWR294XLOP"))
        return "awr294x";
    if(system.deviceData.device == "AM62x")
        return "am62x";
};

function getDeviceName() {
    if(system.deviceData.device == "AM64x_beta")
        return "am64x-evm";
    if(system.deviceData.device == "AM243x_ALV_beta")
        return "am243x-evm";
    if(system.deviceData.device == "AM243x_ALX_beta")
        return "am243x-lp";
    if(system.deviceData.device == "AM263x_beta")
        return "am263x-cc";
    if(system.deviceData.device == "AM273x")
        return "am273x-evm";
    if(system.deviceData.device == "AWR294X")
        return "awr294x-evm";
    if(system.deviceData.device == "AM62x")
        return "am62x-sk";
};

function isCName(id) {
    if ( (id != null) && (id.match(/^[a-zA-Z_][0-9a-zA-Z_]*$/) != null) )
    {
        return true;
    }
    return false;
}

function camelSentence(str) {
    return  (" " + str).toLowerCase().replace(/[^a-zA-Z0-9]+(.)/g, function(match, chr)
    {
        return chr.toUpperCase();
    });
}

function getSysCfgCoreNames()
{
    let core_names = [];

    for( let core of system.deviceData.cores)
    {
        core_names.push(core.name);
    }
    return core_names;
}

function getStaticModuleForCore(moduleName, coreName)
{
    let context = system.contexts[coreName];
    let modules = context ? context.system.modules : {};
    let module = modules[moduleName];

    if(module && module.$static)
        return module.$static;

    return undefined;
}

function getModuleForCore(moduleName, coreName)
{
    let context = system.contexts[coreName];
    let modules = context ? context.system.modules : {};
    let module = modules[moduleName];

    return module;
}


function makeConfig(staticConfigArr, fieldname, displayName)
{
    let config = {};

    if(staticConfigArr.length == 0)
        return undefined;

    config.name = `${fieldname}`;
    config.displayName = `${displayName}`;
    config.description = `Select ${displayName}`;
    config.default = staticConfigArr[0].name;
    config.options = [];

    for (let i = 0; i < staticConfigArr.length; i++) {
        let option = {};

        option.name = staticConfigArr[i].name;
        if(staticConfigArr[i].description)
        {
            option.description = staticConfigArr[i].description;
        }
        config.options.push(option);
    }

    return config;
}

function checkSameFieldName(instance, fieldname, report)
{
    let moduleInstances = instance.$module.$instances;

    for (let i = 0; i < moduleInstances.length; i++) {
        if (instance[fieldname] === moduleInstances[i][fieldname] &&
            instance !== moduleInstances[i]) {
            report.logError(`Same ${fieldname} cannot be selected`, instance, fieldname);
        }
    }
}

function checkSameFieldNameOnAllCores(moduleName, instance, fieldname, report)
{
    let coreNames = getSysCfgCoreNames();
    for ( let remoteCoreName of coreNames)
    {
        let remote_core_instance = getModuleForCore(moduleName, remoteCoreName);
        if(remote_core_instance)
        {
            let moduleInstances = remote_core_instance.$instances;
            for (let i = 0; i < moduleInstances.length; i++) {
                if (instance[fieldname] === moduleInstances[i][fieldname] &&
                    instance !== moduleInstances[i]) {
                    report.logError(`Same ${fieldname} cannot be selected`, instance, fieldname);
                }
            }
        }
    }
}

function getUseMcuDomainPeripheralsConfig()
{
    let config = {
        name: "useMcuDomainPeripherals",
        displayName: "Use MCU Domain Peripherals",
        default: false,
    }

    if(getSelfSysCfgCoreName().includes("m4f")) {
        config.default = true;
        config.readOnly = true;
    }

    return config;
}

function isMcuDomainSupported()
{
    switch(getSocName()) {
        case "am243x":
            return true;
        case "am64x":
            return true;
        case "am62x":
            return true;
        default:
            return false;
    }
}

function findDuplicates(arrayToCheck)
{
    const count = arrayToCheck =>
      arrayToCheck.reduce((a, b) => ({ ...a,
        [b]: (a[b] || 0) + 1
      }), {})

    const duplicates = dict =>
      Object.keys(dict).filter((a) => dict[a] > 1)

    return {count: count(arrayToCheck), duplicates: duplicates(count(arrayToCheck))};
}

/*
 * Checks whether a given string is empty and returns a string
 * If stringToCheck is empty, an empty string is returned otherwise
 * stringToAdd is returned
 *
 * stringToCheck - The string to be compared to empty string
 * stringToAdd - The string to return when stringToCheck is non empty
 *
 */
function stringOrEmpty(stringToCheck, stringToAdd)
{
    if (stringToCheck != "")
    {
        return stringToAdd;
    }
    return "";
}

/*
 *  Check that HW signal type matches a specified array of types
 *
 *  Example: within a module's filterHardware(component) method:
 *      for (sig in component.signals) {
 *          let type = component.signals[sig].type;
 *          if (Common.typeMatches(type, ["PWM", "DOUT"])) {
 *              :
 *          }
 *      }
 *
 *  type      - a string or array of strings that are valid signal types
 *  nameArray - array of signal name types that must match one of the signal
 *              types named by type
 *
 *  Returns true iff nameArray contains at least one type name that's
 *          specified the type parameter.
 */
function typeMatches(type, nameArray)
{
    let options = {};

    if (type instanceof Array || typeof type == "object") {
        for (var i = 0; i < type.length; i++) {
            options[type[i]] = 1;
        }
    }
    else if (typeof type == "string" || type instanceof String) {
        options[type] = 1;
    }

    for (var i = 0; i < nameArray.length; i++) {
        let name = nameArray[i];
        if (name in options) {
            return (true);
        }
    }

    return (false);
}

exports = {
    getSelfSysCfgCoreName,
    isSciClientSupported,
    getSocName,
    getDeviceName,
    camelSentence,
    getStaticModuleForCore,
    getModuleForCore,
    getSysCfgCoreNames,
    getUseMcuDomainPeripheralsConfig,
    isMcuDomainSupported,
    findDuplicates,
    stringOrEmpty,
    typeMatches,

    validate: {
        checkSameInstanceName : function (instance, report) {
            checkSameFieldName(instance, "instance", report);
        },
        checkSameInstanceNameOnAllCores : function (moduleName, instance, report) {
            checkSameFieldNameOnAllCores(moduleName, instance, "instance", report);
        },
        checkSameFieldName : checkSameFieldName,
        checkSameFieldNameOnAllCores : checkSameFieldNameOnAllCores,
        checkNumberRange: function(instance, report, property, min, max, displayFormat) {
            if(instance[property] > max || instance[property] < min) {
                let base = 10;

                if(displayFormat == "hex") {
                    base = 16;

                    report.logError( `Value must be between 0x${min.toString(base).toUpperCase()}`
                    + ` .. 0x${max.toString(base).toUpperCase()}`,
                     instance, property);
                }
                else {
                    report.logError( `Value must be between ${min.toString(base).toUpperCase()}`
                    + ` .. ${max.toString(base).toUpperCase()}`,
                     instance, property);
                }
            }
        },
        checkValidCName: function(instance, report, property) {
            if(isCName((instance[property])) === false) {
                report.logError(`Provide a valid C name`, instance, property);
            }
        },
        checkConfigurableValueMatchForAllCores(moduleName, instance, report, configurable, core_instances)
        {
            let coreNames = getSysCfgCoreNames();
            let selfCoreName = getSelfSysCfgCoreName();

            for ( let remoteCoreName of coreNames)
            {
                if( remoteCoreName != selfCoreName)
                {
                    let remote_core_instance = getStaticModuleForCore(moduleName, remoteCoreName);

                    if(remote_core_instance && instance[configurable] != remote_core_instance[configurable])
                    {
                        report.logError("Value MUST match with other cores", instance, configurable);
                        break;
                    }
                }
            }
        },
    },

    ui:
    {
        makeInstanceConfig: function(staticConfigArr) {
            return makeConfig(staticConfigArr, "instance", "Instance");
        },
        makeConfig: makeConfig,
    }
};
