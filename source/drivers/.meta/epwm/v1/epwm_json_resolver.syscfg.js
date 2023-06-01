let common   = system.getScript("/common");
let device_driverlib_peripheral = system.getScript(`/drivers/epwm/soc/epwm_${common.getSocName()}.syscfg.js`);
let epwm_json = system.getScript("/drivers/epwm/v1/epwm.json").epwm;

let OPEN_PAR = "(";
let CLOSE_PAR = ");";
let COMMA = ", ";
let _BASE = "_BASE_ADDR"

function getCode(inst, inst_submodule, ui_name){
    let output = "";
    let appendArguments = true;

    let info = epwm_json.find(input => {
        return (input.name === ui_name) && input.devices.includes(common.getSocName())
    })

    if (!info){
        return "";
    }

    let uis_with_same_driverlib_function = epwm_json.filter(input => {
        return (input.driverlibFunction === info.driverlibFunction) && input.devices.includes(common.getSocName())
    })

    let require_other_ui_names = false;
    let driverlibFunctionArgKeysArray = Object.keys(info.driverlibFunctionArg);
    let outputFunctionArgs = new Array(driverlibFunctionArgKeysArray.length);
    for (let functionArgIndex = 0; functionArgIndex < driverlibFunctionArgKeysArray.length; functionArgIndex++)
    {
        let functionArgName = driverlibFunctionArgKeysArray[functionArgIndex];
        let functionArgValue = info.driverlibFunctionArg[functionArgName];
        if (functionArgName == "base")
        {
            outputFunctionArgs[functionArgIndex] = inst.$name + _BASE;
        }
        else if (functionArgValue == "this")
        {
            outputFunctionArgs[functionArgIndex] = inst_submodule[ui_name];
        }
        else if (functionArgValue == "")
        {
            require_other_ui_names = true;
            let requried_other_ui = uis_with_same_driverlib_function.find(input => {
                if (input.driverlibFunctionArg[functionArgName] == "this")
                {
                    return input
                }
            });
            outputFunctionArgs[functionArgIndex] = inst_submodule[requried_other_ui.name];
        }
        else
        {
            outputFunctionArgs[functionArgIndex] = functionArgValue;
        }
    }

    // For boolean type UI elements
    // If it has an alternate driverlib version then it is boolean type
    // Use driverlibFunctionAlt
    // If driverlibFunction or driverlibFunctionAlt is empty then do not include arguments in the output

    if (info.hasOwnProperty('driverlibFunctionAlt'))
    {
        if (inst_submodule[ui_name])
        {
            if (info.driverlibFunction == "")
            {
                output = "";
                appendArguments = false;
            }
            else
            {
                output = info.driverlibFunction;
                appendArguments = true;
            }
        }
        else
        {
            if (info.driverlibFunctionAlt == "")
            {
                output = "";
                appendArguments = false;
            }
            else
            {
                output = info.driverlibFunctionAlt;
                appendArguments = true;
            }

        }
    }
    else
    {
        output = info.driverlibFunction;
        appendArguments = true;
    }

    // Only add arguments to a function if there is a function being outputted
    // This is cross referenced with the code above
    if (appendArguments)
    {
        output += OPEN_PAR;
        for (let outputFunctionArgIndex = 0; outputFunctionArgIndex < outputFunctionArgs.length; outputFunctionArgIndex++)
        {
            // Check to see if the argument belongs to an OR'd input
            if (String(outputFunctionArgs[outputFunctionArgIndex]).indexOf(',') > -1)
            {
                outputFunctionArgs[outputFunctionArgIndex] = String(outputFunctionArgs[outputFunctionArgIndex]).replace(/,/g, " | ")
            }
            // Check to see if argument is empty (no options of an OR'ed select are selected) and output 0
            if (String(outputFunctionArgs[outputFunctionArgIndex]) == "")
            {
                outputFunctionArgs[outputFunctionArgIndex] = "0";
            }

            output += outputFunctionArgs[outputFunctionArgIndex];
            if (outputFunctionArgIndex != outputFunctionArgs.length - 1)
            {
                output += COMMA;
            }
        }

            output += CLOSE_PAR;
    }

    return output;
}

exports = {
    getCode : getCode,
};