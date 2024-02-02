let common = system.getScript("/common");

/* SOC specific overrides for pinmux data */

function getPinModeOverRide(peripheralPin, muxSetting)
{
    let mode = muxSetting.mode;

    return mode;
}

/* SOC specific configurables for pinmux */

function getPinConfigurables(interfaceName, pinName)
{
    let pinConfig = [];
    pinConfig.push(
        {
            "type": "DropDown",
            "name": "pu_pd",
            "readOnly": false,
            "hidden": false,
            "displayName": "Pull Up/Down",
            "options": [
                {
                    "name": "pu",
                    "displayName": "Pull Up",
                    "value": "N/A"
                },
                {
                    "name": "pd",
                    "displayName": "Pull Down",
                    "value": "N/A"
                },
                {
                    "name": "nopull",
                    "displayName": "No Pull",
                    "value": "N/A"
                }
            ],
            "default": "nopull",
            "legacy": true
        },
        {
            "type": "DropDown",
            "name": "slewRate",
            "readOnly": false,
            "hidden": false,
            "displayName": "Slew Rate",
            "options": [
                {
                    "name": "high",
                    "displayName": "High",
                    "value": "N/A"
                },
                {
                    "name": "low",
                    "displayName": "Low",
                    "value": "N/A"
                },
            ],
            "default": "low",
            "legacy": true
        },
    );
    if(interfaceName === "ICSSM"){
        pinConfig.push({
            "type": "CheckBox",
            "name": "rx",
            "readOnly": false,
            "hidden": false,
            "displayName": "rx",
            "default": false,
            "legacy": true
        })
    }
    if( interfaceName == "SDFM" || interfaceName == "EQEP" || interfaceName == "GPIO")
    {
        pinConfig.push(
            {
                "type": "CheckBox",
                "name": "inv",
                "readOnly": false,
                "hidden": false,
                "displayName": "Invert",
                "default": false,
                "legacy": true
            },
            {
                "type": "DropDown",
                "name": "qualSel",
                "readOnly": false,
                "hidden": false,
                "displayName": "Qual Sel",
                "options": [
                    {
                        "name": "sync",
                        "displayName": "Sync",
                        "value": "N/A"
                    },
                    {
                        "name": "3sample",
                        "displayName": "3 Sample",
                        "value": "N/A"
                    },
                    {
                        "name": "6sample",
                        "displayName": "6 Sample",
                        "value": "N/A"
                    },
                    {
                        "name": "async",
                        "displayName": "Async",
                        "value": "N/A"
                    }
                ],
                "default": "sync",
                "legacy": true
            },
        );
    }
    return pinConfig;
}

function getPinMode(peripheralPin)
{
    let devicePin = system.deviceData.devicePins[peripheralPin.$solution.packagePinName];
    if (devicePin === undefined) return null
    let muxSetting = _.find(devicePin.mux.muxSetting,
                                function(muxSetting) {
                                    return (muxSetting.peripheralPin.name == peripheralPin.$solution.peripheralPinName);
                                }
                            );

    let muxMode = muxSetting.mode;

    /* work around to override issues in pinmux for a SOC */
    if(getPinModeOverRide)
    {
        muxMode = getPinModeOverRide(peripheralPin, muxSetting);
    }

    return muxMode;
}

function getPinConfigCStruct(pin, interfaceName, inst)
{
    let mode = getPinMode(pin);
    if (mode === null) return ""
    let settings = "( ";

    let obj;
    if( interfaceName == "GPIO_n") obj = inst;
    else obj = pin;

    settings += `PIN_MODE(${mode}) `
    switch(obj.pu_pd) {
        case "pu":
            settings += "| PIN_PULL_UP ";
            break;
        case "pd":
            settings += "| PIN_PULL_DOWN ";
            break;
        case "nopull":
            settings += "| PIN_PULL_DISABLE ";
            break;
    }
    switch(obj.slewRate) {
        case "low":
            settings += "| PIN_SLEW_RATE_LOW ";
            break;
        case "high":
            settings += "| PIN_SLEW_RATE_HIGH ";
            break;
    }
    if(obj.inv != undefined)
    {
        if(obj.inv == true)
        {
            settings += "| PIN_INVERT ";
        }
    }
    if(obj.qualSel != undefined)
    {
        switch(obj.qualSel) {
            case "sync":
                settings += "| PIN_QUAL_SYNC ";
                break;
            case "3sample":
                settings += "| PIN_QUAL_3SAMPLE ";
                break;
            case "6sample":
                settings += "| PIN_QUAL_6SAMPLE ";
                break;
            case "async":
                settings += "| PIN_QUAL_ASYNC ";
                break;
        }
    }
    if(interfaceName == "GPIO_n")
    {
        let cpu = common.getSelfSysCfgCoreName();
        switch(cpu) {
            case "r5fss0-0":
                settings += "| PIN_GPIO_R5SS0_0 ";
                break;
            case "r5fss0-1":
                settings += "| PIN_GPIO_R5SS0_1 ";
                break;
            case "r5fss1-0":
                settings += "| PIN_GPIO_R5SS1_0 ";
                break;
            case "r5fss1-1":
                settings += "| PIN_GPIO_R5SS1_1 ";
                break;
        }
    }
    settings += ")";

    let pinConfigString =
    `/* ${pin.$solution.peripheralPinName} -> ${pin.$solution.devicePinName} (${pin.$solution.packagePinName}) */
    {
        PIN_${pin.$solution.devicePinName.toUpperCase()},
        ${settings}
    },`
    //QSPI_CLKLB is not exposed on any pin, so the pinmux register config is being done manually
    if(pin.$solution.peripheralPinName == "QSPI_CLK")
    {
        pinConfigString +=
    `
    /* QSPI_CLKLB -> QSPI_CLKLB */
    {
        PIN_QSPI_CLKLB,
        ( PIN_MODE(0) | PIN_FORCE_INPUT_ENABLE | PIN_FORCE_OUTPUT_ENABLE)
    },`
    }
    return pinConfigString;
}

exports = {
    getPinModeOverRide,
    getPinConfigurables,
    getPinConfigCStruct,
};
