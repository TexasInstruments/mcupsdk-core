
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
    let pinlist = ["MDIO","RGMII"]
    if(pinlist.includes(interfaceName))
    {
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
                "default": "pd",
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
                "default": "high",
                "legacy": true
            },
        );
    }
    else {
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
    }

    return pinConfig;
}

function getPinMode(peripheralPin)
{
    let devicePin = system.deviceData.devicePins[peripheralPin.$solution.packagePinName];
    let muxSetting = _.find(devicePin.mux.muxSetting,
                                function(muxSetting) {
                                    return (muxSetting.peripheralPin.name == peripheralPin.$solution.peripheralPinName);
                                }
                            );

    let muxMode = muxSetting.mode;

    /* work around to override issues in pinmux for a SOC */
    {
        muxMode = getPinModeOverRide(peripheralPin, muxSetting);
    }

    return muxMode;
}

function getPinConfigCStruct(pin, interfaceName)
{
    let mode = getPinMode(pin);
    let settings = "( ";

    settings += `PIN_MODE(${mode}) `
    switch(pin.pu_pd) {
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
    switch(pin.slewRate) {
        case "low":
            settings += "| PIN_SLEW_RATE_LOW ";
            break;
        case "high":
            settings += "| PIN_SLEW_RATE_HIGH ";
            break;
    }
    settings += ")";

    let pinConfigString =
    `/* ${pin.$solution.peripheralPinName} -> ${pin.$solution.devicePinName} (${pin.$solution.packagePinName}) */
    {
        PIN_${pin.$solution.devicePinName.toUpperCase()},
        ${settings}
    },`

    return pinConfigString;
}

exports = {
    getPinModeOverRide,
    getPinConfigurables,
    getPinConfigCStruct,
};
