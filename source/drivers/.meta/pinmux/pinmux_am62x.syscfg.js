
/* SOC specific overrides for pinmux data */

function getPinModeOverRide(peripheralPin, muxSetting)
{
    let mode = muxSetting.mode;

    if( muxSetting.peripheralPin.peripheralName.includes("PRU_ICSS")
     && muxSetting.peripheralPin.peripheralName.includes("MII_G_RT") )
    {
        /* In MII_G_RT sub-peripheral within PRU_ICSS the mux value in device is not as expected.
         * Hence changing the value to match the requirement in SW
         */
        if(muxSetting.mode === "14")
            mode = "1";
        if(muxSetting.mode === "13")
            mode = "0";
    }

    return mode;
}

/* SOC specific configurables for pinmux */

function getPinConfigurables(interfaceName, pinName)
{
    let pinConfig = _.cloneDeep(
        system.deviceData.interfaces[interfaceName].interfacePins[pinName].configurables
    );
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
    let pu_pd = undefined;

    if(pin.pu_pd != undefined)
       pu_pd = pin.pu_pd;
    if(pin.pull != undefined)
       pu_pd = pin.pull;

    let rx = pin.rx;
    let mode = getPinMode(pin);
    let settings = "( ";

    /* if no values provided for pu_pd and rx, then use defaults as defined in deviceData */
    if( pu_pd == undefined && rx == undefined)
    {
        let peripheralPinDefaultConfig = system.deviceData.peripheralPins[pin.$solution.peripheralPinName].interfacePin.configurables;

        /* create a instance like obj, so that we can get to the defaults as we would from UI */
        const instanceLikeObj = _.reduce(peripheralPinDefaultConfig, (result, configurable) => {
            result[configurable.name] = configurable.default;
            return result;
        }, {});

        pu_pd = instanceLikeObj.pu_pd;
        rx = instanceLikeObj.rx;
    }

    settings += `PIN_MODE(${mode}) `
    if(rx===true)
    {
        settings += "| PIN_INPUT_ENABLE ";
    }
    switch(pu_pd) {
        case "pu":
            settings += "| PIN_PULL_DIRECTION ";
            break;
        case "pd":
            break;
        case "nopull":
            settings += "| PIN_PULL_DISABLE ";
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
