
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/pinmux/pinmux_${common.getSocName()}`);

function getPinRequirements(interfaceName, pinName, pinDisplayName)
{
    let pinConfig = soc.getPinConfigurables(interfaceName, pinName);

    let pin = {
        name           : pinName,
        displayName    : (pinDisplayName ? pinDisplayName : pinName),
        interfaceNames : [pinName],
        config         : pinConfig,
    };
    return pin;
}

/* valid values
     configurableName "pu_pd", valid defaultValue are "pu", "pd", "nopull"
     configurableName "rx", valid defaultValue are true, false
 */
function setConfigurableDefault(pin, configurableName, defaultValue)
{
    let configurable = _.find(pin.config,
        function(c) {
            return (c.name == configurableName);
        }
    );
    if(configurable)
    {
        configurable.default = defaultValue;
    }
}

/* here peripheral MUST contain array of "resources" one for each pin

   This array is searched for the pin name and if found default configurable value is set
 */
function setPeripheralPinConfigurableDefault(peripheral, pinName, configurableName, defaultValue)
{
    let resources = peripheral.resources;

    if(resources && Array.isArray(resources))
    {
        resources.forEach( function(pinResource) {
            if(pinResource.name === pinName)
            {
                setConfigurableDefault(pinResource, configurableName, defaultValue);
            }
        });
    }
}

function getInterfacePinList(interfaceName)
{
    let interfacePins = system.deviceData.interfaces[interfaceName].interfacePins;

    return Object.keys(interfacePins);
}

function getPinConfigurables(interfaceName, pinName)
{
    let pinConfig = _.cloneDeep(
            system.deviceData.interfaces[interfaceName].interfacePins[pinName].configurables
        );

    return pinConfig;
}

function getGpioPinRequirements(interfaceName, pinName)
{
    let pinConfig = soc.getPinConfigurables(interfaceName, pinName);

    let interfacePins = Object.keys(system.deviceData.interfaces[interfaceName].interfacePins);

    let pin = {
		name           : "gpioPin",
		displayName    : "GPIO Pin",
        interfaceNames : interfacePins,
        config         : pinConfig,
    };
    return pin;
}

exports = {
    getPinRequirements,
    getGpioPinRequirements,
    getPinConfigurables,
    getInterfacePinList,
    setConfigurableDefault,
    setPeripheralPinConfigurableDefault,
};
