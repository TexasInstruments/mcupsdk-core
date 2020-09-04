let supported_spi_drivers = [
    {
        name: "qspi",
    },
];

let defaultFlashConfig = system.getScript("GD25B64.json");

function getDriverOptions()
{
    return supported_spi_drivers;
}

function getDefaultDriver()
{
    return supported_spi_drivers[0].name;
}

function getDefaultFlashName()
{
    return "GD25B64C";
}

function getDefaultFlashConfig()
{
    return defaultFlashConfig;
}

function getDefaultProtocol()
{
    return { name : "1s_1s_4s", displayName : "1S-1S-4S" };
}

exports = {
    getDriverOptions,
    getDefaultDriver,
    getDefaultFlashName,
    getDefaultProtocol,
    getDefaultFlashConfig,
};
