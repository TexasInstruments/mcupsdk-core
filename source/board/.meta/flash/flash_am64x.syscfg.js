let common = system.getScript("/common");

let supported_nor_spi_drivers = [
    {
        name: "ospi",
    },
];

let defaultFlashConfig = system.getScript("/board/flash/S28HS512T.json");

function getDriverOptions()
{
    return supported_nor_spi_drivers;
}

function getDefaultDriver()
{
    return supported_nor_spi_drivers[0].name;
}

function getDefaultFlashName()
{
    return "S28HS512T";
}

function getDefaultProtocol()
{
    return { name : "8d_8d_8d", displayName : "8D-8D-8D" };
}

function getDefaultProtocolJson()
{
    return "p888d";
}

function getDefaultFlashConfig()
{
    return defaultFlashConfig;
}

exports = {
    getDriverOptions,
    getDefaultDriver,
    getDefaultFlashName,
    getDefaultProtocol,
    getDefaultProtocolJson,
    getDefaultFlashConfig,
};
