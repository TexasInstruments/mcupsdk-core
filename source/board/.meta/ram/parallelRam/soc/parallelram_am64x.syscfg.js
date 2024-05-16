let common = system.getScript("/common");

let supported_parallelRam_drivers = [
    {
        name: "gpmc",
    },
];

let defaultPsramConfig = system.getScript("/board/psram/IS67WVE4M16EBLL-70BLA1.json");

function getDriverOptions()
{
    return supported_parallelRam_drivers;
}

function getDefaultDriver()
{
    return supported_psram_drivers[0].name;
}

function getDefaultPsramName()
{
    return "IS67WVE4M16EBLL70BLA1";
}

function getDefaultPsramConfig()
{
    return defaultPsramConfig;
}

function getDefaultDevice()
{
    return "PSRAM";
}
exports = {
    getDriverOptions,
    getDefaultDriver,
    getDefaultPsramName,
    getDefaultPsramConfig,
    getDefaultDevice,
};