
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("mcspi");

    return system.getScript(`/drivers/mcspi/${driverVer}/mcspi_${driverVer}`);
}

exports = getModule();
