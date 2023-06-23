
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("gpmc");

    return system.getScript(`/drivers/gpmc/${driverVer}/gpmc_${driverVer}`);
}

exports = getModule();
