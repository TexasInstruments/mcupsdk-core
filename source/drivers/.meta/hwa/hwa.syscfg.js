
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("hwa");

    return system.getScript(`/drivers/hwa/${driverVer}/hwa_${driverVer}`);
}

exports = getModule();
