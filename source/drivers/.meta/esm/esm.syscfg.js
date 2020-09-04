
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("esm");

    return system.getScript(`/drivers/esm/${driverVer}/esm_${driverVer}`);
}

exports = getModule();
