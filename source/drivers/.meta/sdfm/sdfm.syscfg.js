
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("sdfm");

    return system.getScript(`/drivers/sdfm/${driverVer}/sdfm_${driverVer}`);
}

exports = getModule();
