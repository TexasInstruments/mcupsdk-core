
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("eqep");

    return system.getScript(`/drivers/eqep/${driverVer}/eqep_${driverVer}`);
}

exports = getModule();
