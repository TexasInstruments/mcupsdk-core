
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("cbuff");

    return system.getScript(`/drivers/cbuff/${driverVer}/cbuff_${driverVer}`);
}

exports = getModule();
