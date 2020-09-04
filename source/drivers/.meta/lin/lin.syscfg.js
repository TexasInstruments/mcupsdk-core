
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("lin");

    return system.getScript(`/drivers/lin/${driverVer}/lin_${driverVer}`);
}

exports = getModule();
