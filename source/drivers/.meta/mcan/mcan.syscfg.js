
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("mcan");

    return system.getScript(`/drivers/mcan/${driverVer}/mcan_${driverVer}`);
}

exports = getModule();
