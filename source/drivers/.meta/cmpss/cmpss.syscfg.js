
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("cmpss");

    return system.getScript(`/drivers/cmpss/${driverVer}/cmpss_${driverVer}`);
}

exports = getModule();
