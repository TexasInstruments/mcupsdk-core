
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("pruicss");

    return system.getScript(`/drivers/pruicss/${driverVer}/pruicss_${driverVer}`);
}

exports = getModule();
