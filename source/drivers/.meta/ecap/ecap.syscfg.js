
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("ecap");

    return system.getScript(`/drivers/ecap/${driverVer}/ecap_${driverVer}`);
}

exports = getModule();
