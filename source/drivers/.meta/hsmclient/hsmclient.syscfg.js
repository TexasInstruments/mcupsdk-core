let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("hsmclient");

    return system.getScript(`/drivers/hsmclient/${driverVer}/hsmclient_${driverVer}`);
}

exports = getModule();
