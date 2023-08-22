
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("resolver");

    return system.getScript(`/drivers/resolver/${driverVer}/resolver_${driverVer}`);
}

exports = getModule();
