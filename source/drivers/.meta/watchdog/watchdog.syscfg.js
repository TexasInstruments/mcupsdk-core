
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("watchdog");

    return system.getScript(`/drivers/watchdog/${driverVer}/watchdog_${driverVer}`);
}

exports = getModule();
