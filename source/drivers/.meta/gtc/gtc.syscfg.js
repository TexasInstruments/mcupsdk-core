
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("gtc");

    return system.getScript(`/drivers/gtc/${driverVer}/gtc_${driverVer}`);
}

exports = getModule();
