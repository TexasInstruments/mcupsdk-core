
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("epwm");

    return system.getScript(`/drivers/epwm/${driverVer}/epwm_${driverVer}`);
}

exports = getModule();
