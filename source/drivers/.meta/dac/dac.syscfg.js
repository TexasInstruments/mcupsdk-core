
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("dac");

    return system.getScript(`/drivers/dac/${driverVer}/dac_${driverVer}`);
}

exports = getModule();
