
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("mcasp");

    return system.getScript(`/drivers/mcasp/${driverVer}/mcasp_${driverVer}`);
}

exports = getModule();
