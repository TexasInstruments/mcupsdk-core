
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("edma");

    return system.getScript(`/drivers/edma/${driverVer}/edma_${driverVer}`);
}

exports = getModule();
