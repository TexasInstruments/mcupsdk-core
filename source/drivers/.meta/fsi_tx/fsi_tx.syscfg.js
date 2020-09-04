
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("fsi_tx");

    return system.getScript(`/drivers/fsi_tx/${driverVer}/fsi_tx_${driverVer}`);
}

exports = getModule();
