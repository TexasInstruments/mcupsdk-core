
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("fsi_rx");

    return system.getScript(`/drivers/fsi_rx/${driverVer}/fsi_rx_${driverVer}`);
}

exports = getModule();
