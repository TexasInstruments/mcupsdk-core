
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("mibspi");

    return system.getScript(`/drivers/mibspi/${driverVer}/mibspi_${driverVer}`);
}

exports = getModule();
