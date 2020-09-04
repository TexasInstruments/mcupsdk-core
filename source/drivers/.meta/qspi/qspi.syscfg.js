
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("qspi");

    return system.getScript(`/drivers/qspi/${driverVer}/qspi_${driverVer}`);
}

exports = getModule();
