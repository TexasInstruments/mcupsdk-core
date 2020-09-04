
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("ospi");

    return system.getScript(`/drivers/ospi/${driverVer}/ospi_${driverVer}`);
}

exports = getModule();
