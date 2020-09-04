
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("uart");

    return system.getScript(`/drivers/uart/${driverVer}/uart_${driverVer}`);
}

exports = getModule();
