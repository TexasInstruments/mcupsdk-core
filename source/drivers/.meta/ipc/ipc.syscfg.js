
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("ipc");

    return system.getScript(`/drivers/ipc/${driverVer}/ipc_${driverVer}`);
}

exports = getModule();
