
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("csirx");

    return system.getScript(`/drivers/csirx/${driverVer}/csirx_${driverVer}`);
}

exports = getModule();
