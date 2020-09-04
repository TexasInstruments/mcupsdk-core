let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("firewall");

    return system.getScript(`/drivers/firewall/${driverVer}/firewall_${driverVer}`);
}

exports = getModule();



