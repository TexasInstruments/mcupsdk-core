let common = system.getScript("/common");
let soc = system.getScript(`/security/soc/security_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("firewall_service");

    return system.getScript(`/security/firewall_service/${driverVer}/firewall_service_${driverVer}`);
}

exports = getModule();
