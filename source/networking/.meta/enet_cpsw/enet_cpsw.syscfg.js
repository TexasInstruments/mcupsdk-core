
let common = system.getScript("/common");
let soc = system.getScript(`/networking/soc/networking_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("enet_cpsw");

    return system.getScript(`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}`);
}

exports = getModule();