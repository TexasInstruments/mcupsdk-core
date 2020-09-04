
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("pcie");

    return system.getScript(`/drivers/pcie/${driverVer}/pcie_${driverVer}`);
}

exports = getModule();
