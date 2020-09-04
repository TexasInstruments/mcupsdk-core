let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("mmcsd");

    return system.getScript(`/drivers/mmcsd/${driverVer}/mmcsd_${driverVer}`);
}

exports = getModule();
