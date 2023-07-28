
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("mcu_bist");

    return system.getScript(`/drivers/mcu_bist/${driverVer}/mcu_bist_${driverVer}`);
}

exports = getModule();
