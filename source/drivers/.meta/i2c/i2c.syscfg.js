
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("i2c");

    return system.getScript(`/drivers/i2c/${driverVer}/i2c_${driverVer}`);
}

exports = getModule();
