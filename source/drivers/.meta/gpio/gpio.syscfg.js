
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("gpio");

    return system.getScript(`/drivers/gpio/${driverVer}/gpio_${driverVer}`);
}

exports = getModule();
