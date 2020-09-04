
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("adc");

    return system.getScript(`/drivers/adc/${driverVer}/adc_${driverVer}`);
}

exports = getModule();
