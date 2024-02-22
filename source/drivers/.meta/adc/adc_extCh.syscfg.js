
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("adc_extCh");

    return system.getScript(`/drivers/adc/${driverVer}/adc_extCh_${driverVer}`);
}

exports = getModule();
