
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("adc_sc");

    return system.getScript(`/drivers/adc/${driverVer}/adc_sc_${driverVer}`);
}

exports = getModule();