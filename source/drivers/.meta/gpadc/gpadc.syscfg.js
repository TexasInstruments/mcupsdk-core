
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("gpadc");

    return system.getScript(`/drivers/gpadc/${driverVer}/gpadc_${driverVer}`);
}

exports = getModule();
