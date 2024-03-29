
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("adcbuf");

    return system.getScript(`/drivers/adcbuf/${driverVer}/adcbuf_${driverVer}`);
}

exports = getModule();
