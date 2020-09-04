
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("crc");

    return system.getScript(`/drivers/crc/${driverVer}/crc_${driverVer}`);
}

exports = getModule();
