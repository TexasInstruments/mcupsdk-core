let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("bootloader");

    return system.getScript(`/drivers/bootloader/${driverVer}/bootloader_${driverVer}`);
}

exports = getModule();
