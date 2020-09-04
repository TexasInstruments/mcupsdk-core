
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("ddr");

    return system.getScript(`/drivers/ddr/${driverVer}/ddr_${driverVer}`);
}

exports = getModule();
