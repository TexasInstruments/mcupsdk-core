
let common = system.getScript("/common");
let soc = system.getScript(`/optiflash/soc/optiflash_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("flc");

    return system.getScript(`/optiflash/FLC/${driverVer}/FLC_${driverVer}`);
}

exports = getModule();
