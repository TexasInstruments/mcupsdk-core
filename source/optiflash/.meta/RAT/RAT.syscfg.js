
let common = system.getScript("/common");
let soc = system.getScript(`/optiflash/soc/optiflash_${common.getSocName()}`);

function getModule()
{
    let driverVer = soc.getDriverVer("rat");
    return system.getScript(`/optiflash/RAT/${driverVer}/rat_${driverVer}`);
}

exports = getModule();