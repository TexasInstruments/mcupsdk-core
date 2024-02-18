let common = system.getScript("/common");
let soc = system.getScript(`/security/soc/security_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("keyring_import");

    return system.getScript(`/security/keyring_import/${driverVer}/keyring_import_${driverVer}`);
}

exports = getModule();
