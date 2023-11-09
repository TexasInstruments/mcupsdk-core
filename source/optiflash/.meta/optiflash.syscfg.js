
let common = system.getScript("/common");
let soc = system.getScript(`/optiflash/soc/optiflash_${common.getSocName()}`);

exports = {
    displayName: "OptiFlash",
    topModules: soc.getTopModules(),
};
