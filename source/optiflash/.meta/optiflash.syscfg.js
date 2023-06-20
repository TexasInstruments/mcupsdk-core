
let common = system.getScript("/common");
let soc = system.getScript(`/optiflash/soc/optiflash_${common.getSocName()}`);

exports = {
    displayName: "OpTI-Flash",
    topModules: soc.getTopModules(),
};
