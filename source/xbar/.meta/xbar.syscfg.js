
let common = system.getScript("/common");
let soc = system.getScript(`/xbar/soc/xbar_${common.getSocName()}`);

exports = {
    displayName: "XBAR",
    topModules: soc.getTopModules(),
};
