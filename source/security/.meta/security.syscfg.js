
let common = system.getScript("/common");
let soc = system.getScript(`/security/soc/security_${common.getSocName()}`);

exports = {
    displayName: "TI HSM Services",
    topModules: soc.getTopModules(),
};
