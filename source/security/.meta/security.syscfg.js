
let common = system.getScript("/common");
let soc = system.getScript(`/security/soc/security_${common.getSocName()}`);

let selfCoreName = common.getSelfSysCfgCoreName();

let SecServicesObj = {}

if (!selfCoreName.includes("hsm")) {
    SecServicesObj = {
        displayName: "TI HSM Services",
        topModules: soc.getTopModules(),
    };

}
exports = SecServicesObj;