
let common = system.getScript("/common");
let soc = system.getScript(`/industrial_comms/soc/industrial_comms_${common.getSocName()}`);

exports = {
    displayName: "TI Industrial Comms Toolkit",
    topModules: soc.getTopModules(),
};
