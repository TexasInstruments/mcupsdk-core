
let common = system.getScript("/common");
let soc = system.getScript(`/motor_control/soc/motor_control_${common.getSocName()}`);

exports = {
    displayName: "TI Motor Control Protocols",
    topModules: soc.getTopModules(),
};
