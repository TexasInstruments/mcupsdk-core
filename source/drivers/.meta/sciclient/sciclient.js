
let common = system.getScript("/common");

function getCpuID() {
    return system.getScript(`/drivers/soc/drivers_${common.getSocName()}`).getCpuID();
}

let sciclient_module = {
    getCpuID,
};

exports = sciclient_module;
