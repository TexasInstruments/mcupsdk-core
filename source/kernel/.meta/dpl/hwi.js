
let common = system.getScript("/common");

function getIntcBaseAddr() {

    return system.getScript(`/kernel/dpl/hwi_${common.getSocName()}`).getIntcBaseAddr();
}

function getHwiMaxPriority() {
    let maxPriority = 15;
    if(common.getSelfSysCfgCoreName().includes("m4f")) {
        maxPriority = 7;
    }

    return maxPriority;
}

exports = {
    getIntcBaseAddr,
    getHwiMaxPriority,
};

