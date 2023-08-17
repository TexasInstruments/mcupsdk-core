
let common = system.getScript("/common");

function getIntcBaseAddr() {

    return system.getScript(`/kernel/dpl/hwi_${common.getSocName()}`).getIntcBaseAddr();
}

function getHwiMaxPriority() {
    let maxPriority = 15;
    if(common.getSelfSysCfgCoreName().includes("m4f")) {
        maxPriority = 7;
    }
	if(common.getSelfSysCfgCoreName().includes("hsm")) {
        maxPriority = 7;
    }

    return maxPriority;
}

function getPriorityConfigSupported()
{
    let isPriorityConfigSupported = true;

    if(common.getSelfSysCfgCoreName().includes("c66"))
    {
        isPriorityConfigSupported = false;
    }

    return isPriorityConfigSupported;
}

function getHwiDefaultPriority() {
    let defaultPriority = 15;
    
    if(common.getSelfSysCfgCoreName().includes("m4f")) {
        defaultPriority = 7;
    }
    if(common.getSelfSysCfgCoreName().includes("hsm")) {
        defaultPriority = 7;
    }
	if(common.getSelfSysCfgCoreName().includes("a53")) {
        defaultPriority = 9;
    }

    return defaultPriority;
}

exports = {
    getIntcBaseAddr,
    getHwiMaxPriority,
    getPriorityConfigSupported,
    getHwiDefaultPriority,
};

