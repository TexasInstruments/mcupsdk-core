
let common = system.getScript("/common");

let intcBaseAddr_r5f = 0x2FFF0000;

function getIntcBaseAddr() {
    
    let cpu = common.getSelfSysCfgCoreName();
    let intcAddr = intcBaseAddr_r5f;

    if(cpu.match(/r5f*/))
    {
        intcAddr = intcBaseAddr_r5f;
    }
    return intcAddr;
}

exports = {
    getIntcBaseAddr,
};

