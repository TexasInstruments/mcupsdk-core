
let common = system.getScript("/common");

let ratBaseAddr_m4f = 0x44200000;
let ratBaseAddr_r5f = 0x2FFE0000;

let longDescription_m4f = 
`M4F needs a Region Address Translate (RAT) to be programed to access certain
peripheral and memory regions esp those in the MainSS. Refer to the device TRM for regions that need a 
RAT translation mapping.`
    ;

let longDescription_r5f = 
`R5F has a Region Address Translate (RAT) to access memory and peripheral regions beyond 32b address space. 
In most cases for R5F, there is no need to program RAT for R5F. Refer to the device TRM for regions that 
need a RAT translation mapping.`
    ;

function getRatBaseAddr() {
    
    let cpu = common.getSelfSysCfgCoreName();
    let ratAddr = ratBaseAddr_r5f;

    if(cpu.match(/m4f*/))
    {
        ratAddr = ratBaseAddr_m4f;
    }
    return ratAddr;
}

function getLongDescription() {

    let cpu = common.getSelfSysCfgCoreName();
    let longDescription = longDescription_r5f;

    if(cpu.match(/m4f*/))
    {
        longDescription = longDescription_m4f;
    }
    return longDescription;    
}

exports = {
    getRatBaseAddr,
    getLongDescription,
};

