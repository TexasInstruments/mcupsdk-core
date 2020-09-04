
let common = system.getScript("/common");

let ratBaseAddr_m4f = 0x44200000;

let longDescription_m4f =
`M4F needs a Region Address Translate (RAT) to be programed to access certain
peripheral and memory regions esp those in the MainSS. Refer to the device TRM for regions that need a
RAT translation mapping.`
    ;


function getRatBaseAddr() {

    let cpu = common.getSelfSysCfgCoreName();
    let ratAddr;

    if(cpu.match(/m4f*/))
    {
        ratAddr = ratBaseAddr_m4f;
    }
    return ratAddr;
}

function getLongDescription() {

    let cpu = common.getSelfSysCfgCoreName();
    let longDescription;

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

