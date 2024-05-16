let common = system.getScript("/common");
let soc = system.getScript(`/board/soc/board_${common.getSocName()}`);

function getModule() {

    let ramDriverVer = soc.getDriverVer("parallelRam");

    return system.getScript(`/board/ram/parallelRam/${ramDriverVer}/parallelram_${ramDriverVer}`);
}

exports = getModule();