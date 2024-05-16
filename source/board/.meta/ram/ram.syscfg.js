
let common = system.getScript("/common");
let soc = system.getScript(`/board/soc/board_${common.getSocName()}`);

function getModule() {

    let ramDriverVer = soc.getDriverVer("ram");

    return system.getScript(`/board/ram/${ramDriverVer}/ram_${ramDriverVer}`);
}

exports = getModule();