
let common = system.getScript("/common");
let soc = system.getScript(`/board/soc/board_${common.getSocName()}`);

function getModule() {

    let flashDriverVer = soc.getDriverVer("flash");

    return system.getScript(`/board/flash/${flashDriverVer}/flash_${flashDriverVer}`);
}

exports = getModule();
