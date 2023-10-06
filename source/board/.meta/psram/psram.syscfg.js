
let common = system.getScript("/common");
let soc = system.getScript(`/board/soc/board_${common.getSocName()}`);

function getModule() {

    let psramDriverVer = soc.getDriverVer("psram");

    return system.getScript(`/board/psram/${psramDriverVer}/psram_${psramDriverVer}`);
}

exports = getModule();