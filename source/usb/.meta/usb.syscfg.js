
let common = system.getScript("/common");
let soc = system.getScript(`/usb/soc/usb_${common.getSocName()}`);

exports = {
    displayName: "USB",
    topModules: soc.getTopModules(),
};
