
let common = system.getScript("/common");

const topModules_main = [
    "/usb/tinyusb/tinyusb",
];

const topModules_mcu = [

];

const topModules_a53 = [

];

exports = {
    getTopModules: function() {

        let topModules = topModules_main;

        if(common.getSelfSysCfgCoreName().includes("m4f")) {
            topModules = topModules_mcu;
        }
        if (common.getSelfSysCfgCoreName().match(/a53*/))
        {
            topModules = topModules_a53;
        }

        return topModules;
    },
};
