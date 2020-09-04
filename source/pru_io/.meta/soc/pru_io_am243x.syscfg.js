
let common = system.getScript("/common");

const topModules_main = [
    "/pru_io/adc/adc",
    "/pru_io/pru_ipc/pru_ipc",
];

const topModules_mcu = [

];

exports = {
    getTopModules: function() {

        let topModules = topModules_main;

        if(common.getSelfSysCfgCoreName().includes("m4f")) {
            topModules = topModules_mcu;
        }

        return topModules;
    },
};
