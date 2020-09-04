
let common = system.getScript("/common");

const topModules_main = [
    "/security/crypto/crypto",
    "/security/sa2ul/sa2ul",
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
