
let common = system.getScript("/common");

const topModules_main = [

];

exports = {
    getTopModules: function() {

        let topModules = topModules_main;

        return common.getSelfSysCfgCoreName().includes("r5f")||common.getSelfSysCfgCoreName().includes("c66")?topModules_main:[];
    },
};