
let common = system.getScript("/common");

const topModules = [

];

exports = {
    getTopModules: function() {
        return common.getSelfSysCfgCoreName().includes("r5f")||common.getSelfSysCfgCoreName().includes("c66")?topModules:[];
    },
};
