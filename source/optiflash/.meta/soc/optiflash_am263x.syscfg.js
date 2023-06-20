
let common = system.getScript("/common");

const topModules_main = [];

exports = {
    getTopModules: function() {
        return common.getSelfSysCfgCoreName().includes("r5f") ? topModules_main:[];
    }
};
