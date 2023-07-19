let common = system.getScript("/common");

const topModules_main = [
    "/kernel/dpl/addr_translate",
    "/kernel/dpl/clock",
    "/kernel/dpl/debug_log",
    "/kernel/dpl/mpu_armv7",
    "/kernel/dpl/timer",
    "/kernel/dpl/profile",
];

const topModules = [
    "/kernel/dpl/addr_translate",
    "/kernel/dpl/clock",
    "/kernel/dpl/debug_log",
    "/kernel/dpl/mpu_armv7",
    "/kernel/dpl/timer",
];

exports = {
    getTopModules: function() {
        if (common.getSelfSysCfgCoreName().match(/r5f*/)) {
            return topModules_main;
        }
        return topModules;
    },
};
