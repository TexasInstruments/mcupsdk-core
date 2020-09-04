let common = system.getScript("/common");

const topModules = [
    "/kernel/dpl/addr_translate",
    "/kernel/dpl/clock",
    "/kernel/dpl/debug_log",
    "/kernel/dpl/mpu_armv7",
    "/kernel/dpl/timer",
];

const topModules_a53 = [
    "/kernel/dpl/clock",
    "/kernel/dpl/debug_log",
    "/kernel/dpl/mmu_armv8",
    "/kernel/dpl/timer",
];

exports = {
    getTopModules: function() {
        if (common.getSelfSysCfgCoreName().match(/a53*/)) {
            return topModules_a53;
        }
        return topModules;
    },
};
