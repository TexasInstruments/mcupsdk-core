let common = system.getScript("/common");

const topModules_main = [
    "/kernel/dpl/addr_translate",
    "/kernel/dpl/clock",
    "/kernel/dpl/debug_log",
    "/kernel/dpl/mpu_armv7",
    "/kernel/dpl/timer",
    "/kernel/dpl/profile",
];

const topModules_mcu = [
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
        if (common.getSelfSysCfgCoreName().match(/r5f*/)) {
            return topModules_main;
        }
        if (common.getSelfSysCfgCoreName().match(/m4f*/)) {
            return topModules_mcu;
        }
    },
};
