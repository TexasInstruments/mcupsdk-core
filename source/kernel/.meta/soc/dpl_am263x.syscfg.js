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
    "/kernel/dpl/clock",
    "/kernel/dpl/debug_log",
    "/kernel/dpl/mpu_armv7",
    "/kernel/dpl/timer",
];

exports = {
    getTopModules: function() {
        return common.getSelfSysCfgCoreName().includes("hsm")?topModules_mcu:topModules_main;
    },
};
