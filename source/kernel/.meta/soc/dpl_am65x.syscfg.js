let common = system.getScript("/common");

const topModules = [
    "/kernel/dpl/addr_translate",
    "/kernel/dpl/clock",
    "/kernel/dpl/debug_log",
    "/kernel/dpl/mpu_armv7",
    "/kernel/dpl/timer",
];

exports = {
    getTopModules: function() {
        return topModules;
    },
};
