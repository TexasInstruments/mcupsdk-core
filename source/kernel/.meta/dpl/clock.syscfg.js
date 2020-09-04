
let common = system.getScript("/common");
let soc = system.getScript(`/kernel/dpl/clock_${common.getSocName()}`);

function getInstanceConfig(moduleInstance) {
    let staticConfigArr = soc.getStaticConfigArr();
    let staticConfig = staticConfigArr.find( o => o.name === moduleInstance.instance);

    let clkSelMuxValue = soc.getTimerClockSourceValue(moduleInstance);

    return {
        ...staticConfig,
        ...moduleInstance,
        clkSelMuxValue : clkSelMuxValue,
    }
};

let clock_module = {
    displayName: "Clock",
    longDescription: "Make sure one and only one clock instance is added and setup properly to generate timer ticks at required frequency",

    templates: {
        "/kernel/dpl/dpl_config.c.xdt": {
            dpl_config: "/kernel/dpl/clock.c.xdt",
            dpl_init: "/kernel/dpl/clock_init.c.xdt",
        },
        "/kernel/dpl/dpl_config.h.xdt": "/kernel/dpl/clock.h.xdt",
    },

    moduleStatic : {
        config: [
            soc.makeInstanceConfig(),
            {
                name: "timerInputClkHz",
                displayName: "Input Clock Frequency (Hz)",
                default: soc.getDefaultTimerClockSourceMhz(),
                description: "Make sure the value put here matches the actual input clock frequency provided on the board/SOC"
            },
            soc.getTimerClockSourceConfigArr(),
            {
                name: "usecPerTick",
                displayName: "Tick Period (usecs)",
                default: 1000,
                description: "Timer tick period in units of usecs. MUST be >= 100 and <= 1000000. MUST divide 1000000, i.e 1sec in integer units.",
            },
        ],

        validate : function (instance, report) {
            common.validate.checkNumberRange(instance, report, "usecPerTick", 100, 1000000, "dec");
            common.validate.checkNumberRange(instance, report, "timerInputClkHz", 32000, 1000000000, "dec");

            if( (1000000 % instance.usecPerTick) != 0) {
                report.logError( `Tick period must divide 1000000 usecs (1sec) in integer multiples`, instance, "usecPerTick");
            }
        },
    },

    getInstanceConfig,
};

exports = clock_module;
