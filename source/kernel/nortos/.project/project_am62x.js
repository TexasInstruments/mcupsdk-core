let path = require('path');

const files_m4f = {
    common: [
        "boot_armv7m.c",
        "CacheP_null.c",
        "DebugP_shmLogWriter.c",
        "DebugP_uartLogWriter.c",
        "DebugP_uartScanf.c",
        "DebugP_memTraceLogWriter.c",
        "DebugP_log.c",
        "DebugP_nortos.c",
        "SemaphoreP_nortos.c",
        "ClockP_nortos.c",
        "ClockP_nortos_m4.c",
        "HeapP_nortos.c",
        "HeapP_internal.c",
        "printf.c",
        "HwiP_armv7m.c",
        "HwiP_armv7m_handlers_nortos.c",
        "SysTickTimerP.c",
        "TimerP.c",
        "TaskP_nortos.c",
        "MpuP_armv7m.c",
        "AddrTranslateP.c",
        "CycleCounterP_m4.c",
        "QueueP_nortos.c",
    ]
};

const cflags_m4f = {
    common: [

    ]
};

const asmfiles_m4f = {
    common: [
    ],
};

const filedirs_m4f = {
    common: [
        "dpl/common",
        "dpl/m4",
    ],
};


const buildOptionCombos = [
    { device: device, cpu: "m4f", cgt: "ti-arm-clang", os: "nortos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "nortos";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    if(buildOption.cpu.match(/m4f*/)) {
        build_property.files = files_m4f;
        build_property.asmfiles = asmfiles_m4f;
        build_property.filedirs = filedirs_m4f;
        build_property.cflags = cflags_m4f;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
