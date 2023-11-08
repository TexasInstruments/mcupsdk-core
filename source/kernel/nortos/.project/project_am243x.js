let path = require('path');

const files_r5f = {
    common: [
        "boot_armv7r.c",
        "DebugP_shmLogWriter.c",
		"DebugP_shmLogReader.c",
        "DebugP_uartLogWriter.c",
        "DebugP_uartScanf.c",
        "DebugP_memTraceLogWriter.c",
        "DebugP_log.c",
        "DebugP_nortos.c",
        "SemaphoreP_nortos.c",
        "ClockP_nortos.c",
        "ClockP_nortos_r5.c",
        "EventP_nortos.c",
        "HeapP_nortos.c",
        "TimerP.c",
        "TaskP_nortos.c",
        "HwiP_armv7r_vim.c",
        "HwiP_armv7r_handlers_nortos.c",
        "MpuP_armv7r.c",
        "CacheP_armv7r.c",
        "CpuId_armv7r.c",
        "HeapP_internal.c",
        "printf.c",
        "AddrTranslateP.c",
        "PmuP_armv7r.c",
        "QueueP_nortos.c",
    ]
};

const defines_r5f = {
    common: [
        "OS_NORTOS"
    ]
};

const cflags_r5f = {
    common: [
        "-Wno-extra"
    ]
};

const asmfiles_r5f = {
    common: [
        "HwiP_armv7r_handlers_nortos_asm.S",
        "HwiP_armv7r_vectors_nortos_asm.S",
        "HwiP_armv7r_vectors_nortos_sbl_asm.S",
        "boot_armv7r_asm.S",
        "HwiP_armv7r_asm.S",
        "MpuP_armv7r_asm.S",
        "CacheP_armv7r_asm.S",
        "CpuId_armv7r_asm.S",
        "PmuP_armv7r_asm.S",
    ],
};

const filedirs_r5f = {
    common: [
        "dpl/r5",
        "dpl/common",
    ],
};

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
    { device: device, cpu: "r5f", cgt: "ti-arm-clang", os: "nortos"},
    { device: device, cpu: "r5f", cgt: "gcc-armv7",    os: "nortos"},
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

    if(buildOption.cpu.match(/r5f*/)) {
        build_property.files = files_r5f;
        build_property.asmfiles = asmfiles_r5f;
        build_property.filedirs = filedirs_r5f;
        if(buildOption.cgt.match(/ti-arm-clang*/)) {
            build_property.cflags = cflags_r5f;
        }
        build_property.defines = defines_r5f;
    }
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
