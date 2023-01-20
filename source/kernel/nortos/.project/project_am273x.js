let path = require('path');

const files_r5f = {
    common: [
        "boot_armv7r.c",
        "AddrTranslateP_null.c",
        "CacheP_armv7r.c",
        "ClockP_nortos.c",
        "ClockP_nortos_r5.c",
        "DebugP_log.c",
        "DebugP_nortos.c",
        "DebugP_shmLogWriter.c",
		"DebugP_shmLogReader.c",
        "DebugP_shmLogReader.c",
        "DebugP_uartLogWriter.c",
        "DebugP_uartScanf.c",
        "DebugP_memTraceLogWriter.c",
        "HeapP_internal.c",
        "CpuId_armv7r.c",
        "EventP_nortos.c",
        "HeapP_nortos.c",
        "HwiP_armv7r_vim.c",
        "HwiP_armv7r_handlers_nortos.c",
        "MpuP_armv7r.c",
        "PmuP_armv7r.c",
        "printf.c",
        "QueueP_nortos.c",
        "SemaphoreP_nortos.c",
        "TaskP_nortos.c",
        "TimerP_rti.c",
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
        "boot_armv7r_asm.S",
        "CacheP_armv7r_asm.S",
        "HwiP_armv7r_asm.S",
        "HwiP_armv7r_handlers_nortos_asm.S",
        "HwiP_armv7r_vectors_nortos_asm.S",
        "HwiP_armv7r_vectors_nortos_sbl_asm.S",
        "MpuP_armv7r_asm.S",
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

const files_c66 = {
    common: [
        "AddrTranslateP_null.c",
        "CacheP_c66.c",
        "ClockP_nortos.c",
        "ClockP_nortos_c66.c",
        "CycleCounterP_c66.c",
        "DebugP_log.c",
        "DebugP_nortos.c",
        "DebugP_shmLogWriter.c",
        "DebugP_shmLogReader.c",
        "DebugP_uartLogWriter.c",
        "DebugP_uartScanf.c",
        "DebugP_memTraceLogWriter.c",
        "HeapP_internal.c",
        "HeapP_nortos.c",
        "HwiP_c66.c",
        "HwiP_c66_handlers_nortos.c",
        "printf.c",
        "QueueP_nortos.c",
        "SemaphoreP_nortos.c",
        "TaskP_nortos.c",
        "TimerP_rti.c",
    ]
};

const cflags_c66 = {
    common: [

    ]
};

const asmfiles_c66 = {
    common: [
        "HwiP_c66_asm.S",
        "HwiP_c66_vectors_nortos_asm.S",
    ],
};

const filedirs_c66 = {
    common: [
        "dpl/common",
        "dpl/c66",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang", os: "nortos"},
    { device: device, cpu: "c66", cgt: "ti-c6000",     os: "nortos"},
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
        build_property.cflags = cflags_r5f;
        build_property.defines = defines_r5f;
    }
    if(buildOption.cpu.match(/c66*/)) {
        build_property.files = files_c66;
        build_property.asmfiles = asmfiles_c66;
        build_property.filedirs = filedirs_c66;
        build_property.cflags = cflags_c66;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
