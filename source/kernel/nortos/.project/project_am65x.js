let path = require('path');

const files_r5f = {
    common: [
        "AddrTranslateP.c",
        "boot_armv7r.c",
        "CacheP_armv7r.c",
        "ClockP_nortos.c",
        "ClockP_nortos_r5.c",
        "CpuId_armv7r.c",
        "DebugP_log.c",
        "DebugP_memTraceLogWriter.c",
        "DebugP_nortos.c",
        "DebugP_shmLogWriter.c",
        "DebugP_shmLogReader.c",
        "DebugP_uartLogWriter.c",
        "EventP_nortos.c",
        "HeapP_internal.c",
        "HeapP_nortos.c",
        "HwiP_armv7r_handlers_nortos.c",
        "HwiP_armv7r_vim.c",
        "MpuP_armv7r.c",
        "printf.c",
        "PmuP_armv7r.c",
        "QueueP_nortos.c",
        "SemaphoreP_nortos.c",
        "TaskP_nortos.c",
        "TimerP.c",
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
        "CpuId_armv7r_asm.S",
        "HwiP_armv7r_handlers_nortos_asm.S",
        "HwiP_armv7r_vectors_nortos_asm.S",
        "HwiP_armv7r_vectors_nortos_sbl_asm.S",
        "HwiP_armv7r_asm.S",
        "MpuP_armv7r_asm.S",
        "PmuP_armv7r_asm.S",
    ],
};

const filedirs_r5f = {
    common: [
        "dpl/common",
        "dpl/r5",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang", os: "nortos"},
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

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
