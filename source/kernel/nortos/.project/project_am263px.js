let path = require('path');

const files_r5f = {
    common: [
        "AddrTranslateP_32bit.c",
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
        "CpuId_armv7r.c",
        "EventP_nortos.c",
        "HeapP_nortos.c",
        "TimerP_rti.c",
        "TaskP_nortos.c",
        "HwiP_armv7r_vim.c",
        "HwiP_armv7r_handlers_nortos.c",
        "MpuP_armv7r.c",
        "CacheP_armv7r.c",
        "HeapP_internal.c",
        "printf.c",
        "QueueP_nortos.c",
        "PmuP_armv7r.c",
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
        "Mutex_armv7r_asm.S",

    ],
};

const filedirs_r5f = {
    common: [
        "dpl/r5",
        "dpl/common",
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
