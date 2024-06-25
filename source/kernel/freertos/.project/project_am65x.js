let path = require('path');

const files_r5f = {
    common: [
        // picked from nortos DPL
        "AddrTranslateP.c",
        "boot_armv7r.c",
        "CacheP_armv7r.c",
        "DebugP_shmLogWriter.c",
        "DebugP_shmLogReader.c",
        "DebugP_log.c",
        "DebugP_uartLogWriter.c",
        "HwiP_armv7r_vim.c",
        "HeapP_internal.c",
        "MpuP_armv7r.c",
        "printf.c",
        "PmuP_armv7r.c",
        "QueueP_nortos.c",
        "TimerP.c",
        // picked from freertos DPL
        "ClockP_freertos.c",
        "ClockP_freertos_r5.c",
        "MailboxP_freertos.c",
        "DebugP_freertos.c",
        "DebugP_memTraceLogWriter.c",
        "DebugP_shmLogReader_freertos.c",
        "EventP_freertos.c",
        "HwiP_armv7r_handlers_freertos.c",
        "HeapP_freertos.c",
        "SemaphoreP_freertos.c",
        "TaskP_freertos.c",
        "MailboxP_freertos.c",
        // picked from freertos "kernel"
        "event_groups.c",
        "heap_3.c",
        "list.c",
        "queue.c",
        "tasks.c",
        "timers.c",
        // picked from freertos "posix"
        "FreeRTOS_POSIX_clock.c",
        "FreeRTOS_POSIX_mqueue.c",
        "FreeRTOS_POSIX_pthread_barrier.c",
        "FreeRTOS_POSIX_pthread_cond.c",
        "FreeRTOS_POSIX_pthread_mutex.c",
        "FreeRTOS_POSIX_pthread.c",
        "FreeRTOS_POSIX_sched.c",
        "FreeRTOS_POSIX_semaphore.c",
        "FreeRTOS_POSIX_timer.c",
        "FreeRTOS_POSIX_unistd.c",
        "FreeRTOS_POSIX_utils.c",
        // picked from freertos "portable"
        "port.c",
    ]
};

const includes_r5f = {
    common: [
        "config/am65x/r5f",
        "portable/TI_ARM_CLANG/ARM_CR5F",
        "FreeRTOS-Kernel/include",
        "FreeRTOS-POSIX/include",
        "FreeRTOS-POSIX/include/private",
        "FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include",
        "FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include/portable",
    ],
};

const cflags_r5f = {
    common: [
        "-Wno-extra"
    ]
};

const asmfiles_r5f = {
    common: [
        // picked from nortos DPL
        "boot_armv7r_asm.S",
        "CacheP_armv7r_asm.S",
        "CpuId_armv7r_asm.S",
        "HwiP_armv7r_asm.S",
        "MpuP_armv7r_asm.S",
        "PmuP_armv7r_asm.S",
        // picked from freertos DPL
        "HwiP_armv7r_vectors_freertos_asm.S",
        "HwiP_armv7r_handlers_freertos_asm.S",
        // picked from freertos "portable"
        "portASM.S",
    ],
};

const filedirs_r5f = {
    common: [
        // picked from nortos DPL
        "../nortos/dpl/r5",
        "../nortos/dpl/common",
        // picked from freertos DPL
        "dpl/common",
        "dpl/r5",
        // picked from freertos "kernel"
        "FreeRTOS-Kernel/",
        "FreeRTOS-Kernel/portable/MemMang",
        // picked from freertos "posix"
        "FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/source/",
        // picked from freertos "portable"
        "portable/TI_ARM_CLANG/ARM_CR5F",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang", os: "freertos"},
];

const templates_freertos_r5f =
[
    {
        input: ".project/templates/am65x/freertos/FreeRTOSConfig.h.xdt",
        output: "config/am65x/r5f/FreeRTOSConfig.h",
        options: {

        },
    }
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "freertos";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    if(buildOption.cpu == "r5f") {
        build_property.files = files_r5f;
        build_property.includes = includes_r5f;
        build_property.asmfiles = asmfiles_r5f;
        build_property.filedirs = filedirs_r5f;
        build_property.cflags = cflags_r5f;
        build_property.templates = templates_freertos_r5f;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
