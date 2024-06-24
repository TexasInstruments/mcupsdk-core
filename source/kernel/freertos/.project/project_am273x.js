let path = require('path');

const files_r5f = {
    common: [
        // picked from nortos DPL
        "boot_armv7r.c",
        "AddrTranslateP_null.c",
        "DebugP_shmLogWriter.c",
        "DebugP_shmLogReader.c",
        "DebugP_uartLogWriter.c",
        "DebugP_uartScanf.c",
        "DebugP_memTraceLogWriter.c",
        "DebugP_log.c",
        "TimerP_rti.c",
        "HwiP_armv7r_vim.c",
        "MpuP_armv7r.c",
        "CacheP_armv7r.c",
        "CpuId_armv7r.c",
        "HeapP_internal.c",
        "printf.c",
        "PmuP_armv7r.c",
        "QueueP_nortos.c",
        // picked from freertos DPL
        "ClockP_freertos.c",
        "ClockP_freertos_r5.c",
        "DebugP_freertos.c",
        "DebugP_shmLogReader_freertos.c",
        "HeapP_freertos.c",
        "HwiP_armv7r_handlers_freertos.c",
        "SemaphoreP_freertos.c",
        "TaskP_freertos.c",
        "EventP_freertos.c",
        "MailboxP_freertos.c",
        // picked from freertos "kernel"
        "event_groups.c",
        "tasks.c",
        "timers.c",
        "queue.c",
        "list.c",
        "heap_3.c",
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

const files_c66 = {
    common: [
        // picked from nortos DPL
        "AddrTranslateP_null.c",
        "CacheP_c66.c",
        "CycleCounterP_c66.c",
        "DebugP_shmLogWriter.c",
        "DebugP_shmLogReader.c",
        "DebugP_uartLogWriter.c",
        "DebugP_uartScanf.c",
        "DebugP_memTraceLogWriter.c",
        "DebugP_log.c",
        "HeapP_internal.c",
        "HwiP_c66.c",
        "printf.c",
        "QueueP_nortos.c",
        "TimerP_rti.c",
        // picked from freertos DPL
        "ClockP_freertos.c",
        "ClockP_freertos_c66.c",
        "DebugP_freertos.c",
        "DebugP_shmLogReader_freertos.c",
        "HeapP_freertos.c",
        "HwiP_c66_handlers_freertos.c",
        "SemaphoreP_freertos.c",
        "TaskP_freertos.c",
        "EventP_freertos.c",
        "MailboxP_freertos.c",
        // picked from freertos "kernel"
        "event_groups.c",
        "tasks.c",
        "timers.c",
        "queue.c",
        "list.c",
        "heap_3.c",
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
        "port_hwi.c",
    ]
};

const includes_r5f = {
    common: [
        "FreeRTOS-Kernel/include",
        "portable/TI_ARM_CLANG/ARM_CR5F",
        "config/am273x/r5f",
        "FreeRTOS-POSIX/include",
        "FreeRTOS-POSIX/include/private",
        "FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include",
        "FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include/portable",
    ],
};

const includes_c66 = {
    common: [
        "FreeRTOS-Kernel/include",
        "portable/TI_CGT/DSP_C66",
        "config/am273x/c66",
        "FreeRTOS-POSIX/include",
        "FreeRTOS-POSIX/include/private",
        "FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include",
        "FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include/portable",
    ],
};

const cflags_r5f = {
    common: [
        "-Wno-extra"
    ],
    release: [
        "-Oz",
        "-flto",
    ],
};

const cflags_c66 = {
    common: [
    ]
};

const asmfiles_r5f = {
    common: [
        // picked from nortos DPL
        "boot_armv7r_asm.S",
        "HwiP_armv7r_asm.S",
        "MpuP_armv7r_asm.S",
        "CacheP_armv7r_asm.S",
        "CpuId_armv7r_asm.S",
        "PmuP_armv7r_asm.S",
        "Mutex_armv7r_asm.S",
        // picked from freertos DPL
        "HwiP_armv7r_vectors_freertos_asm.S",
        "HwiP_armv7r_handlers_freertos_asm.S",
        // picked from freertos "portable"
        "portASM.S",
    ],
};

const asmfiles_c66 = {
    common: [
        // picked from nortos DPL
        "HwiP_c66_asm.S",
        // picked from freertos "portable"
        "port_hwi_disp_always.S",
        "port_hwi_switch.S",
        "port_hwi_vectors_freertos_asm.S",
        "port_tasksupport.S",
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

const filedirs_c66 = {
    common: [
        // picked from nortos DPL
        "../nortos/dpl/c66",
        "../nortos/dpl/common",
        // picked from freertos DPL
        "dpl/common",
        "dpl/c66",
        // picked from freertos "kernel"
        "FreeRTOS-Kernel/",
        "FreeRTOS-Kernel/portable/MemMang",
        // picked from freertos "posix"
        "FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/source/",
        // picked from freertos "portable"
        "portable/TI_CGT/DSP_C66",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang", os: "freertos"},
    { device: device, cpu: "c66", cgt: "ti-c6000",     os: "freertos"},
];

const templates_freertos_r5f =
[
    {
        input: ".project/templates/am273x/freertos/FreeRTOSConfig.h.xdt",
        output: "config/am273x/r5f/FreeRTOSConfig.h",
        options: {

        },
    }
];

const templates_freertos_c66 =
[
    {
        input: ".project/templates/am273x/freertos/FreeRTOSConfig.h.xdt",
        output: "config/am273x/c66/FreeRTOSConfig.h",
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

    if(buildOption.cpu == "c66") {
        build_property.files = files_c66;
        build_property.includes = includes_c66;
        build_property.asmfiles = asmfiles_c66;
        build_property.filedirs = filedirs_c66;
        build_property.cflags = cflags_c66;
        build_property.templates = templates_freertos_c66;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
