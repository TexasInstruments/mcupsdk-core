let path = require('path');

const files_r5f_common = {
    common: [
        // picked from nortos DPL
        "boot_armv7r.c",
        "DebugP_shmLogWriter.c",
        "DebugP_shmLogReader.c",
        "DebugP_uartLogWriter.c",
        "DebugP_uartScanf.c",
        "DebugP_memTraceLogWriter.c",
        "DebugP_log.c",
        "TimerP.c",
        "HwiP_armv7r_vim.c",
        "MpuP_armv7r.c",
        "CacheP_armv7r.c",
        "CpuId_armv7r.c",
        "HeapP_internal.c",
        "printf.c",
        "AddrTranslateP.c",
        "PmuP_armv7r.c",
        "QueueP_nortos.c",
        // picked from freertos DPL
        "ClockP_freertos.c",
        "ClockP_freertos_r5.c",
        "SemaphoreP_freertos.c",
        "HwiP_armv7r_handlers_freertos.c",
        "HeapP_freertos.c",
        "DebugP_freertos.c",
        "DebugP_shmLogReader_freertos.c",
        "TaskP_freertos.c",
        "EventP_freertos.c",
        "MailboxP_freertos.c",
        // picked from freertos "kernel"
        "event_groups.c",
        "tasks.c",
        "timers.c",
        "queue.c",
        "list.c",
        "stream_buffer.c",
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
        // picked from freertos CLI
        "FreeRTOS_CLI.c",

    ]
};

const files_r5f = {
    common: [
        ...files_r5f_common.common,
        // picked from freertos "portable/TI_ARM_CLANG/ARM_CR5F"
        "port.c",
    ]
};

const files_r5f_mpu = {
    common: [
        ...files_r5f_common.common,
        // picked from freertos "portable/TI_ARM_CLANG/ARM_CR5F_MPU"
        "port.c",
        "sdk_mpu_wrappers.c",
        // picked from freertos "FreeRTOS-Kernel/portable/Common"
        "mpu_wrappers_v2.c"
    ]
};

const files_a53 = {
    common: [
        // picked from nortos DPL
        "boot_armv8.c",
        "DebugP_shmLogWriter.c",
        "DebugP_shmLogReader.c",
        "DebugP_uartLogWriter.c",
        "DebugP_uartScanf.c",
        "DebugP_memTraceLogWriter.c",
        "DebugP_log.c",
        "TimerP.c",
        "HwiP_armv8_gic.c",
        "MmuP_armv8.c",
        "CacheP_armv8.c",
        "HeapP_internal.c",
        "printf.c",
        "AddrTranslateP.c",
        "PmuP_armv8.c",
        "QueueP_nortos.c",
        // picked from freertos DPL
        "ClockP_freertos.c",
        "ClockP_freertos_a53.c",
        "SemaphoreP_freertos.c",
        "HwiP_armv8_handlers_freertos.c",
        "HeapP_freertos.c",
        "DebugP_freertos.c",
        "DebugP_shmLogReader_freertos.c",
        "TaskP_freertos.c",
        "EventP_freertos.c",
        "MailboxP_freertos.c",
        // picked from freertos "kernel"
        "event_groups.c",
        "tasks.c",
        "timers.c",
        "queue.c",
        "list.c",
        "stream_buffer.c",
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
        // picked from freertos CLI
        "FreeRTOS_CLI.c",
    ]
};

const files_a53_smp = {
    common: [
        // picked from nortos DPL
        "boot_armv8.c",
        "DebugP_shmLogWriter.c",
        "DebugP_shmLogReader.c",
        "DebugP_uartLogWriter.c",
        "DebugP_uartScanf.c",
        "DebugP_memTraceLogWriter.c",
        "DebugP_log.c",
        "TimerP.c",
        "HwiP_armv8_gic.c",
        "MmuP_armv8.c",
        "CacheP_armv8.c",
        "HeapP_internal.c",
        "printf.c",
        "AddrTranslateP.c",
        "PmuP_armv8.c",
        "QueueP_nortos.c",
        // picked from freertos DPL
        "ClockP_freertos.c",
        "ClockP_freertos_a53.c",
        "SemaphoreP_freertos.c",
        "HwiP_armv8_handlers_freertos.c",
        "HeapP_freertos.c",
        "DebugP_freertos.c",
        "DebugP_shmLogReader_freertos.c",
        "TaskP_freertos.c",
        "EventP_freertos.c",
        "MailboxP_freertos.c",
        // picked from freertos "kernel"
        "event_groups.c",
        "tasks.c",
        "timers.c",
        "queue.c",
        "list.c",
        "stream_buffer.c",
        "heap_4.c",
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
        // picked from freertos "portable_smp"
        "port.c",
        // picked from freertos CLI
        "FreeRTOS_CLI.c",
    ]
};

const files_m4f = {
    common: [
        // picked from nortos DPL
        "boot_armv7m.c",
        "DebugP_shmLogWriter.c",
        "DebugP_shmLogReader.c",
        "DebugP_uartLogWriter.c",
        "DebugP_uartScanf.c",
        "DebugP_memTraceLogWriter.c",
        "DebugP_log.c",
        "TimerP.c",
        "MpuP_armv7m.c",
        "CacheP_null.c",
        "HeapP_internal.c",
        "printf.c",
        "AddrTranslateP.c",
        "CycleCounterP_m4.c",
        "HwiP_armv7m.c",
        "SysTickTimerP.c",
        "QueueP_nortos.c",
        // picked from freertos DPL
        "ClockP_freertos.c",
        "ClockP_freertos_m4.c",
        "SemaphoreP_freertos.c",
        "HwiP_armv7m_handlers_freertos.c",
        "HeapP_freertos.c",
        "DebugP_freertos.c",
        "DebugP_shmLogReader_freertos.c",
        "TaskP_freertos.c",
        "EventP_freertos.c",
        "MailboxP_freertos.c",
        // picked from freertos "kernel"
        "event_groups.c",
        "tasks.c",
        "timers.c",
        "queue.c",
        "list.c",
        "stream_buffer.c",
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

const includes_r5f_common = {
    common: [
        "FreeRTOS-Kernel/include",
        "FreeRTOS-POSIX/include",
        "FreeRTOS-POSIX/include/private",
        "FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include",
        "FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include/portable",
        "FreeRTOS-Plus-CLI",
    ],
};

const includes_r5f = {
    common: [
        ...includes_r5f_common.common,
        "portable/TI_ARM_CLANG/ARM_CR5F",
        "config/am64x/r5f",
    ],
};

const includes_r5f_mpu = {
    common: [
        ...includes_r5f_common.common,
        "portable/TI_ARM_CLANG/ARM_CR5F_MPU",
        "config/am64x/r5f_mpu",
    ],
};

const includes_a53 = {
    common: [
        "FreeRTOS-Kernel/include",
        "portable/GCC/ARM_CA53",
        "config/am64x/a53",
        "FreeRTOS-POSIX/include",
        "FreeRTOS-POSIX/include/private",
        "FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include",
        "FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include/portable",
        "FreeRTOS-Plus-CLI",
    ],
};

const includes_a53_smp = {
    common: [
        "FreeRTOS-Kernel-smp/include",
        "portable_smp/GCC/ARM_CA53",
        "config/am64x/a53-smp",
        "FreeRTOS-POSIX/include",
        "FreeRTOS-POSIX/include/private",
        "FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include",
        "FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include/portable",
        "FreeRTOS-Plus-CLI",
    ],
};

const includes_m4f = {
    common: [
        "FreeRTOS-Kernel/include",
        "portable/TI_ARM_CLANG/ARM_CM4F",
        "config/am64x/m4f",
        "FreeRTOS-POSIX/include",
        "FreeRTOS-POSIX/include/private",
        "FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include",
        "FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include/portable",
    ],
};

const cflags_r5f_common = {
    common: [
        "-Wno-extra"
    ]
};

const cflags_r5f = {
    ...cflags_r5f_common,
};

const cflags_r5f_mpu = {
    ...cflags_r5f_common,
};

const cflags_a53 = {
    common: [
        "-Wno-unused-function", /* needed to suppress warnings in FreeRTOS-POSIX */
        "-Wno-maybe-uninitialized", /* needed to suppress warnings in FreeRTOS-POSIX */
    ]
};

const defines_a53_smp = {
    common: [
        "SMP_FREERTOS",
    ],
};

const defines_a53_amp = {
    common: [
        "AMP_FREERTOS_A53",
    ],
};

const asmfiles_r5f_common = {
    common: [
        // picked from nortos DPL
        "boot_armv7r_asm.S",
        "HwiP_armv7r_asm.S",
        "MpuP_armv7r_asm.S",
        "CacheP_armv7r_asm.S",
        "CpuId_armv7r_asm.S",
        "PmuP_armv7r_asm.S",
        // picked from freertos DPL
        "HwiP_armv7r_vectors_freertos_asm.S",
        "HwiP_armv7r_handlers_freertos_asm.S",
        "Mutex_armv7r_asm.S",
    ],
};

const asmfiles_r5f = {
    common: [
        ...asmfiles_r5f_common.common,
        // picked from freertos "portable/TI_ARM_CLANG/ARM_CR5F"
        "portASM.S",
    ],
};

const asmfiles_r5f_mpu = {
    common: [
        ...asmfiles_r5f_common.common,
        // picked from freertos "portable/TI_ARM_CLANG/ARM_CR5F_MPU"
        "portASM.S",
        "mpu_wrappers_v2_asm.S",
    ],
};

const asmfiles_a53 = {
    common: [
        // picked from nortos DPL
        "boot_armv8_asm.S",
        "HwiP_armv8_asm.S",
        "MmuP_armv8_asm.S",
        "CacheP_armv8_asm.S",
        "PmuP_armv8_asm.S",
        "common_armv8_asm.S",
        // picked from freertos DPL
        // none
        // picked from freertos "portable"
        "portASM.S",
        "SpinlockP_armv8.S"
    ],
};

const asmfiles_a53_smp = {
    common: [
        // picked from nortos DPL
        "boot_armv8_asm.S",
        "HwiP_armv8_asm.S",
        "MmuP_armv8_asm.S",
        "CacheP_armv8_asm.S",
        "PmuP_armv8_asm.S",
        "common_armv8_asm.S",
        // picked from freertos DPL
        // none
        // picked from freertos "portable"
        "portASM.S",
        "SpinlockP_armv8.S"
    ],
};

const filedirs_r5f_common = {
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
        // picked from freertos CLI
        "FreeRTOS-Plus-CLI/",
    ],
};

const filedirs_r5f_ti_arm_clang = {
    common: [
        ...filedirs_r5f_common.common,
        "portable/TI_ARM_CLANG/ARM_CR5F",
        "../nortos/dpl/r5/ti-arm-clang",
        "dpl/r5/ti-arm-clang",
    ],
};

const filedirs_r5f_mpu = {
    common: [
        ...filedirs_r5f_common.common,
        "portable/TI_ARM_CLANG/ARM_CR5F_MPU",
        "FreeRTOS-Kernel/portable/Common",
        "../nortos/dpl/r5/ti-arm-clang",
        "dpl/r5/ti-arm-clang",
    ],
};

const filedirs_a53 = {
    common: [
        // picked from nortos DPL
        "../nortos/dpl/a53",
        "../nortos/dpl/common",
        // picked from freertos DPL
        "dpl/common",
        "dpl/a53",
        // picked from freertos "kernel"
        "FreeRTOS-Kernel/",
        "FreeRTOS-Kernel/portable/MemMang",
        // picked from freertos "posix"
        "FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/source/",
        // picked from freertos "portable"
        "portable/GCC/ARM_CA53",
        // picked from freertos CLI
        "FreeRTOS-Plus-CLI/",
    ],
};

const filedirs_a53_smp = {
    common: [
        // picked from nortos DPL
        "../nortos/dpl/a53",
        "../nortos/dpl/common",
        // picked from freertos DPL
        "dpl/common",
        "dpl/a53",
        // picked from freertos "kernel"
        "FreeRTOS-Kernel-smp/",
        "FreeRTOS-Kernel-smp/portable/MemMang",
        // picked from freertos "posix"
        "FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/source/",
        // picked from freertos "portable"
        "portable_smp/GCC/ARM_CA53",
        // picked from freertos CLI
        "FreeRTOS-Plus-CLI/",
    ],
};

const filedirs_m4f = {
    common: [
        // picked from nortos DPL
        "../nortos/dpl/m4",
        "../nortos/dpl/common",
        // picked from freertos DPL
        "dpl/common",
        "dpl/m4",
        // picked from freertos "kernel"
        "FreeRTOS-Kernel/",
        "FreeRTOS-Kernel/portable/MemMang",
        // picked from freertos "posix"
        "FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/source/",
        // picked from freertos "portable"
        "portable/TI_ARM_CLANG/ARM_CM4F",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang", os: "freertos"},
    { device: device, cpu: "r5f", cgt: "gcc-armv7",    os: "freertos"},
    { device: device, cpu: "m4f", cgt: "ti-arm-clang", os: "freertos"},
    { device: device, cpu: "a53", cgt: "gcc-aarch64",  os: "freertos"},
    { device: device, cpu: "a53-smp", cgt: "gcc-aarch64",  os: "freertos"},
];

const templates_freertos_r5f =
[
    {
        input: ".project/templates/am64x/freertos/FreeRTOSConfig.h.xdt",
        output: "config/am64x/r5f/FreeRTOSConfig.h",
        options: {

        },
    }
];

const templates_freertos_r5f_mpu =
[
    {
        input: ".project/templates/am64x/freertos/FreeRTOSConfig_mpu.h.xdt",
        output: "config/am64x/r5f_mpu/FreeRTOSConfig.h",
        options: {

        },
    }
];

const templates_freertos_a53 =
[
    {
        input: ".project/templates/am64x/freertos/FreeRTOSConfig.h.xdt",
        output: "config/am64x/a53/FreeRTOSConfig.h",
        options: {

        },
    }
];

const templates_freertos_a53_smp =
[
    {
        input: ".project/templates/am64x/freertos/FreeRTOSConfig_smp.h.xdt",
        output: "config/am64x/a53-smp/FreeRTOSConfig.h",
        options: {

        },
    }
];

const templates_freertos_m4f =
[
    {
        input: ".project/templates/am64x/freertos/FreeRTOSConfig.h.xdt",
        output: "config/am64x/m4f/FreeRTOSConfig.h",
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
        build_property.filedirs = filedirs_r5f_ti_arm_clang;
        build_property.cflags = cflags_r5f;
        build_property.templates = templates_freertos_r5f;
    }
    if(buildOption.cpu == "r5f-mpu") {
        build_property.files = files_r5f_mpu;
        build_property.includes = includes_r5f_mpu;
        build_property.asmfiles = asmfiles_r5f_mpu;
        build_property.filedirs = filedirs_r5f_mpu;
        build_property.cflags = cflags_r5f_mpu;
        build_property.templates = templates_freertos_r5f_mpu;
    }
    if(buildOption.cpu == "a53") {
        build_property.files = files_a53;
        build_property.includes = includes_a53;
        build_property.asmfiles = asmfiles_a53;
        build_property.filedirs = filedirs_a53;
        build_property.cflags = cflags_a53;
        build_property.templates = templates_freertos_a53;
        build_property.defines = defines_a53_amp;
    }
    if(buildOption.cpu == "a53-smp")
    {
        build_property.files = files_a53_smp;
        build_property.includes = includes_a53_smp;
        build_property.asmfiles = asmfiles_a53_smp;
        build_property.filedirs = filedirs_a53_smp;
        build_property.cflags = cflags_a53;
        build_property.templates = templates_freertos_a53_smp;
        build_property.defines = defines_a53_smp;
    }
    if(buildOption.cpu == "m4f") {
        build_property.files = files_m4f;
        build_property.includes = includes_m4f;
        build_property.filedirs = filedirs_m4f;
        build_property.templates = templates_freertos_m4f;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
