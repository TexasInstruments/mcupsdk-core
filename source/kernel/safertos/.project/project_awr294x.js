let path = require('path');

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
        "TimerP_rti.c",
        "QueueP_nortos.c",
        // picked from safertos DPL
        "ClockP_safertos.c",
        "ClockP_safertos_c66.c",
        "DebugP_safertos.c",
        "DebugP_shmLogReader_safertos.c",
        "HeapP_safertos.c",
        "HwiP_c66_handlers_safertos.c",
        "SemaphoreP_safertos.c",
        "TaskP_safertos.c",
        "EventP_safertos.c",
        // picked from safertos "kernel"
        "list.c",
        "eventgroups.c",
        "task.c",
        "queue.c",
        "evtmplx.c",
        "timers.c",
        "semaphore.c",
        "mutex.c",
        "streambuffer.c",
        // picked from safertos "portable"
        "portable.c",
        // picked from safertos "api"
        "apiSemaphoreWrapper.c",
        "apiEvtMplxWrapper.c",
        "apiTimersWrapper.c",
        "apiMutexWrapper.c",
        "apiStreamBufferWrapper.c",
        "apiWrapper.c",
        "apiEventGroupsWrapper.c",
    ]
};

const includes_c66 = {
    common: [
        "safeRTOS/api/201_C66x",
        "safeRTOS/api/PrivWrapperStd",
        "safeRTOS/portable/201_C66x",
        "safeRTOS/portable/201_C66x/005_TI_CGT",
        "safeRTOS/kernel/include_api",
        "safeRTOS/kernel/include_prv",
        "safeRTOS/config",
    ],
};

const cflags_c66 = {
    common: [
        "--strip_coff_underscore"
    ]
};

const asmfiles_c66 = {
    common: [
        // picked from nortos DPL
        "HwiP_c66_safertos_asm.S",
    ],
};

const asmextnfiles_c66 = {
    common: [
        // picked from safertos "portable"
        "portasm.asm",
    ],
};

const filedirs_c66 = {
    common: [
        // picked from nortos DPL
        "../nortos/dpl/c66",
        "../nortos/dpl/common",
        // picked from safertos DPL
        "dpl/c66",
        "dpl/common",
        // picked from safertos "kernel"
        "safeRTOS/kernel",
        // picked from safertos "portable"
        "safeRTOS/portable/201_C66x",
        "safeRTOS/portable/201_C66x/005_TI_CGT",
        // picked from safertos "api"
        "safeRTOS/api/PrivWrapperStd",
    ],
};

const defines_safertos = {
    common: [
        "OS_SAFERTOS",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "c66", cgt: "ti-c6000",     os: "safertos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "safertos";
    property.isInternal = false;
    property.isSkipTopLevelBuild = true;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    if(buildOption.cpu == "c66") {
        build_property.files = files_c66;
        build_property.includes = includes_c66;
        build_property.asmfiles = asmfiles_c66;
        build_property.asmextnfiles = asmextnfiles_c66
        build_property.filedirs = filedirs_c66;
        build_property.cflags = cflags_c66;
        build_property.defines = defines_safertos;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
