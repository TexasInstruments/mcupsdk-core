
const common = require(`../common.js`);

const cgt_r5f = {
    cflags: {
        common: [
            "--cpu=Cortex-R5",
            "--fpu=VFPv3_D16",
        ],
    },
    asmflags: {
        common: [
            "--cpu=Cortex-R5",
            "--fpu=VFPv3_D16",
        ],
    },
    lflags: {
        common: [
        ],
    },
}

const cgt_common = {
    path: "CGT_IAR_ARM_PATH",
    cc: "iccarm",
    as: "iasmarm",
    ar: "iarchive",
    lnk: "ilinkarm",

    includes: {
        common: [
            "${MCU_PLUS_SDK_PATH}/source"
        ],
    },
    cflags: {
        common: [
            "--cpu_mode thumb",
            "--debug",
            "--diag_suppress=Pa039",
            "--diag_suppress=Pe177",
            "-e",
            "--endian=little",
            "--silent",
        ],
        cpp_common: [
           
        ],
        debug: [
            "-D_DEBUG_=1",
        ],
        release: [
            "-Oh",
        ],
    },
    asmflags: {
        common: [
        ],
    },
    arflags: {
        common: [
            "--create",
        ],
    },
    lflags: {
        common: [
            "--silent",
             "--entry __iar_program_start",
	        "--semihosting=iar_breakpoint",
            "--redirect __write=__write_buffered",
        ],
    },
    libdirs: {
        common: [
        ],
    },
    libs: {
        common: [
        ],
    },
};

const cgt_instrumentation = {
    cflags: {
        common: [
           
        ],
    },
};

function getCgtOptions(cpu, device)
{
    let cgtOptions = {};

    if(cpu.match(/r5f*/))
    {
        cgtOptions = common.mergeCgtOptions(cgt_common, cgt_r5f);
        if(common.isInstrumentationMode()) {
            cgtOptions = common.mergeCgtOptions(cgtOptions, cgt_instrumentation);
        }
    }
    return cgtOptions;
}

module.exports = {

    getCgtOptions,
};
