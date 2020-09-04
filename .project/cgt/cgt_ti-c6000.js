
const common = require(`../common.js`);

const cgt_common = {
    path: "CGT_TI_C6000_PATH",
    cc: "cl6x",
    ar: "ar6x",
    lnk: "lnk6x",
    strip: "strip6x",
    objcopy: "",

    includes: {
        common: [
            "${CG_TOOL_ROOT}/include",
            "${MCU_PLUS_SDK_PATH}/source"
        ],
    },
    cflags: {
        common: [
            "-mv6600",
            "--abi=eabi",
            "-q",
            "-mi10",
            "-mo",
            "-pden",
            "-pds=238",
            "-pds=880",
            "-pds1110",
            "--emit_warnings_as_errors",
        ],
        debug: [
            "-D_DEBUG_=1",
        ],
        release: [
            "--program_level_compile",
            "-o3",
            "-mf3",
        ],
    },
    arflags: {
        common: [
            "rq",
        ],
    },
    lflags: {
        common: [
            "--warn_sections",
            "--emit_warnings_as_errors",
            "--rom_model",
            "-x",
        ],
    },
    libdirs: {
        common: [
            "${CG_TOOL_ROOT}/lib",
        ],
    },
    libs: {
        common: [
            "libc.a",
        ],
    },
};

function getCgtOptions(cpu)
{
    let cgtOptions = {};

    cgtOptions = cgt_common;

    return cgtOptions;
}

module.exports = {

    getCgtOptions,
};
