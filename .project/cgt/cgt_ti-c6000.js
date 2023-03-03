
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

const cgt_common_awr294x = {
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
            "--c99",
            "-q",
            "-mo",
            "-pden",
            "--emit_warnings_as_errors",
            "--mem_model:const=data",
            "--mem_model:data=far_aggregates",
            "--remove_hooks_when_inlining",
            "-on2",
            "--disable_push_pop",
            "--fp_mode=relaxed",
            "--assume_control_regs_read",
        ],
        debug: [
            "-D_DEBUG_=1",
        ],
        release: [
            "--program_level_compile",
            "-o3",
            "-mf2",
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
            "--ram_model",
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

function getCgtOptions(cpu, device)
{
    let cgtOptions = {};

    if(device.match(/awr294x*/))
    {
        cgtOptions = cgt_common_awr294x;
    }
    else
    {
        cgtOptions = cgt_common;
    }

    return cgtOptions;
}

module.exports = {

    getCgtOptions,
};
