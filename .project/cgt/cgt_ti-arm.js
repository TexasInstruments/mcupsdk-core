const cgt = {
    path: "CGT_TI_ARM_PATH",
    cc: "armcl",
    ar: "armar",

    includes: {
        common: [
            "$(CGT_TI_ARM_PATH)/include",
            ".",
            "$(MCU_PLUS_SDK_PATH)/source"],
    },
    cflags: {
        common: [
            "-c", "-qq",
            "-pdsw225",
            "--endian=little",
            "-mv7R5",
            "--abi=eabi",
            "--float_support=vfpv3d16",
            "--code_state=16",
            "--emit_warnings_as_errors",
        ],
        debug: [
            "--symdebug:dwarf",
            "-g",
            "-D_DEBUG_=1",
        ],
        release: [
            "--embed_inline_assembly",
            "-ms",
            "-o3",
        ],
    },
    arflags: {
        common: [
            "rq",
        ]
    },
};

module.exports = cgt;
