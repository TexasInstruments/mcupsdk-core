
const common = require(`../common.js`);

const cgt_a53 ={
    cflags: {
        common: [
            "-mcpu=cortex-a53+fp+simd",
            "-mabi=lp64",
            "-mcmodel=large",
            "-mstrict-align",
            "-mfix-cortex-a53-835769",
            "-mfix-cortex-a53-843419",
        ],
    },
}

const cgt_common = {
    path: "CGT_GCC_AARCH64_PATH",
    cc: "aarch64-none-elf-gcc",
    ar: "aarch64-none-elf-gcc-ar",
    lnk: "aarch64-none-elf-gcc",
    strip: "aarch64-none-elf-strip",
    objcopy: "aarch64-none-elf-objcopy",
    cov: "aarch64-none-elf-gcov",

    includes: {
        common: [
            "${MCU_PLUS_SDK_PATH}/source",
        ],
    },
    cflags: {
        common: [
            "-Wall",
            "-Werror",
            "-g",
            "-Wno-int-to-pointer-cast",
            "-Wno-pointer-to-int-cast",
            "-Wno-unused-but-set-variable",
            "-fdata-sections",
	        "-ffunction-sections"
        ],
        debug: [
            "-D_DEBUG_=1"
        ],
        release: [
            "-O2"
        ],
    },
    arflags: {
        common: [
            "cr",
        ],
    },
    lflags: {
        common: [
            "-Wl,-static",
	        "-Wl,--gc-sections",
	        "-Wl,--build-id=none",
            "-lstdc++",
	        "-lgcc",
	        "-lm",
	        "-lc",
	        "-lrdimon",
        ],
    },
    libdirs: {
        common: [
            "$(CGT_GCC_AARCH64_PATH)/lib",
        ],
    }
};

function getCgtOptions(cpu, device)
{
    let cgtOptions = {};

    if (cpu.match(/a53*/))
    {
        cgtOptions = common.mergeCgtOptions(cgt_common, cgt_a53);
    }
    return cgtOptions;
}

module.exports = {

    getCgtOptions,
};