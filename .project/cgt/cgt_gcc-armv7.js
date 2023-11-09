
const common = require(`../common.js`);

const cgt_r5f = {
    cflags: {
        common: [
            "-mcpu=cortex-r5",
            "-mfloat-abi=hard",
            "-mfpu=vfpv3-d16",
        ],
    },
    lflags: {
        common: [
            "-mcpu=cortex-r5",
            "-mfloat-abi=hard",
            "-mfpu=vfpv3-d16",
        ],
    },
}

const cgt_common = {
    path: "CGT_GCC_ARMV7_PATH",
    cc: "arm-none-eabi-gcc",
    ar: "arm-none-eabi-gcc-ar",
    lnk: "arm-none-eabi-gcc",
    cpp: "arm-none-eabi-cpp",
    strip: "arm-none-eabi-strip",
    objcopy: "arm-none-eabi-objcopy",

    includes: {
        common: [
            "${CG_TOOL_ROOT}/arm-none-eabi/include",
            "${MCU_PLUS_SDK_PATH}/source",
        ],
    },
    cflags: {
        common: [
            "-Wall",
            "-mthumb",
            "-Werror",
            "-g",
            "-Wno-unused-function",
            "-Wno-enum-compare",
            "-Wno-uninitialized",
            "-Wno-int-to-pointer-cast",
            "-fgnu89-inline",
            "-Wno-pointer-to-int-cast",
            "-Wno-unused-variable",
            "-Wno-unused-but-set-variable",
            "-fdata-sections", /* this option is needed so that compiler throws away unused sections */
	        "-ffunction-sections", /* this option is needed so that compiler throws away unused sections */
        ],
        debug: [
            "-D_DEBUG_=1",
        ],
        release: [
            "-O2",
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
	        "-lnosys", /* this disable CCS output, rdimon enables CCS output, i.e semi-hosting, but there is some problem so disabling it */
        ],
    },
    libdirs: {
        common: [
            "${CGT_GCC_ARMV7_PATH}/lib",
        ],
    },
};

function getCgtOptions(cpu, device)
{
    let cgtOptions = {};

    if(cpu.match(/r5f*/))
    {
        cgtOptions = common.mergeCgtOptions(cgt_common, cgt_r5f);
    }
    return cgtOptions;
}

module.exports = {

    getCgtOptions,
};
