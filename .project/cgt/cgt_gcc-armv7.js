
const common = require(`../common.js`);

const cgt_r5f = {
    cflags: {
        common: [
            "-mcpu=cortex-r5",
            "-mfloat-abi=hard",
            "-mfpu=vfpv3-d16",
            "-marm",
        ],
    },
    lflags: {
        common: [
            "-mcpu=cortex-r5",
            "-mfloat-abi=hard",
            "-mfpu=vfpv3-d16",
            "-marm",
        ],
    },
}

const cgt_common = {
    path: "CGT_GCC_ARMV7_PATH",
    cc: "arm-none-eabi-gcc",
    ar: "arm-none-eabi-gcc-ar",
    lnk: "arm-none-eabi-gcc",
    strip: "arm-none-eabi-strip",
    objcopy: "arm-none-eabi-objcopy",

    includes: {
        common: [
            "${MCU_PLUS_SDK_PATH}/source"
        ],
    },
    cflags: {
        common: [
            "-Wall",
            "-Werror",
            "-g",
            "-Wno-gnu-variable-sized-type-not-at-end",
            "-Wno-unused-function",
            "-Wno-int-to-pointer-cast",
            "-Wno-pointer-to-int-cast",
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
