
const common = require(`../common.js`);

const cgt_icssg0 = {
    cflags: {
        common: [
            "-DICSSG0"
        ],
    },
}

const cgt_icssg1 = {
    cflags: {
        common: [
            "-DICSSG1"
        ],
    },
}

const cgt_pru0 = {
    cflags: {
        common: [
            "-DPRU0",
            "-DSLICE0"
        ],
    },
}

const cgt_pru1 = {
    cflags: {
        common: [
            "-DPRU1",
            "-DSLICE1"
        ],
    },
}

const cgt_rtpru0 = {
    cflags: {
        common: [
            "-DRTU_PRU0",
            "-DSLICE0"
        ],
    },
}

const cgt_rtpru1 = {
    cflags: {
        common: [
            "-DRTU_PRU1",
            "-DSLICE1"
        ],
    },
}

const cgt_txpru0 = {
    cflags: {
        common: [
            "-DTX_PRU0",
            "-DSLICE0"
        ],
    },
}

const cgt_txpru1 = {
    cflags: {
        common: [
            "-DTX_PRU1",
            "-DSLICE1"
        ],
    },
}

const cgt_common = {
    path: "CGT_TI_PRU_PATH",

    includes: {
        common: [
            "${CG_TOOL_ROOT}/include",
            "${MCU_PLUS_SDK_PATH}/source"
        ],
    },
    cflags: {
        common: [
            "-v4",
        ],
        debug: [
            "-D_DEBUG_=1",
        ],
        release: [
            "-O3",
        ],
    },
    lflags: {
        common: [
            "-llibc.a"
        ],
    },
    libdirs: {
        common: [
            "${CG_TOOL_ROOT}/lib"
        ],
    },
};

function getCgtOptions(cpu, device)
{
    let cgtOptions = {};

    if(cpu.match(/pru0/))
    {
        cgtOptions = common.mergeCgtOptions(cgt_common, cgt_pru0);
    }
    if(cpu.match(/pru1/))
    {
        cgtOptions = common.mergeCgtOptions(cgtOptions, cgt_pru1);
    }
    if(cpu.match(/rtpru0/))
    {
        cgtOptions = common.mergeCgtOptions(cgtOptions, cgt_rtpru0);
    }
    if(cpu.match(/rtpru1/))
    {
        cgtOptions = common.mergeCgtOptions(cgtOptions, cgt_rtpru1);
    }
    if(cpu.match(/txpru0/))
    {
        cgtOptions = common.mergeCgtOptions(cgtOptions, cgt_txpru0);
    }
    if(cpu.match(/txpru1/))
    {
        cgtOptions = common.mergeCgtOptions(cgtOptions, cgt_txpru1);
    }
    if(cpu.match(/icssg0/))
    {
        cgtOptions = common.mergeCgtOptions(cgtOptions, cgt_icssg0);
    }
    if(cpu.match(/icssg1/))
    {
        cgtOptions = common.mergeCgtOptions(cgtOptions, cgt_icssg1);
    }
    return cgtOptions;
}

module.exports = {

    getCgtOptions,
};
