let path = require('path');
const _ = require('lodash');

const files = {
    common: [
		"avbtp.c",
		"avtpc.c",
		"avtpc_direct.c",
		"avtpc_acf.c",
		"avtpc_acf_common.c",
		"avtpc_crf.c",
		"avtpc_acf_debug_strings.c",
		"avtpd.c",
		"avtp_ethernet.c",
		"avtpgcfg.c",
		"tilld_ll_avtp_ethernet.c",
		"xl4-extmod-xl4avtp.c",
		"xl4-extmod-xl4avtp_runconf.c"
		// conl2 started
		"aaf_avtpc_listener.c",
		"aaf_avtpc_talker.c",
		"conl2_avtpc_listener.c",
		"conl2_avtpc_talker.c",
		"aes3_helper.c"
		// conl2 end
    ],
};

const filedirs = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/eval_src/tsn_l2",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/eval_src/tsn_l2/tilld",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/eval_src/tsn_l2/l2conf",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/eval_src/tsn_conl2",
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/$(MCU_PLUS_SDK_MCU)/r5f",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/mod",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_gptp",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_uniconf",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_uniconf/yangs",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_uniconf/yangs/generated",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_combase/tilld/sitara",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_unibase",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/eval_inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/eval_inc/tsn_l2",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/eval_inc/tsn_conl2",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/eval_src",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/eval_src/tsn_l2",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/eval_src/tsn_l2/l2conf",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/eval_src/tsn_conl2",
    ],
};

const defines = {
    common: [
        'TSNPKGVERSION=\\"1.2.3\\"',
        'PRINT_FORMAT_NO_WARNING',
        'UB_LOGCAT=5',
        'UB_LOGTSTYPE=UB_CLOCK_REALTIME',
        'AVTPD_HAVE_NO_SIGNAL=1',
        'NO_GETOPT_LONG=1',
        'AVTPD_IN_LIBRARY=1'
    ],
};

const deviceSpecificIncludes = {
    am243x : [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am243x/r5f",
    ],
    am64x : [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am64x/r5f",
    ],
    am263x : [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am263x/r5f",
    ],
    am273x : [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am273x/r5f",
    ],
    awr294x : [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/awr294x/r5f",
    ],
};

const cflags = {
    common: [
        "-Wno-extra",
        "-Wvisibility",
        "--include tsn_buildconf/sitara_buildconf.h",
    ],
    release: [
        "-Oz",
        "-flto",
    ],
};

const deviceSpecific_cflags = {
     am243x : [
        "-mthumb",
        "-fno-strict-aliasing",
    ],
    am64x : [
        "-mthumb",
        "-fno-strict-aliasing",
    ],
    am263x : [
    ],
    am273x : [
        "-fno-strict-aliasing",
    ],
    awr294x : [
        "-fno-strict-aliasing",
    ],
};

const buildOptionCombos = [
    { device: "am263x", cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "am243x", cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "am273x", cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "am64x",  cpu: "r5f", cgt: "ti-arm-clang"},
    { device: "awr294x", cpu: "r5f", cgt: "ti-arm-clang"},
];

function getComponentProperty(device) {
    let property = {};

    property.dirPath = path.resolve(__dirname, "../tsn-stack/eval_src");
    property.type = "library";
    property.name = "tsn_l2-freertos";
    property.tag  = "tsn_l2_freertos";
    property.isInternal = false;

    deviceBuildCombos = []
    for (buildCombo of buildOptionCombos)
    {
        if (buildCombo.device === device)
        {
            deviceBuildCombos.push(buildCombo)
        }
    }
    property.buildOptionCombos = deviceBuildCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;

    includes.common = _.union(includes.common, deviceSpecificIncludes[device]);
    build_property.includes = includes;

    build_property.defines = defines;

    cflags.common = _.union(cflags.common, deviceSpecific_cflags[device]);
    build_property.cflags = cflags;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
