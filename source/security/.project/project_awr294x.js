let path = require('path');

let device = "awr294x";

const files_r5f = {
    common: [
        //Taken from crypto library
        "dma.c",
        "dthe.c",
        "dthe_aes.c",
        "dthe_edma.c",
        "dthe_sha.c",
        "crypto_util.c",
        "hsmclient.c",
		"hsmclient_loadhsmrt.c",
		"hsmclient_utils.c",
        "sipc_notify_cfg.c",
		"sipc_notify_src.c",
    ],
};

const files_c66 = {
    common: [
        //Taken from crypto library
        "dma.c",
        "dthe.c",
        "dthe_aes.c",
        "dthe_edma.c",
        "dthe_sha.c",
        "crypto_util.c",
        "hsmclient.c",
		"hsmclient_utils.c",
        "sipc_notify_cfg.c",
		"sipc_notify_src.c",
    ],
};

const filedirs = {
    common: [
        "security_common/drivers/crypto",
        "security_common/drivers/crypto/dthe",
        "security_common/drivers/crypto/dthe/dma/edma",
        "security_common/drivers/hsmclient",
		"security_common/drivers/hsmclient/soc/awr294x",
		"security_common/drivers/hsmclient/utils",
        "security_common/drivers/secure_ipc_notify/",
		"security_common/drivers/secure_ipc_notify/soc/",
		"security_common/drivers/secure_ipc_notify/soc/awr294x",
    ],
};

const cflags_r5 = {
    common: [
        "-mno-unaligned-access",
        "-Wno-extra",
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/security",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
    { device: device, cpu: "c66", cgt: "ti-c6000"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "security";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    if(buildOption.cpu.match(/r5f*/)) {
        build_property.files = files_r5f;
        build_property.filedirs = filedirs;
        build_property.cflags = cflags_r5;
    }
    if(buildOption.cpu.match(/c66*/)) {
        build_property.files = files_c66;
        build_property.filedirs = filedirs;
    }
    build_property.includes = includes;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};