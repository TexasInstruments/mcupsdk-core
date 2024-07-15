let path = require('path');

let device = "am263px";

const files_r5f = {
    common: [
        //Taken from crypto library
        "dma.c",
        "dthe.c",
        "dthe_aes.c",
        "dthe_edma.c",
        "dthe_sha.c",
        "pka.c",
        "eip29t2_firmware.c",
        "crypto_util.c",
        "rng.c",
        "hsmclient.c",
		"hsmclient_loadhsmrt.c",
		"hsmclient_utils.c",
        "sipc_notify_cfg.c",
		"sipc_notify_src.c",
    ],
};

const filedirs_r5f = {
    common: [
        "security_common/drivers/crypto",
        "security_common/drivers/crypto/dthe",
        "security_common/drivers/crypto/dthe/dma/edma",
        "security_common/drivers/crypto/pka",
        "security_common/drivers/crypto/rng",
        "security_common/drivers/hsmclient",
		"security_common/drivers/hsmclient/soc/am263px",
		"security_common/drivers/hsmclient/utils",
        "security_common/drivers/secure_ipc_notify/",
		"security_common/drivers/secure_ipc_notify/soc/",
		"security_common/drivers/secure_ipc_notify/soc/am263px",
    ],
};

const cflags = {
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
        build_property.filedirs = filedirs_r5f;
    }
    build_property.cflags = cflags;
    build_property.includes = includes;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};