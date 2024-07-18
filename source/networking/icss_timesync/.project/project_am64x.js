let path = require('path');

let device = "am64x";

const files = {
    common: [
        "icss_timeSync_init.c",
        "icss_timeSync_osal.c",
        "icss_timeSync_utils.c",
        "icss_timeSync.c",
        "icss_timeSyncApi.c",
    ],
};

const filedirs = {
    common: [
        "${INDUSTRIAL_COMMUNICATIONS_SDK_PATH}/source/networking/icss_timesync",
    ],
};

const cflags = {
    common: [
        "-mno-unaligned-access",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "icss_timesync";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.cflags = cflags;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
