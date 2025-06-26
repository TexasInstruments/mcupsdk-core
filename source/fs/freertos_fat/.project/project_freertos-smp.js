let path = require('path');

const files = {
    common: [
        "ff_crc.c",
        "ff_dir.c",
        "ff_error.c",
        "ff_fat.c",
        "ff_file.c",
        "ff_ioman.c",
        "ff_memory.c",
        "ff_stdio.c",
        "ff_string.c",
        "ff_sys.c",
        "ff_format.c",
        "ff_mmcsd.c",
        "portable.c",
    ],
};

const filedirs = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/fs/freertos_fat/FreeRTOS-FAT",
        "${MCU_PLUS_SDK_PATH}/source/fs/freertos_fat/portable",
        "${MCU_PLUS_SDK_PATH}/source/fs/freertos_fat/portable/nortos",
    ],
};

const includes_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/fs/freertos_fat/FreeRTOS-FAT/include",
        "${MCU_PLUS_SDK_PATH}/source/fs/freertos_fat/config",
        "${MCU_PLUS_SDK_PATH}/source/fs/freertos_fat/portable",
    ],
};

const includes_a53 = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/fs/freertos_fat/FreeRTOS-FAT/include",
        "${MCU_PLUS_SDK_PATH}/source/fs/freertos_fat/config",
        "${MCU_PLUS_SDK_PATH}/source/fs/freertos_fat/portable",
    ],
};

const cflags = {
    common: [
        "-Wno-extra",
        "-Wno-uninitialized",
        "-Wno-unused-but-set-variable",
    ],
};
const cflags_a53 = {
    common: [
        "-Wno-extra",
        "-Wno-uninitialized",
        "-Wno-unused-but-set-variable",
        "-Wno-stringop-truncation",
        "-Wno-overflow"
    ],
};

const buildOptionCombos = [
    { device: "am64x", cpu: "a53", cgt: "gcc-aarch64",os: "freertos-smp"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "freeRTOS_fat-freertos-smp";
    property.tag = "freeRTOS_fat-freertos-smp";
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

    build_property.filedirs = filedirs;
    build_property.files = files;
    if(buildOption.cpu.match(/r5f*/)) {
        build_property.includes = includes_r5f;
        if(buildOption.cgt.match(/ti-arm-clang*/)) {
            build_property.cflags = cflags;
        }
    }
    if(buildOption.cpu.match(/a53*/)) {
        build_property.includes = includes_a53;
        build_property.cflags = cflags_a53;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
