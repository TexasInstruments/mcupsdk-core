let path = require('path');

let device = "awr294x";

const files = {
    common: [
        "test_adcbuf.c",
        "adcbuf_test_pos.c",
        "adcbuf_test_neg.c",
        "main.c",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
    ],
};

const libdirs_nortos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/test/unity/lib",
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/test/unity/",
    ],
};

const libs_nortos_r5f = {
    common: [
        "nortos.awr294x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.awr294x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.awr294x.r5f.ti-arm-clang.${ConfigName}.lib",
        "unity.awr294x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_nortos_c66 = {
    common: [
        "nortos.awr294x.c66.ti-c6000.${ConfigName}.lib",
        "drivers.awr294x.c66.ti-c6000.${ConfigName}.lib",
        "board.awr294x.c66.ti-c6000.${ConfigName}.lib",
        "unity.awr294x.c66.ti-c6000.${ConfigName}.lib",
    ],
};


const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const templates_nortos_r5f =
[
    {
        input: ".project/templates/awr294x/common/linker_r5f.cmd.xdt",
        output: "linker.cmd",
        options: {
            isSingleCore: true,
        },
    },
    {
        input: ".project/templates/awr294x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_main",
        },
    }
];

const templates_nortos_c66 =
[
    {
        input: ".project/templates/awr294x/common/linker_c66.cmd.xdt",
        output: "linker.cmd",
        options: {
            isSingleCore: true,
        },
    },
    {
        input: ".project/templates/awr294x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "test_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "awr294x-evm", os: "nortos"},
    { device: device, cpu: "c66ss0",   cgt: "ti-c6000",     board: "awr294x-evm", os: "nortos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "adcbuf_ut";
    property.isInternal = true;
    property.skipProjectSpec = true;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.includes = includes;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;

    if(buildOption.cpu.match(/r5f*/)) {
        build_property.libdirs = libdirs_nortos;
        build_property.libs = libs_nortos_r5f;
        build_property.templates = templates_nortos_r5f;
    }
    if(buildOption.cpu.match(/c66*/)) {
        build_property.libdirs = libdirs_nortos;
        build_property.libs = libs_nortos_c66;
        build_property.templates = templates_nortos_c66;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
