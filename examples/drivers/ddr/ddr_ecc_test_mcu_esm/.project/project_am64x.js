let path = require('path');

let device = "am64x";

const files_r5f = {
    common: [
        "ddr_ecc_test.c",
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
        "../../../dpl", /* SDL dpl base add an extra lvl*/
    ],
};

const files_m4f = {
    common: [
        "ddr_ecc_mcu_esm_handle.c",
        "main.c",
        "dpl_interface.c"
    ],
};

const libdirs_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
    ],
};

const libdirs_m4f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/source/sdl/lib",
    ],
};

const libs_nortos_r5f = {
    common: [
        "nortos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_nortos_m4f = {
    common: [
        "nortos.am64x.m4f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am64x.m4f.ti-arm-clang.${ConfigName}.lib",
        "board.am64x.m4f.ti-arm-clang.${ConfigName}.lib",
        "sdl.am64x.m4f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const includes_m4f = {
    common: [
        "../../../dpl",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg"

const readmeDoxygenPageTag = "EXAMPLES_DRIVERS_DDR_ECC_TEST_MCU_ESM";

const templates_nortos_r5f =
[
    {
        input: ".project/templates/am64x/common/linker_r5f.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/am64x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "ddr_ecc_test_main",
        },
    },
];

const templates_nortos_m4f =
[
    {
        input: ".project/templates/am64x/common/linker_m4f.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/am64x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "ddr_ecc_test_mcu_esm_handle",
        },
    }
];

const buildOptionCombos = [
    { device: "am64x", cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "nortos", isPartOfSystemProject: true},
    { device: "am64x", cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "nortos", isPartOfSystemProject: true},
    { device: "am64x", cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-sk", os: "nortos", isPartOfSystemProject: true},
    { device: "am64x", cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am64x-sk", os: "nortos", isPartOfSystemProject: true},
];

const systemProjects = [
    {
        name: "ddr_ecc_test_mcu_esm",
        tag: "nortos",
        skipProjectSpec: false,
        readmeDoxygenPageTag: readmeDoxygenPageTag,
        board: "am64x-evm",
        projects: [
            { device: "am64x", cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "nortos"},
            { device: "am64x", cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "nortos"},
        ],
    },
    {
        name: "ddr_ecc_test_mcu_esm",
        tag: "nortos",
        skipProjectSpec: false,
        readmeDoxygenPageTag: readmeDoxygenPageTag,
        board: "am64x-sk",
        projects: [
            { device: "am64x", cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-sk", os: "nortos"},
            { device: "am64x", cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am64x-sk", os: "nortos"},
        ],
    }
]

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "ddr_ecc_test_mcu_esm";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.filedirs = filedirs;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;

    if(buildOption.cpu.match(/r5f*/)) {
        build_property.files = files_r5f;
        build_property.libdirs = libdirs_r5f;
        build_property.libs = libs_nortos_r5f;
        build_property.templates = templates_nortos_r5f;
    }

    if(buildOption.cpu.match(/m4f*/)) {
        build_property.files = files_m4f;
        build_property.libdirs = libdirs_m4f;
        build_property.libs = libs_nortos_m4f;
        build_property.templates = templates_nortos_m4f;
        build_property.includes = includes_m4f;
    }

    return build_property;
}

function getSystemProjects(device)
{
    return systemProjects;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
    getSystemProjects,
};
