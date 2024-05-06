let path = require('path');

let device = "am243x";

const files = {
    common: [
        "empty.c",
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
    ],
};

const libs_nortos_r5f = {
    common: [
        "nortos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_nortos_r5f_gcc = {
    common: [
        "nortos.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "drivers.am243x.r5f.gcc-armv7.${ConfigName}.lib",
    ],
};

const libs_nortos_m4f = {
    common: [
        "nortos.am243x.m4f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.m4f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_EMPTY";

const templates_nortos_r5f =
[
    {
        input: ".project/templates/am243x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "empty_main",
        },
    }
];

const templates_nortos_r5f_gcc =
[
    {
        input: ".project/templates/am243x/common/linker_r5f_gcc.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/am243x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "empty_main",
        },
    }
];

const templates_nortos_m4f =
[
    {
        input: ".project/templates/am243x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "empty_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am243x-evm", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss1-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss1-1", cgt: "ti-arm-clang", board: "am243x-evm", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am243x-lp", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss1-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss1-1", cgt: "ti-arm-clang", board: "am243x-lp", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-evm", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-1", cgt: "gcc-armv7", board: "am243x-evm", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss1-0", cgt: "gcc-armv7", board: "am243x-evm", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss1-1", cgt: "gcc-armv7", board: "am243x-evm", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-lp", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-1", cgt: "gcc-armv7", board: "am243x-lp", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss1-0", cgt: "gcc-armv7", board: "am243x-lp", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss1-1", cgt: "gcc-armv7", board: "am243x-lp", os: "nortos", isPartOfSystemProject: true},
];

const systemProjects = [
    {
        name: "empty",
        tag: "nortos",
        skipProjectSpec: false,
        readmeDoxygenPageTag: readmeDoxygenPageTag,
        board: "am243x-evm",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "nortos"},
            { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am243x-evm", os: "nortos"},
            { device: device, cpu: "r5fss1-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "nortos"},
            { device: device, cpu: "r5fss1-1", cgt: "ti-arm-clang", board: "am243x-evm", os: "nortos"},
            { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "nortos"},
        ],
    },
    {
        name: "empty",
        tag: "nortos_gcc-armv7",
        skipProjectSpec: false,
        readmeDoxygenPageTag: readmeDoxygenPageTag,
        board: "am243x-evm",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-evm", os: "nortos"},
            { device: device, cpu: "r5fss0-1", cgt: "gcc-armv7", board: "am243x-evm", os: "nortos"},
            { device: device, cpu: "r5fss1-0", cgt: "gcc-armv7", board: "am243x-evm", os: "nortos"},
            { device: device, cpu: "r5fss1-1", cgt: "gcc-armv7", board: "am243x-evm", os: "nortos"},
        ],
    },
    {
        name: "empty",
        tag: "nortos",
        skipProjectSpec: false,
        readmeDoxygenPageTag: readmeDoxygenPageTag,
        board: "am243x-lp",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "nortos"},
            { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am243x-lp", os: "nortos"},
            { device: device, cpu: "r5fss1-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "nortos"},
            { device: device, cpu: "r5fss1-1", cgt: "ti-arm-clang", board: "am243x-lp", os: "nortos"},
            { device: device, cpu: "m4fss0-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "nortos"},
        ],
    },
    {
        name: "empty",
        tag: "nortos_gcc-armv7",
        skipProjectSpec: false,
        readmeDoxygenPageTag: readmeDoxygenPageTag,
        board: "am243x-lp",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-lp", os: "nortos"},
            { device: device, cpu: "r5fss0-1", cgt: "gcc-armv7", board: "am243x-lp", os: "nortos"},
            { device: device, cpu: "r5fss1-0", cgt: "gcc-armv7", board: "am243x-lp", os: "nortos"},
            { device: device, cpu: "r5fss1-1", cgt: "gcc-armv7", board: "am243x-lp", os: "nortos"},
        ],
    },

];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "empty";
    property.isInternal = false;
    property.tirexResourceSubClass = [ "example.gettingstarted" ];
    property.description = "An Empty Example. This example is intended to be a starting point for new development"
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.libdirs = libdirs_nortos;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    if(buildOption.cpu.match(/r5f*/)) {
        if(buildOption.cgt.match(/gcc*/) )
        {
            build_property.libs = libs_nortos_r5f_gcc;
            build_property.templates = templates_nortos_r5f_gcc;
        }
        else
        {
            build_property.libs = libs_nortos_r5f;
            build_property.templates = templates_nortos_r5f;
        }
    }
    if(buildOption.cpu.match(/m4f*/)) {
        build_property.libs = libs_nortos_m4f;
        build_property.templates = templates_nortos_m4f;
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
