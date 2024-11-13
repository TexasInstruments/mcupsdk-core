let path = require('path');

const device_project = require("../../../.project/device/project_am243x.js");

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

const libdirs_threadx = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/threadx/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
    ],
};

const includes_threadx_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/threadx/threadx_src/common/inc",
        "${MCU_PLUS_SDK_PATH}/source/kernel/threadx/ports/ti_arm_gcc_clang_cortex_r5/inc",
    ],
};

const libs_threadx_r5f = {
    common: [
        "threadx.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_EMPTY";

const templates_threadx_r5f =
[
    {
        input: ".project/templates/am243x/threadx/main_threadx.c.xdt",
        output: "../main.c",
        options: {
        entryFunction: "empty_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "threadx", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am243x-evm", os: "threadx", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss1-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "threadx", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss1-1", cgt: "ti-arm-clang", board: "am243x-evm", os: "threadx", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "threadx", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am243x-lp", os: "threadx", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss1-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "threadx", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss1-1", cgt: "ti-arm-clang", board: "am243x-lp", os: "threadx", isPartOfSystemProject: true},
];

const systemProjects = [
    {
        name: "empty",
        tag: "threadx",
        skipProjectSpec: false,
        readmeDoxygenPageTag: readmeDoxygenPageTag,
        board: "am243x-evm",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "threadx"},
            { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am243x-evm", os: "threadx"},
            { device: device, cpu: "r5fss1-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "threadx"},
            { device: device, cpu: "r5fss1-1", cgt: "ti-arm-clang", board: "am243x-evm", os: "threadx"},
        ],
    },
    {
        name: "empty",
        tag: "threadx",
        skipProjectSpec: false,
        readmeDoxygenPageTag: readmeDoxygenPageTag,
        board: "am243x-lp",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "threadx"},
            { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am243x-lp", os: "threadx"},
            { device: device, cpu: "r5fss1-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "threadx"},
            { device: device, cpu: "r5fss1-1", cgt: "ti-arm-clang", board: "am243x-lp", os: "threadx"},
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
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    if(buildOption.cpu.match(/r5f*/)) {
        if(buildOption.os.match(/freertos*/) )
        {
            build_property.includes = includes_freertos_r5f;
            build_property.libdirs = libdirs_freertos;
            if(buildOption.cgt.match(/gcc*/) )
            {
                build_property.libs = libs_freertos_r5f_gcc;
                build_property.templates = templates_freertos_r5f_gcc;
            }
            else
            {
                build_property.libs = libs_freertos_r5f;
                build_property.templates = templates_freertos_r5f;
            }
        } else if (buildOption.os.match(/threadx*/)) {
            build_property.includes = includes_threadx_r5f;
            build_property.libdirs = libdirs_threadx;
            build_property.libs = libs_threadx_r5f;
            build_property.templates = templates_threadx_r5f;
        }
    }
    if(buildOption.cpu.match(/m4f*/)) {
        if(buildOption.os.match(/freertos*/) )
        {
            build_property.includes = includes_freertos_m4f;
            build_property.libdirs = libdirs_freertos;
            build_property.libs = libs_freertos_m4f;
            build_property.templates = templates_freertos_m4f;
        }
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
