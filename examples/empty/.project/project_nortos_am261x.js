let path = require('path');

let device = "am261x";

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

const libs_nortos_r5f_ti_arm_clang = {
    common: [
        "nortos.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};


const libs_nortos_r5f_iar_arm = {
    common: [
        "nortos.am261x.r5f.iar-arm.${ConfigName}.lib",
        "drivers.am261x.r5f.iar-arm.${ConfigName}.lib",
    ],
};

const lnkfiles_ti_arm_clang = {
    common: [
        "linker.cmd",
    ]
};

const lnkfiles_iar_arm = {
    common: [
        "linker.icf",
    ]
};

const syscfgfile = "example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_EMPTY";

const templates_nortos_r5f =
[
    {
        input: ".project/templates/am261x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "empty_main",
        },
    }
];


const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-som", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am261x-som", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-lp", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am261x-lp", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-0", cgt: "iar-arm", board: "am261x-lp", os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-1", cgt: "iar-arm", board: "am261x-lp", os: "nortos", isPartOfSystemProject: true},
];

const systemProjects = [
    {
        name: "empty",
        tag: "nortos",
        cgt: "ti-arm-clang",
        skipProjectSpec: false,
        readmeDoxygenPageTag: readmeDoxygenPageTag,
        board: "am261x-som",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-som", os: "nortos"},
            { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am261x-som", os: "nortos"},
        ],
    },
    {
        name: "empty",
        tag: "nortos",
        cgt: "ti-arm-clang",
        skipProjectSpec: false,
        readmeDoxygenPageTag: readmeDoxygenPageTag,
        board: "am261x-lp",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-lp", os: "nortos"},
            { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am261x-lp", os: "nortos"},
        ],
    },
    {
        name: "empty",
        tag: "nortos",
        cgt: "iar-arm",
        skipProjectSpec: false,
        readmeDoxygenPageTag: readmeDoxygenPageTag,
        board: "am261x-lp",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "iar-arm", board: "am261x-lp", os: "nortos"},
            { device: device, cpu: "r5fss0-1", cgt: "iar-arm", board: "am261x-lp", os: "nortos"},
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
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    if(buildOption.cpu.match(/r5f*/)) {
        build_property.templates = templates_nortos_r5f;
        if(buildOption.cgt.match(/ti-arm-clang/)) {
            build_property.libs = libs_nortos_r5f_ti_arm_clang;
            build_property.lnkfiles = lnkfiles_ti_arm_clang;
        }
        else if(buildOption.cgt.match(/iar-arm/)) {
            build_property.libs = libs_nortos_r5f_iar_arm;
            build_property.lnkfiles = lnkfiles_iar_arm;
        }
    }
    if(buildOption.cpu.match(/c66*/)) {
        build_property.libs = libs_nortos_c66;
        build_property.templates = templates_nortos_c66;
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
