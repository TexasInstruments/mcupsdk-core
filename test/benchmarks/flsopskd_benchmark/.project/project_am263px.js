const project_name = "flsopskd_benchmark";
let path = require('path');

let device = "am263px";

const r5f00_asmfiles = {
    common: [
        "csl_arm_r5_pmu.S",
        "hopperFnx.S"
    ]
}

const r5f01_asmfiles = {
    common: [
        "csl_arm_r5_pmu.S"
    ]
}

const r5fss00_projectspec_files = {
    common: [
        "csl_arm_r5.h",
        "csl_arm_r5_pmu.h",
        "appprofile.h",
    ]
}

const r5fss00_files = {
    common: [
        "main.c",
        "appprofile.c",
    ],
};

const r5fss01_projectspec_files = {
    common: [
        "csl_arm_r5.h",
        "csl_arm_r5_pmu.h",
        "appprofile.h",
        "base.h",
        "board.h",
    ]
}

const r5fss01_files = {
    common: [
        "main.c",
        "appprofile.c",
        "board.c",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../..",
        "../../..", /* Example base */
    ],
};

const includes = {
    common:[
        "..",       /* core_os_combo base */
        "../..",
        "../../..", /* Example base */
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am263px/r5f",
    ]
}

const libdirs_nortos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
    ],
};

const libdirs_freertos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
    ],
};

const libs_nortos_r5f = {
    common: [
        "nortos.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};
const libs_freertos_r5f = {
    common: [
        "drivers.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "freertos.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const templates_nortos_r5f =
[];

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_flsopskd_benchmark";

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-cc", os: "freertos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am263px-cc", os: "nortos", isPartOfSystemProject: true},
];

const systemProjects = [
    {
        name: project_name,
        tag: project_name,
        skipProjectSpec: false,
        readmeDoxygenPageTag: readmeDoxygenPageTag,
        board: "am263px-cc",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-cc", os: "freertos"},
            { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am263px-cc", os: "nortos"},
        ],
    }
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = project_name;
    property.isInternal = true;
    property.tirexResourceSubClass = [ "example.gettingstarted" ];
    property.description = "An flsopskd_benchmark Example."
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};
    if(buildOption.cpu === "r5fss0-0")
    {
        build_property.asmfiles = r5f00_asmfiles;
        build_property.files = r5fss00_files;
        build_property.projectspecfiles = r5fss00_projectspec_files;
    }
    else if(buildOption.cpu === "r5fss0-1")
    {
        build_property.asmfiles = r5f01_asmfiles;
        build_property.files = r5fss01_files;
        build_property.projectspecfiles = r5fss01_projectspec_files;
        build_property.templates = templates_nortos_r5f;
    }
    build_property.includes = includes;
    build_property.filedirs = filedirs;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;

    if(buildOption.cpu.match(/r5f*/)) {

        if(buildOption.os.match(/freertos*/) )
        {
            build_property.libs = libs_freertos_r5f;
            build_property.libdirs = libdirs_freertos;
        }
        else
        {
            build_property.libs = libs_nortos_r5f;
            build_property.libdirs = libdirs_nortos;
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
