let path = require('path');

let device = "am263x";

const files_nortos_rf5_00 = {
    common: [
        "gpio_controller.c",
        "main.c",
    ],
};
const files_nortos_rf5_01 = {
    common: [
        "mcspi_peripheral.c",
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
    ],
};


const libs_nortos_r5f = {
    common: [
        "nortos.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};


const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_DRIVERS_GPIO_CONTROLLER_MCSPI_PERIPHERAL";

const templates_nortos_r5fss00 =
[
    {
        input: ".project/templates/am263x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "gpio_controller_main",
        },
    }
];
const templates_nortos_r5fss01 =
[
    {
        input: ".project/templates/am263x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "mcspi_peripheral_main",
        },
    }
];


const buildOptionCombos = [
    { device: "am263x", cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263x-cc", os: "nortos", isPartOfSystemProject: true},
    { device: "am263x", cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am263x-cc", os: "nortos", isPartOfSystemProject: true},
];

const systemProjects = [
    {
        name: "gpio_controller_mcspi_peripheral",
        tag: "nortos",
        skipProjectSpec: false,
        readmeDoxygenPageTag: readmeDoxygenPageTag,
        board: "am263x-cc",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263x-cc", os: "nortos", isPartOfSystemProject: true},
            { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am263x-cc", os: "nortos", isPartOfSystemProject: true},
        ],
    },
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "gpio_controller_mcspi_peripheral";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.filedirs = filedirs;
    build_property.libdirs = libdirs_nortos;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;

if(buildOption.cpu.match(/r5f*/)) {
    build_property.libs = libs_nortos_r5f;

    if(buildOption.cpu.match(/r5fss0-0/)) {
            build_property.files = files_nortos_rf5_00;
            build_property.templates = templates_nortos_r5fss00;
        }
        else{
            build_property.files = files_nortos_rf5_01;
            build_property.templates = templates_nortos_r5fss01;
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
