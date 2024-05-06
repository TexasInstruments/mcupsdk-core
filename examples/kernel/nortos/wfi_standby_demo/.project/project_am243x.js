let path = require('path');

let device = "am243x";

const r5_0_files = {
    common: [
        "i2c_current_monitor.c",
        "main.c",
        "board.c",
    ],
};

const r5_1_files = {
    common: [
        "wfi_demo.c",
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

const defines_nortos = {
    common: [
        "OS_NORTOS",
        "INA226"
    ],
}

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

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const templates_nortos_r5f =
[
    {
        input: ".project/templates/am243x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "wfi_standby_demo_main",
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
            entryFunction: "wfi_standby_demo_main",
        },
    }
];


const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "nortos",isPartOfSystemProject: true},
    { device: device, cpu: "r5fss1-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "nortos",isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-evm", os: "nortos",isPartOfSystemProject: true},
    { device: device, cpu: "r5fss1-0", cgt: "gcc-armv7", board: "am243x-evm", os: "nortos",isPartOfSystemProject: true},
];

const systemProject = [
    {
        name: "wfi_standby_demo",
        tag: "nortos",
        skipProjectSpec: false,
        board: "am243x-evm",
        projects: [
			{ device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "nortos"},
			{ device: device, cpu: "r5fss1-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "nortos"},
        ],
    },
    {
        name: "wfi_standby_demo",
        tag: "nortos_gcc-armv7",
        skipProjectSpec: false,
        board: "am243x-evm",
        projects: [
			{ device: device, cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-evm", os: "nortos"},
			{ device: device, cpu: "r5fss1-0", cgt: "gcc-armv7", board: "am243x-evm", os: "nortos"},
        ],
    },
]

const readmeDoxygenPageTag = "EXAMPLES_KERNEL_NORTOS_WFI_DEMO";

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "wfi_standby_demo";
    property.isInternal = false;
    property.description = "WFI Standy mode example demo"
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

    if(buildOption.cpu.match(/r5fss0-0*/)) {
        build_property.files = r5_0_files;
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
            build_property.defines = defines_nortos;
        }
    if(buildOption.cpu.match(/r5fss1-0*/)) {
        build_property.files = r5_1_files;
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
            build_property.defines = defines_nortos;
        }

    return build_property;
}

function getSystemProjects(device)
{
    return systemProject;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
    getSystemProjects,
};
