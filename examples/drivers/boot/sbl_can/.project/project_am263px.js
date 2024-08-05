let path = require('path');

let device = "am263px";

const files = {
    common: [
        "main.c",
        "board.c",
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
        "${MCU_PLUS_SDK_PATH}/source/security/lib",
    ],
};

const libs_nortos_r5f = {
    common: [
        "nortos.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "security.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/security",
    ],
};

const template_options_cc = {
    bootformat: "RPRC",
    board: "am263px-cc"
}

const template_options_lp = {
    bootformat: "RPRC",
    board: "am263px-lp"
}

const templates_cc =
[
    {
        input: ".project/templates/am263px/sbl/sbl_can/main.c.xdt",
        output: "../main.c",
        options: template_options_cc
    },
    {
        input: ".project/templates/am263px/sbl/sbl_can/am263px-cc/board.c.xdt",
        output: "../board.c",
        options: template_options_cc
    }
];


const templates_lp =
[
    {
        input: ".project/templates/am263px/sbl/sbl_can/main.c.xdt",
        output: "../main.c",
        options: template_options_lp
    },
    {
        input: ".project/templates/am263px/sbl/sbl_can/am263px-lp/board.c.xdt",
        output: "../board.c",
        options: template_options_cc
    }
];

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_DRIVERS_SBL_CAN";

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-cc", os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-lp", os: "nortos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "sbl_can";
    property.isInternal = false;
    property.isBootLoader = true;
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
    if(buildOption.board === "am263px-cc")
    {
        build_property.templates = templates_cc;
    }
    else if(buildOption.board === "am263px-lp")
    {
        build_property.templates = templates_lp;
    }
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;

    if(buildOption.cpu.match(/r5f*/)) {
        build_property.libs = libs_nortos_r5f;
    }
    build_property.includes = includes;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
