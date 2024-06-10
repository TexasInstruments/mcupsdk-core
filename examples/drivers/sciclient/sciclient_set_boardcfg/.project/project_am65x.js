let path = require('path');

let device = "am65x";

const files = {
    common: [
        "sciclient_set_boardcfg.c",
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
        "../../../sciclient_default_boardcfg",
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
        "nortos.am65x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am65x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg"
const readmeDoxygenPageTag = "EXAMPLES_DRIVERS_SCICLIENT_SET_BOARDCFG";
const templates_nortos_r5f =
[
    {
        input: ".project/templates/am65x/common/linker_r5f.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/am65x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "sciclient_set_boardcfg_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am65x-idk", os: "nortos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "sciclient_set_boardcfg";
    property.isInternal = false;
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

	if(buildOption.os.match(/nortos*/) ){

	build_property.libdirs	= libdirs_nortos;
	build_property.templates = templates_nortos_r5f;
	build_property.libs = libs_nortos_r5f;

	}

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
