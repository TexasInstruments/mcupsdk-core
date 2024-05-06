let path = require('path');

let device = "am243x";

const files = {
    common: [
        "cdc_echo_freertos.c",
        "usb_descriptors.c",
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

const defines = {
    common: [
        "TINYUSB_INTEGRATION"
    ],
};

const includes_freertos_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/usb/tinyusb/config/freertos/am64x_am243x/cdc_config",
        "${MCU_PLUS_SDK_PATH}/source/usb/tinyusb/tinyusb-stack/src",
		"${MCU_PLUS_SDK_PATH}/source/kernel/dpl",
		"${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
		"${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am243x/r5f",
		"${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
    ],
};

const libdirs_freertos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/usb/cdn/lib",
        "${MCU_PLUS_SDK_PATH}/source/usb/tinyusb/lib",
    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "usbd_cdn_freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "usbd_tusb_cdc_freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_freertos_r5f_gcc = {
    common: [
        "freertos.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "board.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "drivers.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "usbd_cdn_freertos.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "usbd_tusb_cdc_freertos.am243x.r5f.gcc-armv7.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_USB_CDC_ECHO";

const templates_freertos_r5f =
[
];

const templates_freertos_r5f_gcc =
[
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "freertos",isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am243x-evm", os: "freertos",isPartOfSystemProject: true},
	{ device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "freertos",isPartOfSystemProject: true},
	{ device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am243x-lp", os: "freertos",isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-evm", os: "freertos",isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-1", cgt: "gcc-armv7", board: "am243x-evm", os: "freertos",isPartOfSystemProject: true},
	{ device: device, cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-lp", os: "freertos",isPartOfSystemProject: true},
	{ device: device, cpu: "r5fss0-1", cgt: "gcc-armv7", board: "am243x-lp", os: "freertos",isPartOfSystemProject: true},

];

const systemProject = [
    {
        name: "cdc_echo_freertos",
        tag: "freertos",
        skipProjectSpec: false,
        board: "am243x-evm",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "freertos"},
            { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am243x-evm", os: "freertos"},
        ],
    },
    {
        name: "cdc_echo_freertos",
        tag: "freertos",
        skipProjectSpec: false,
        board: "am243x-lp",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "freertos"},
            { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am243x-lp", os: "freertos"},
        ],
    },
    {
        name: "cdc_echo_freertos",
        tag: "freertos_gcc-armv7",
        skipProjectSpec: false,
        board: "am243x-evm",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-evm", os: "freertos"},
            { device: device, cpu: "r5fss0-1", cgt: "gcc-armv7", board: "am243x-evm", os: "freertos"},
        ],
    },
    {
        name: "cdc_echo_freertos",
        tag: "freertos_gcc-armv7",
        skipProjectSpec: false,
        board: "am243x-lp",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "gcc-armv7", board: "am243x-lp", os: "freertos"},
            { device: device, cpu: "r5fss0-1", cgt: "gcc-armv7", board: "am243x-lp", os: "freertos"},
        ],
    },
]
function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "cdc_echo_freertos";
    property.isInternal = false;
    property.description = "A USB Device CDC echo."
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.libdirs = libdirs_freertos;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    if(buildOption.cpu.match(/r5f*/)) {
        build_property.defines = defines;
        build_property.includes = includes_freertos_r5f;
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
