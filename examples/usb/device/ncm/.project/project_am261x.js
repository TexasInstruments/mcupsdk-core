let path = require('path');

let device = "am261x";

const nortos_files = {
    common: [
        "ncm_main.c",
        "main.c",
    ],
};

const freertos_files = {
    common: [
        "ncm_main_freertos.c",
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

const libdirs_nortos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/source/usb/synp/lib",
        "${MCU_PLUS_SDK_PATH}/source/usb/tinyusb/lib",
    ],
};

const libdirs_freertos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/source/usb/synp/lib",
        "${MCU_PLUS_SDK_PATH}/source/usb/tinyusb/lib",
    ],
};

const includes_nortos_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/usb/tinyusb/config/nortos/am261x/ncm_config",
        "${MCU_PLUS_SDK_PATH}/source/usb/tinyusb/tinyusb-stack/src",
        "${MCU_PLUS_SDK_PATH}/source/usb/tinyusb/tinyusb-stack/lib/networking",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am261x/usb/",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/include/",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/include/lwip/apps",
    ],
};

const includes_freertos_r5f_common = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am261x/r5f",
        "${MCU_PLUS_SDK_PATH}/source/usb/tinyusb/config/nortos/am261x/ncm_config",
        "${MCU_PLUS_SDK_PATH}/source/usb/tinyusb/tinyusb-stack/src",
        "${MCU_PLUS_SDK_PATH}/source/usb/tinyusb/tinyusb-stack/lib/networking",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am261x/usb/",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/include/",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/include/lwip/apps",
    ],
};

const includes_freertos_r5f_ti_arm_clang = {
    common: [
        ...includes_freertos_r5f_common.common,
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
    ],
};

const includes_freertos_r5f_iar_arm = {
    common: [
        ...includes_freertos_r5f_common.common,
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/IAR/ARM_CR5F",
    ],
};


const libs_nortos_r5f_ti_arm_clang = {
    common: [
        "nortos.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "usbd_synp_nortos.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "usbd_tusb_ncm_nortos.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};


const libs_nortos_r5f_iar_arm = {
    common: [
        "nortos.am261x.r5f.iar-arm.${ConfigName}.lib",
        "drivers.am261x.r5f.iar-arm.${ConfigName}.lib",
        "board.am261x.r5f.iar-arm.${ConfigName}.lib",
        "usbd_synp_nortos.am261x.r5f.iar-arm.${ConfigName}.lib",
        "usbd_tusb_ncm_nortos.am261x.r5f.iar-arm.${ConfigName}.lib",
    ],
};

const libs_freertos_r5f_ti_arm_clang = {
    common: [
        "freertos.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "usbd_synp_freertos.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "usbd_tusb_ncm_nortos.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_freertos_r5f_iar_arm = {
    common: [
        "freertos.am261x.r5f.iar-arm.${ConfigName}.lib",
        "drivers.am261x.r5f.iar-arm.${ConfigName}.lib",
        "board.am261x.r5f.iar-arm.${ConfigName}.lib",
        "usbd_synp_freertos.am261x.r5f.iar-arm.${ConfigName}.lib",
        "usbd_tusb_ncm_nortos.am261x.r5f.iar-arm.${ConfigName}.lib",
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

const readmeDoxygenPageTag = "EXAMPLES_USB_NCM";

const templates_nortos_r5f =
[
    {
        input: ".project/templates/am261x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "ncm_main",
        },
    }
];

const templates_freertos_r5f =
[
    {
        input: ".project/templates/am261x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "ncm_main",
        },
    }
];


const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-som", os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-som", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-lp", os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-lp", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "iar-arm", board: "am261x-lp", os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "iar-arm", board: "am261x-lp", os: "freertos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "usb_ncm";
    property.isInternal = false;
    property.description = "A USB Device NCM example"
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.filedirs = filedirs;
    build_property.syscfgfile = syscfgfile;
    build_property.defines = defines;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    
    if(buildOption.cpu.match(/r5f*/)) {
        if(buildOption.cgt.match(/ti-arm-clang*/)) {
            build_property.lnkfiles = lnkfiles_ti_arm_clang;
            if(buildOption.os.match(/freertos*/) )
            {
                build_property.files = freertos_files;
                build_property.includes = includes_freertos_r5f_ti_arm_clang;
                build_property.libdirs = libdirs_freertos;
                build_property.libs = libs_freertos_r5f_ti_arm_clang;
                build_property.templates = templates_freertos_r5f;
            }
            else
            {
                build_property.files = nortos_files;
                build_property.includes = includes_nortos_r5f;
                build_property.libdirs = libdirs_nortos;
                build_property.libs = libs_nortos_r5f_ti_arm_clang;
                build_property.templates = templates_nortos_r5f;
            }
        }
        else if(buildOption.cgt.match(/iar-arm*/)) {
            build_property.lnkfiles = lnkfiles_iar_arm;
            if(buildOption.os.match(/freertos*/) )
            {
                build_property.files = freertos_files;
                build_property.includes = includes_freertos_r5f_iar_arm;
                build_property.libdirs = libdirs_freertos;
                build_property.libs = libs_freertos_r5f_iar_arm;
                build_property.templates = templates_freertos_r5f;
            }
            else
            {
                build_property.files = nortos_files;
                build_property.includes = includes_nortos_r5f;
                build_property.libdirs = libdirs_nortos;
                build_property.libs = libs_nortos_r5f_iar_arm;
                build_property.templates = templates_nortos_r5f;
            }
        }
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
