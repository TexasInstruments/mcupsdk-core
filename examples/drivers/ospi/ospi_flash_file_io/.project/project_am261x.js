let path = require('path');

let device = "am261x";

const files = {
            common:
            [
                "ospi_flash_file_io.c",
                "main.c",
            ],
    }

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
        "${MCU_PLUS_SDK_PATH}/source/fs/littlefs/lib"
    ],
};

const libdirs_freertos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/source/fs/littlefs/lib"
    ],
};

const includes_freertos_r5f_common = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am261x/r5f",
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
        "littlefs.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_nortos_r5f_iar_arm = {
    common: [
        "nortos.am261x.r5f.iar-arm.${ConfigName}.lib",
        "drivers.am261x.r5f.iar-arm.${ConfigName}.lib",
        "board.am261x.r5f.iar-arm.${ConfigName}.lib",
        "littlefs.am261x.r5f.iar-arm.${ConfigName}.lib",
    ],
};

const libs_freertos_r5f_ti_arm_clang = {
    common: [
        "freertos.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "littlefs.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_freertos_r5f_iar_arm = {
    common: [
        "freertos.am261x.r5f.iar-arm.${ConfigName}.lib",
        "drivers.am261x.r5f.iar-arm.${ConfigName}.lib",
        "board.am261x.r5f.iar-arm.${ConfigName}.lib",
        "littlefs.am261x.r5f.iar-arm.${ConfigName}.lib",
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

const syscfgfile = "example.syscfg"

const readmeDoxygenPageTag = "EXAMPLES_DRIVERS_OSPI_FLASH_FILE_IO";


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
    property.name = "ospi_flash_file_io";
    property.isInternal = false;
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
        if(buildOption.cgt.match(/ti-arm-clang*/)) {
            build_property.lnkfiles = lnkfiles_ti_arm_clang;
            if (buildOption.os.match(/freertos*/)) {
                build_property.includes = includes_freertos_r5f_ti_arm_clang;
                build_property.libdirs = libdirs_freertos;
                build_property.libs = libs_freertos_r5f_ti_arm_clang;
            }
            else {
                build_property.libs = libs_nortos_r5f_ti_arm_clang;
            }
        }
        else if(buildOption.cgt.match(/iar-arm*/)) {
            build_property.lnkfiles = lnkfiles_iar_arm;
            if (buildOption.os.match(/freertos*/)) {
                build_property.includes = includes_freertos_r5f_iar_arm;
                build_property.libdirs = libdirs_freertos;
                build_property.libs = libs_freertos_r5f_iar_arm;
            }
            else {
                build_property.libs = libs_nortos_r5f_iar_arm;
            }
        }
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
