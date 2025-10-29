let path = require('path');

let device = "am261x";

const files = {
    common: [
        "i2c_led_blink_interrupt_lld.c",
        "main.c",
        "board.c"
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

const libdirs = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
    ],
};

const libs_ti_arm_clang = {
    common: [
        "nortos.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_iar_arm = {
    common: [
        "nortos.am261x.r5f.iar-arm.${ConfigName}.lib",
        "drivers.am261x.r5f.iar-arm.${ConfigName}.lib",
        "board.am261x.r5f.iar-arm.${ConfigName}.lib",
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

const readmeDoxygenPageTag = "EXAMPLES_DRIVERS_I2C_LED_BLINK_INTERRUPT_LLD";

const templates_nortos_r5f =
[
    {
        input: ".project/templates/am261x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "i2c_led_blink_main",
        },
    }
];


const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-lp", os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "iar-arm", board: "am261x-lp", os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-som", os: "nortos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "i2c_led_blink_interrupt_lld";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.libdirs = libdirs;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.templates = templates_nortos_r5f;

    if(buildOption.cgt.match(/ti-arm-clang*/)) {
        build_property.lnkfiles = lnkfiles_ti_arm_clang;
        build_property.libs = libs_ti_arm_clang;
    }
    else if(buildOption.cgt.match(/iar-arm*/)) {
        build_property.lnkfiles = lnkfiles_iar_arm;
        build_property.libs = libs_iar_arm;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
