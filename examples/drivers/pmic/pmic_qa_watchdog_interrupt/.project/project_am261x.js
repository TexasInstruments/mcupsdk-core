let path = require('path');

let device = "am261x";

const files_derby = {
    common: [
        "pmic_qa_watchdog_derby_interrupt.c",
        "main.c",
        "board.c",
    ],
};

const files_blackbird = {
    common: [
        "pmic_qa_watchdog_blackbird_interrupt.c",
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
        "${MCU_PLUS_SDK_PATH}/source/board/pmic/lib",
    ],
};

const libdirs_freertos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/pmic/lib",
    ],
};

const includes_freertos_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am261x/r5f",
    ],
};


const libs_nortos_r5f_derby = {
    common: [
        "nortos.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "pmic_derby.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_nortos_r5f_blackbird = {
    common: [
        "nortos.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "pmic_blackbird.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_freertos_r5f_derby = {
    common: [
        "freertos.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "pmic_derby.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_freertos_r5f_blackbird = {
    common: [
        "freertos.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "pmic_blackbird.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_PMIC_QA_WATCHDOG_INTERRUPT";

const templates_nortos_r5f =
[
    {
        input: ".project/templates/am261x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "pmic_qa_watchdog_interrupt_main",
        },
    }
];

const templates_freertos_r5f =
[
    {
        input: ".project/templates/am261x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "pmic_qa_watchdog_interrupt_main",
        },
    }
];


const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-lp", os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-lp", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-som", os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-som", os: "freertos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "pmic_qa_watchdog_interrupt";
    property.isInternal = false;
    property.description = "An example to show PMIC watchdog operation in Q/A mode which generates a Interrupt when MCU fails to send watchdog signal. "
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
        if(buildOption.board.match(/am261x-lp*/) ){
            build_property.files = files_derby;
            if(buildOption.os.match(/freertos*/)) {
                build_property.includes = includes_freertos_r5f;
                build_property.libdirs = libdirs_freertos;
                build_property.libs = libs_freertos_r5f_derby;
                build_property.templates = templates_freertos_r5f;
            }
            else {
                build_property.libs = libs_nortos_r5f_derby;
                build_property.templates = templates_nortos_r5f;
            }
        }else{
            build_property.files = files_blackbird;
            if(buildOption.os.match(/freertos*/)) {
                build_property.includes = includes_freertos_r5f;
                build_property.libdirs = libdirs_freertos;
                build_property.libs = libs_freertos_r5f_blackbird;
                build_property.templates = templates_freertos_r5f;
            }
            else {
                build_property.libs = libs_nortos_r5f_blackbird;
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
