let path = require('path');

let device = "am263px";

const files = {
    common: [
        "main.c",
        "sent_decoder.c",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
        "../../../firmware",
        "."
    ],
};

const libdirs_freertos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/source/pru_io/lib",
    ],
};

const includes_freertos_r5f_am263px_lp = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am263px/r5f",
        "${MCU_PLUS_SDK_PATH}/source/pru_io/driver",
        "${MCU_PLUS_SDK_PATH}/examples/pru_io/sent/decoder_pruicss_iep_ecap/example",
        "${MCU_PLUS_SDK_PATH}/examples/pru_io/sent/decoder_pruicss_iep_ecap/firmware/enhanced_serial_message/am263px-cc",
        "${MCU_PLUS_SDK_PATH}/examples/pru_io/sent/decoder_pruicss_iep_ecap/firmware/no_serial_message/am263px-cc",
        "${MCU_PLUS_SDK_PATH}/examples/pru_io/sent/decoder_pruicss_iep_ecap/firmware/short_serial_message/am263px-cc",
    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am263px.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_SENT_DECODER_PRUICSS_IEP_ECAP";

const templates_freertos_r5f =
[
    {
        input: ".project/templates/am263px/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "sent_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263px-cc", os: "freertos"},
];


function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "sent_decoder_using_iep_capture";
    property.isInternal = false;
    property.description = "SENT Decoder Example Using PRUICSS IEP ECAP"
    property.buildOptionCombos = buildOptionCombos;
    property.tirexResourceSubClass = [ "example.gettingstarted" ];

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.includes = includes_freertos_r5f_am263px_lp;

    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;

    build_property.libdirs = libdirs_freertos;
    build_property.libs = libs_freertos_r5f;
    build_property.templates = templates_freertos_r5f;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
