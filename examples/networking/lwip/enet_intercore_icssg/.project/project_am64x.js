let path = require('path');

let device = "am64x";

const main_files = {
    common: [
            "udp_iperf.c",
            "app_control.c",
            "ti_ic_open_close.c",
            "netif_common.c",
            "app_netif.c",
            "app_icssgconfighandler.c",
            "app_tcpserver.c",
            "app_main.c",
            "main.c",
    ],
};

const remote_files = {
    common: [
            "netif_common.c",
            "udp_iperf.c",
            "app_netif.c",
            "app_control.c",
            "ti_ic_open_close.c",
            "app_tcpclient.c",
            "app_main.c",
            "main.c",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const main_filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../../common", /* Example base */
        "../../../core_main", /* Example base */
    ],
};

const remote_filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../../common", /* Example base */
        "../../../core_remote", /* Example base */
        "../../../../../../source/networking/enet/core/lwip_ic/intercore/src", /* IC base */
        "../../../../../../source/networking/enet/core/lwip_ic/lwipific/src", /* IC base */
    ],
};

const libdirs_freertos = {
    common: [
	    "generated",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lib",

    ],
};

const includes_freertos_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am64x/r5f",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/utils",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/utils/include",
	    "${MCU_PLUS_SDK_PATH}/source/networking/enet/utils/V3",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/phy",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/k3/am64x_am243x",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/mdio/V4",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/freertos/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/lwipif/inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/lwip_ic/lwipif/inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am64x",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/lwip_ic",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/lwip_ic/lwipific/inc",
        "${MCU_PLUS_SDK_PATH}/examples/networking/lwip/enet_intercore_icssg/common",

    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "enet-icssg.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "lwipif-icssg-freertos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "lwip-freertos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "lwipif-ic-freertos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "lwip-contrib-freertos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const linker_includePath_freertos = {
    common: [
        "${PROJECT_BUILD_DIR}/syscfg",

    ],
};

const defines_r5f = {
    common: [
        "ENET_ENABLE_PER_ICSSG=1",
    ],
};

const cflags_r5f = {
    release: [
        "-Oz",
        "-flto",
    ],
};

const lflags_r5f = {
    common: [
        "--zero_init=on",
        "--use_memset=fast",
        "--use_memcpy=fast",
        "--mapfile_contents=noltosymrefs"
    ],
};

const loptflags_r5f = {
    release: [
        "-mcpu=cortex-r5",
        "-mfloat-abi=hard",
        "-mfpu=vfpv3-d16",
        "-mthumb",
        "-Oz",
        "-flto"
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_ENET_INTERCORE_ICSSG";

const templates_freertos_r5f =
[
    {
        input: ".project/templates/am64x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "appMain",
            taskPri : "1",
            stackSize : "8192",
        },
    },
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "freertos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am64x-evm", os: "freertos", isPartOfSystemProject: true},
];

const systemProject = [
    {
        name: "enet_intercore_icssg",
        tag: "enet_intercore_icssg",
        skipProjectSpec: false,
        readmeDoxygenPageTag: readmeDoxygenPageTag,
        board: "am64x-evm",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "freertos"},
            { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am64x-evm", os: "freertos"},
            ]
    },
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "enet_intercore_icssg";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    if(buildOption.cpu.match(/r5fss0-0/)) {
        build_property.files = main_files;
    }
    else {
        build_property.files = remote_files;
    }

    build_property.syscfgfile = syscfgfile;
    build_property.projecspecFileAction = "link";
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;

    if(buildOption.cpu.match(/r5f*/)) {
        if(buildOption.os.match(/freertos*/) )
        {
            const _ = require('lodash');
            let libdirs_freertos_cpy = _.cloneDeep(libdirs_freertos);
            /* Logic to remove generated/ from libdirs_freertos, it generates warning for ccs build */
            if (buildOption.isProjectSpecBuild === true)
            {
                var delIndex = libdirs_freertos_cpy.common.indexOf('generated');
                if (delIndex !== -1) {
                    libdirs_freertos_cpy.common.splice(delIndex, 1);
                }
            }
            build_property.includes = includes_freertos_r5f;
            build_property.libdirs = libdirs_freertos_cpy;
            build_property.libs = libs_freertos_r5f;
            build_property.lnkfiles = lnkfiles;
            build_property.templates = templates_freertos_r5f;
            if(buildOption.cpu.match(/r5fss0-0/)) {
                build_property.filedirs = main_filedirs;
            }
            else {
                build_property.filedirs = remote_filedirs;
            }
            build_property.defines = defines_r5f;
            build_property.cflags = cflags_r5f;
            build_property.lflags = lflags_r5f;
            build_property.projectspecLnkPath = linker_includePath_freertos;
            build_property.loptflags = loptflags_r5f;
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



