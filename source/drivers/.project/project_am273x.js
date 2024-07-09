let path = require('path');

let device = "am273x";

const files_r5f = {
    common: [
        "bootloader.c",
        "bootloader_buf_io.c",
        "bootloader_can.c",
        "bootloader_flash.c",
        "bootloader_mem.c",
        "bootloader_soc.c",
        "bootloader_hsmRt_load.c",
        "bootloader_utils_addrxlate.c",
        "bootloader_xmodem.c",
        "bootloader_uniflash.c",
        "bootloader_profile.c",
        "xmodem.c",
        "cbuff.c",
        "cbuff_lvds.c",
        "cbuff_transfer.c",
        "cbuff_edma.c",
        "crc16.c",
        "crc.c",
        "csirx.c",
        "csirx_soc.c",
        "ecap.c",
        "edma.c",
		"epwm.c",
        "esm_v0.c",
        "gpadc.c",
        "gpadc_soc.c",
        "gpio_v1.c",
        "hwa.c",
        "i2c_v1.c",
        "i2c_v1_lld.c",
        "ipc_notify_v1.c",
        "ipc_notify_v1_cfg.c",
        "ipc_rpmsg.c",
        "ipc_rpmsg_vring.c",
        "mcan.c",
        "mibspi_v0.c",
        "mibspi_edma.c",
        "mibspi_soc.c",
        "mpu_firewall.c",
        "mpu_firewall_v0_cfg.c",
        "pinmux.c",
        "pmu.c",
        "qspi.c",
        "qspi_lld.c",
        "qspi_edma_lld.c",
        "rti.c",
        "soc.c",
        "soc_rcm.c",
        "uart_sci.c",
        "uart_sci_edma.c",
        "watchdog_rti.c",
        "watchdog_soc.c",
    ],
};

const files_c66 = {
    common: [
        "cbuff.c",
        "cbuff_lvds.c",
        "cbuff_transfer.c",
        "cbuff_edma.c",
        "crc.c",
        "csirx.c",
        "csirx_soc.c",
        "ecap.c",
        "edma.c",
        "esm_v0.c",
        "gpio_v1.c",
        "hwa.c",
        "i2c_v1.c",
        "i2c_v1_lld.c",
        "ipc_notify_v1.c",
        "ipc_notify_v1_cfg.c",
        "ipc_rpmsg.c",
        "ipc_rpmsg_vring.c",
        "mcasp.c",
        "mcasp_dma.c",
        "mcan.c",
        "mibspi_v0.c",
        "mibspi_edma.c",
        "mibspi_soc.c",
        "mpu_firewall.c",
        "mpu_firewall_v0_cfg.c",
        "pinmux.c",
        "rti.c",
        "soc.c",
        "soc_rcm.c",
        "uart_sci.c",
        "uart_sci_edma.c",
        "watchdog_rti.c",
        "watchdog_soc.c",
    ],
};

const filedirs = {
    common: [
        "bootloader",
        "bootloader/soc/am273x",
        "cbuff/v1",
        "crc/v1",
        "csirx/v0",
        "csirx/v0/soc/am273x",
        "ecap/v2",
        "edma/v0",
		"epwm/v2",
        "esm/v0",
        "gpadc/v0",
        "gpadc/v0/soc/am273x",
        "gpio/v1",
        "hwa/v0",
        "i2c/v1",
        "i2c/v1/lld",
        "ipc_notify/v1",
        "ipc_notify/v1/soc/am273x",
        "ipc_rpmsg",
        "mcasp/v0",
        "mcan/v0",
        "mibspi/v0",
        "mibspi/v0/edma",
        "mibspi/v0/soc/am273x",
        "mpu_firewall/v0",
        "mpu_firewall/v0/soc/am273x",
        "pinmux/am273x",
        "rti/v0",
        "qspi/v0",
        "qspi/v0/lld",
        "qspi/v0/lld/edma",
        "soc/am273x",
        "uart/v1",
        "watchdog/v0",
        `watchdog/v0/soc/${device}`,
    ],
};

const filedirs_r5f =  {
    common: [
        "pmu",
        "pmu/r5f",
    ]
};

const asmfiles_r5f = {
    common: [
        "csl_arm_r5_pmu.S",
    ]
};

const cflags_r5f = {
    release: [
        "-Oz",
        "-flto",
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/security",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
    { device: device, cpu: "c66", cgt: "ti-c6000"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "drivers";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.filedirs = filedirs;
    if(buildOption.cpu.match(/r5f*/)) {
        build_property.filedirs = {common: [...filedirs.common, ...filedirs_r5f.common]};
        build_property.cflags = cflags_r5f;
        build_property.files = files_r5f;
        build_property.asmfiles = asmfiles_r5f;
    }
    if(buildOption.cpu.match(/c66*/)) {
        build_property.files = files_c66;
    }
    build_property.includes = includes;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
