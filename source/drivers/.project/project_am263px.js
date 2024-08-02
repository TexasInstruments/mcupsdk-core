let path = require('path');

let device = "am263px";

const files_r5f = {
    common: [
		"adc.c",
		"bootloader.c",
		"bootloader_buf_io.c",
		"bootloader_can.c",
		"bootloader_flash.c",
		"bootloader_hsmRt_load.c",
		"bootloader_mem.c",
		"bootloader_profile.c",
		"bootloader_soc.c",
		"bootloader_uniflash_common.c",
		"bootloader_uniflash.c",
		"bootloader_uniflash_rprc.c",
		"bootloader_uniflash_mcelf.c",
		"bootloader_xmodem.c",
        "xmodem.c",
		"canfd.c",
        "canfd_soc.c",
        "canfd_dma_edma.c",
        "crc16.c",
		"cmpss.c",
		"dac.c",
		"ecap.c",
		"edma.c",
		"eqep.c",
        "etpwm.c",
        "flsopskd_v0.c",
        "fota_fw_arr.c",
		"fota_agent.c",
		"fsi_rx.c",
		"fsi_tx.c",
		"fss_v0.c",
		"gpio.c",
		"i2c_v1.c",
        "i2c_v1_lld.c",
		"ipc_notify_v1.c",
		"ipc_notify_v1_cfg.c",
		"ipc_rpmsg.c",
		"ipc_rpmsg_vring.c",
		"lin.c",
		"mcan.c",
		"mcspi_v0_lld.c",
		"mcspi_dma_edma.c",
		"mcspi_v0.c",
		"mdio_v0.c",
		"mmcsd_v1.c",
		"mmcsd_v1_lld.c",
		"mmcsd_parse.c",
        "mpu_firewall.c",
        "mpu_firewall_v0_cfg.c",
		"ospi_dma.c",
		"ospi_dma_edma.c",
		"ospi_nor_flash.c",
		"ospi_phy_am26x.c",
		"ospi_v0.c",
		"pinmux.c",
		"pmu.c",
		"pruicss_m_v0.c",
		"pruicss_m_v0_cfg.c",
		"rti.c",
		"resolver.c",
		"sdfm.c",
		"soc.c",
		"soc_rcm.c",
		"spinlock.c",
		"uart_v0.c",
        "uart_v0_lld.c",
        "uart_dma_edma.c",
		"watchdog_rti.c",
		"watchdog_soc.c",
        "rl2.c",
        "flc.c"
	],
};

const filedirs = {
    common: [
    	`pinmux/am263px`,
		`pruicss/m_v0`,
		`pruicss/soc/am263px`,
		`watchdog/v0/soc/${device}`,
		"adc/v2",
		"bootloader",
		"bootloader/bootloader_uniflash",
		"bootloader/bootloader_uniflash/bootloader_uniflash_rprc",
		"bootloader/bootloader_uniflash/bootloader_uniflash_mcelf",
		"bootloader/soc/am263px",
		"cmpss/v0",
		"csl_arm_r5_pmu.S",
		"dac/v0",
		"ecap/v1",
		"edma/v0",
		"epwm/v1",
		"eqep/v1",
		"fota_agent",
		"fsi/v1",
		"fss/v0",
		"gpio/v0",
		"i2c/v1",
        "i2c/v1/lld",
		"ipc_notify/v1",
		"ipc_notify/v1/soc/am263px",
		"ipc_rpmsg/",
		"lin/v0",
		"mcan/v0",
        "mcan/v0/soc/am263px",
        "mcan/v0/dma/edma",
		"mcspi/v0",
        "mcspi/v0/lld",
        "mcspi/v0/lld/dma",
        "mcspi/v0/lld/dma/edma",
		"mdio/v0",
		"mmcsd/v1",
        "mmcsd/v1/lld",
        "mmcsd/v1/lld/internal",
        "mpu_firewall/v0",
        "mpu_firewall/v0/soc/am263px",
		"ospi",
		"ospi/v0",
		"ospi/v0/dma",
		"ospi/v0/dma/edma",
		"ospi/v0/soc/am263px",
		"pmu",
		"pmu/r5f",
		"resolver/v0",
		"rti/v0",
		"sdfm/v0",
		"soc/am263px",
		"spinlock/v0",
		"uart/v0",
        "uart/v0/lld",
        "uart/v0/lld/dma",
        "uart/v0/lld/dma/edma",
		"watchdog/v0",
        "optiflash/v0/flc",
        "optiflash/v0/rl2",
		"flsopskd/v0"
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
	build_property.includes = includes;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
