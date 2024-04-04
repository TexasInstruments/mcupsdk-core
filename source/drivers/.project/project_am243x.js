let path = require('path');

let device = "am243x";

const files_r5f = {
    common: [
        "adc.c",
        "bootloader.c",
        "bootloader_buf_io.c",
        "bootloader_flash.c",
        "bootloader_mem.c",
        "bootloader_mmcsd_raw.c",
        "bootloader_soc.c",
        "bootloader_xmodem.c",
        "bootloader_uniflash.c",
        "bootloader_profile.c",
        "xmodem.c",
        "crc16.c",
        "crc.c",
        "csl_bcdma.c",
        "csl_emif.c",
        "csl_intaggr.c",
        "csl_lcdma_ringacc.c",
        "csl_pktdma.c",
        "csl_ringacc.c",
        "csl_sec_proxy.c",
        "csl_serdes3.c",
        "csl_serdes3_pcie.c",
        "csl_wiz16m_ct2_refclk100MHz_32b_PCIe.c",
        "csl_wiz16m_ct2_refclk19p2MHz_32b_PCIe.c",
        "csl_wiz16m_ct2_refclk20MHz_32b_PCIe.c",
        "csl_wiz16m_ct2_refclk24MHz_32b_PCIe.c",
        "csl_wiz16m_ct2_refclk25MHz_32b_PCIe.c",
        "csl_wiz16m_ct2_refclk26MHz_32b_PCIe.c",
        "csl_wiz16m_ct2_refclk27MHz_32b_PCIe.c",
        "ecap.c",
        "epwm.c",
        "eqep.c",
        "esm_v1.c",
        "elm_v0.c",
        "firewall.c",
        "fsi_rx.c",
        "fsi_tx.c",
        "gpio.c",
        "gpmc_v0.c",
        "gpmc_dma.c",
        "gpmc_dma_udma.c",
        "gtc.c",
        "i2c_v0.c",
        "i2c_v0_lld.c",
        "ipc_notify_v0.c",
        "ipc_notify_v0_cfg.c",
        "ipc_rpmsg.c",
        "ipc_rpmsg_vring.c",
        "lpddr4_16bit_ctl_regs_rw_masks.c",
        "lpddr4_obj_if.c",
        "lpddr4.c",
        "lpddr4_16bit.c",
        "ddr.c",
        "ddr_soc.c",
        "mcan.c",
        "mcspi_v0.c",
        "mcspi_v0_lld.c",
        "mcspi_dma_udma.c",
        "mdio_v0.c",
        "mmcsd_v0.c",
        "mmcsd_priv.c",
        "ospi_v0.c",
        "ospi_dma.c",
        "ospi_dma_udma.c",
        "ospi_nor_flash.c",
        "ospi_phy_dqs.c",
        "pcie.c",
        "pcie_v0.c",
        "pcie_serdes.c",
        "pcie_soc.c",
        "pinmux.c",
        "pmu.c",
        "pruicss_g_v0.c",
        "pruicss_g_v0_cfg.c",
        "sciclient.c",
        "sciclient_pm.c",
        "sciclient_rm.c",
        "sciclient_rm_irq.c",
        "sciclient_procboot.c",
        "sciclient_firewall.c",
        "sciclient_irq_rm.c",
        "sciclient_fmwSecureProxyMap.c",
        "sciclient_boardcfg.c",
        "sciclient_soc_priv.c",
        "soc.c",
        "spinlock.c",
		"csl_timer_mgr.c",
        "uart_v0.c",
        "uart_v0_lld.c",
        "uart_dma_udma.c",
        "udma.c",
        "udma_ch.c",
        "udma_event.c",
        "udma_flow.c",
        "udma_ring_common.c",
        "udma_ring_lcdma.c",
        "udma_rm.c",
        "udma_rmcfg.c",
        "udma_rmcfg_common.c",
        "udma_soc.c",
        "udma_utils.c",
        "watchdog_rti.c",
        "watchdog_soc.c",
    ],
};

const files_m4f = {
    common: [
        "adc.c",
        "crc.c",
        "csl_sec_proxy.c",
        "ecap.c",
        "epwm.c",
        "eqep.c",
        "fsi_rx.c",
        "fsi_tx.c",
        "gpio.c",
        "gtc.c",
        "i2c_v0.c",
        "i2c_v0_lld.c",
        "ipc_notify_v0.c",
        "ipc_notify_v0_cfg.c",
        "ipc_rpmsg.c",
        "ipc_rpmsg_vring.c",
        "mcan.c",
        "mcspi_v0.c",
        "mcspi_v0_lld.c",
        "mcspi_dma_dummy.c",
        "ospi_v0.c",
        "pinmux.c",
        "spinlock.c",
        "sciclient.c",
        "sciclient_pm.c",
        "sciclient_rm.c",
        "sciclient_rm_irq.c",
        "sciclient_procboot.c",
        "sciclient_firewall.c",
        "sciclient_irq_rm.c",
        "sciclient_fmwSecureProxyMap.c",
        "sciclient_boardcfg.c",
        "sciclient_soc_priv.c",
        "soc.c",
        "uart_v0.c",
        "uart_v0_lld.c",
        "uart_dma_dummy.c",
        "watchdog_rti.c",
        "watchdog_soc.c",
    ],
};

const filedirs = {
    common: [
        "adc/v0",
        "bootloader",
        `bootloader/soc/am64x_am243x`,
        "crc/v0",
        "ddr/v0",
        "ddr/v0/cdn_drv/",
        "ddr/v0/cdn_drv/priv",
        "ddr/v0/cdn_drv/include",
        "ddr/v0/cdn_drv/include/16bit",
        "ddr/v0/cdn_drv/include/32bit",
        "ddr/v0/cdn_drv/include/common",
        "ddr/v0/cdn_drv/src",
        "ddr/v0/soc/am64x_am243x",
        "epwm/v0",
        "ecap/v0",
        "eqep/v0",
        "esm/v1",
        "elm/v0",
        "firewall/v0",
        "fsi/v0",
        "gpio/v0",
        "gpmc/v0",
        "gpmc/v0/dma",
        "gpmc/v0/dma/udma",
        "gtc/v0",
        "hw_include/ringacc/V0/priv",
        "hw_include/serdes_cd/V1",
        "hw_include/serdes_cd/V1/V1_1",
        "i2c/v0",
        "i2c/v0/lld",
        "ipc_notify/v0",
        `ipc_notify/v0/soc/${device}`,
        "ipc_rpmsg/",
        "mcan/v0",
        "mcspi/v0",
        "mcspi/v0/lld",
        "mcspi/v0/lld/dma",
        "mcspi/v0/lld/dma/udma",
        "mcspi/v0/lld/dma/dummy",
        "mdio/v0",
        "mmcsd/v0",
        "ospi",
        "ospi/v0",
        "ospi/v0/dma",
        "ospi/v0/dma/udma",
        "pcie",
        "pcie/v0",
        "pcie/v0/soc/am64x_am243x",
        `pinmux/am64x_am243x`,
        `pruicss/g_v0`,
        `pruicss/soc/am64x_am243x`,
        "sciclient",
        `sciclient/soc/am64x_am243x`,
        "soc/am64x_am243x",
        "spinlock/v0",
		"timer_mgr/v0/priv",
        "uart/v0",
        "uart/v0/lld",
        "uart/v0/lld/dma",
        "uart/v0/lld/dma/udma",
        "uart/v0/lld/dma/dummy",
        "udma",
        "udma/hw_include",
        "udma/soc",
        `udma/soc/am64x_am243x`,
        "watchdog/v1",
        `watchdog/v1/soc/am64x_am243x`,
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

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
    { device: device, cpu: "r5f", cgt: "gcc-armv7"},
    { device: device, cpu: "m4f", cgt: "ti-arm-clang"},
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
        if(buildOption.cgt.match(/ti-arm-clang*/)) {
            build_property.cflags = cflags_r5f;
        }
        build_property.filedirs = {common: [...filedirs.common, ...filedirs_r5f.common]};
        build_property.files = files_r5f;
        build_property.asmfiles = asmfiles_r5f;
    }
    if(buildOption.cpu.match(/m4f*/)) {
        build_property.files = files_m4f;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};