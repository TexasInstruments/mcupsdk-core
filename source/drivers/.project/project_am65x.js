let path = require('path');

let device = "am65x"

const files_r5f = {
    common: [
        "bootloader.c",
        "bootloader_buf_io.c",
        "bootloader_flash.c",
        "bootloader_mem.c",
        "bootloader_profile.c",
        "bootloader_soc.c",
        "bootloader_uniflash.c",
        "bootloader_uniflash_common.c",
        "bootloader_uniflash_mcelf.c",
        "bootloader_uniflash_rprc.c",
        "bootloader_xmodem.c",
        "crc16.c",
        "csl_intaggr.c",
        "csl_proxy.c",
        "csl_ringacc.c",
        "csl_serdes3_pcie.c",
        "csl_wiz8m_sb_refclk100p0MHz_32b_8Gbps_PCIE_1l1c.c",
        "csl_serdes3.c",
        "csl_sec_proxy.c",
        "csl_udmap.c",
        "ddr.c",
        "gpio.c",
        "gtc.c",
        "i2c_v0.c",
        "i2c_v0_lld.c",
        "ipc_notify_soc.c",
        "ipc_notify_v0.c",
        "ipc_notify_v0_cfg.c",
        "ipc_rpmsg.c",
        "ipc_rpmsg_vring.c",
        "mmcsd_priv.c",
        "mmcsd_v2.c",
        "ospi_v0.c",
        "ospi_dma.c",
        "ospi_dma_udma.c",
        "ospi_nor_flash.c",
        "ospi_phy.c",
        "pinmux.c",
        "pcie.c",
        "pcie_v1.c",
        "pcie_serdes.c",
        "pcie_soc.c",
        "sciclient.c",
        "sciclient_boardcfg.c",
        "sciclient_firewall.c",
        "sciclient_fmwSecureProxyMap.c",
        "sciclient_irq_rm.c",
        "sciclient_pm.c",
        "sciclient_procboot.c",
        "sciclient_rm.c",
        "sciclient_rm_irq.c",
        "sciclient_soc_priv.c",
        "soc.c",
        "uart_v0.c",
        "uart_v0_lld.c",
        "uart_dma_udma.c",
        "uart_dma_soc.c",
        "udma.c",
        "udma_ch.c",
        "udma_event.c",
        "udma_flow.c",
        "udma_ring_common.c",
        "udma_ring_normal.c",
        "udma_rm.c",
        "udma_rmcfg.c",
        "udma_rmcfg_common.c",
        "udma_soc.c",
        "udma_utils.c",
        "xmodem.c",
    ],
};

const filedirs = {
    common: [
        "bootloader",
		"bootloader/bootloader_uniflash",
		"bootloader/bootloader_uniflash/bootloader_uniflash_rprc",
		"bootloader/bootloader_uniflash/bootloader_uniflash_mcelf",
        `bootloader/soc/am65x`,
        "ddr/v1",
        "gpio/v0",
        "gtc/v0",
        "hw_include/ringacc/V0/priv",
        "hw_include/serdes_sb/V1",
        "i2c/v0",
        "i2c/v0/lld",
        "ipc_notify/v0",
        "ipc_notify/v0/soc/",
        `ipc_notify/v0/soc/${device}`,
        "ipc_rpmsg/",
        "mmcsd",
        "mmcsd/v2",
        "ospi",
        "ospi/v0",
        "ospi/v0/dma",
        `ospi/v0/dma/soc/${device}`,
        "ospi/v0/dma/udma",
        `ospi/v0/soc/${device}`,
        "pcie",
        "pcie/v1",
        "pcie/v1/soc/am65x",
        "pinmux/am65x",
        "sciclient",
        "sciclient/soc/am65x",
        "soc/am65x",
        "uart/v0",
        "uart/v0/lld",
        "uart/v0/lld/dma",
        "uart/v0/lld/dma/udma",
        "uart/v0/lld/dma/soc/am65x",
        "udma/v1",
        "udma/hw_include",
        "udma/soc",
        `udma/soc/am65x`,
    ],
};

const filedirs_r5f =  {
    common: [

    ]
};

const asmfiles_r5f = {
    common: [

    ]
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
        build_property.files = files_r5f;
        build_property.asmfiles = asmfiles_r5f;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
