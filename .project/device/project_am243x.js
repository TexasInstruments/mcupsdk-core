const common = require("../common.js");

const component_file_list = [
    "source/board/.project/project.js",
    "source/cmsis/.project/project.js",
    "source/drivers/.project/project.js",
    "source/fs/freertos_fat/.project/project.js",
    "source/industrial_comms/ethercat_slave/icss_fwhal/.project/project.js",
    "source/industrial_comms/ethercat_slave/stack/patch/.project/project.js",
    "source/industrial_comms/ethernetip_adapter/icss_fwhal/.project/project_mii.js",
    "source/industrial_comms/ethernetip_adapter/icss_fwhal/.project/project_rgmii.js",
    "source/industrial_comms/ethernetip_adapter/stack/lwip/.project/project_contrib.js",
    "source/industrial_comms/ethernetip_adapter/stack/lwip/.project/project_stack.js",
    "source/industrial_comms/hsr_prp/icss_fwhal/.project/project_hsr_mii.js",
    "source/industrial_comms/hsr_prp/icss_fwhal/.project/project_hsr_rgmii.js",
    "source/industrial_comms/hsr_prp/icss_fwhal/.project/project_prp_mii.js",
    "source/industrial_comms/hsr_prp/icss_fwhal/.project/project_prp_rgmii.js",
    "source/industrial_comms/profinet_device/icss_fwhal/.project/project_irt_mii.js",
    "source/industrial_comms/profinet_device/icss_fwhal/.project/project_irt_rgmii.js",
    "source/industrial_comms/profinet_device/icss_fwhal/.project/project_rt_mrp_mii.js",
    "source/industrial_comms/profinet_device/icss_fwhal/.project/project_rt_mrp_rgmii.js",
    "source/kernel/nortos/.project/project.js",
    "source/kernel/freertos/.project/project.js",
    "source/mathlib/.project/project.js",
    "source/motor_control/position_sense/endat/.project/project.js",
    "source/motor_control/position_sense/hdsl/.project/project.js",
    "source/motor_control/position_sense/tamagawa/.project/project.js",
    "source/networking/icss_emac/.project/project.js",
    "source/networking/icss_emac/lwipif/.project/project.js",
    "source/networking/icss_timesync/.project/project.js",
    "source/pru_io/.project/project.js",
    "source/security/.project/project.js",
    "source/usb/.project/project_nortos.js",
    "source/usb/.project/project_freertos.js",
    "source/networking/enet/.project/project_cpsw.js",
    "source/networking/enet/.project/project_cpsw_lwipif_freertos.js",
    "source/networking/enet/.project/project_cpsw_lwipif_nortos.js",
    "source/networking/enet/.project/project_icssg_lwipif_freertos.js",
    "source/networking/enet/.project/project_icssg_lwipif_nortos.js",
    "source/networking/enet/.project/project_icssg.js",
    "source/networking/lwip/.project/project_stack_freertos.js",
    "source/networking/lwip/.project/project_stack_nortos.js",
    "source/networking/lwip/.project/project_contrib_freertos.js",
    "source/networking/lwip/.project/project_contrib_nortos.js",
    "source/networking/mbedtls_library/.project/project.js",
    "test/unity/.project/project.js",
    "docs_src/docs/api_guide/doxy_samples/.project/project.js",
];

const device_defines = {
    common: [
        "SOC_AM243X",
    ],
};

const example_file_list = [
    "examples/drivers/adc/adc_singleshot/.project/project.js",
    "examples/drivers/boot/sbl_jtag_uniflash/.project/project.js",
    "examples/drivers/boot/sbl_null/.project/project.js",
    "examples/drivers/boot/sbl_ospi/.project/project.js",
    "examples/drivers/boot/sbl_ospi_multi_partition/.project/project.js",
    "examples/drivers/boot/sbl_sd/.project/project.js",
    "examples/drivers/boot/sbl_uart/.project/project.js",
    "examples/drivers/boot/sbl_uart_uniflash/.project/project.js",
    "examples/drivers/crc/crc_full_cpu/.project/project.js",
    "examples/drivers/ddr/ddr_ecc_test_main_esm/.project/project.js",
    "examples/drivers/ecap/ecap_epwm_loopback/.project/project.js",
    "examples/drivers/epwm/epwm_duty_cycle/.project/project.js",
    "examples/drivers/epwm/epwm_duty_cycle_sync/.project/project.js",
    "examples/drivers/eqep/eqep_capture/.project/project.js",
    "examples/drivers/fsi/fsi_loopback_interrupt/.project/project.js",
    "examples/drivers/fsi/fsi_loopback_polling/.project/project.js",
    "examples/drivers/gpio/gpio_input_interrupt/.project/project.js",
    "examples/drivers/gpio/gpio_led_blink/.project/project.js",
    "examples/drivers/gpio/gpio_multi_led_blink/.project/project.js",
    "examples/drivers/i2c/i2c_led_blink/.project/project.js",
    "examples/drivers/i2c/i2c_read/.project/project.js",
    "examples/drivers/i2c/i2c_temperature/.project/project.js",
    "examples/drivers/ipc/ipc_notify_echo/.project/project.js",
    "examples/drivers/ipc/ipc_rpmsg_echo/.project/project.js",
    "examples/drivers/ipc/ipc_spinlock_sharedmem/.project/project.js",
    "examples/drivers/mcan/mcan_external_loopback_interrupt/.project/project.js",
    "examples/drivers/mcan/mcan_loopback_interrupt/.project/project.js",
    "examples/drivers/mcan/mcan_loopback_polling/.project/project.js",
    "examples/drivers/mcspi/mcspi_loopback/.project/project.js",
    "examples/drivers/mcspi/mcspi_loopback_dma/.project/project.js",
    "examples/drivers/mcspi/mcspi_performance_8bit/.project/project.js",
    "examples/drivers/mcspi/mcspi_performance_32bit/.project/project.js",
    "examples/drivers/mmcsd/mmcsd_raw_io/.project/project.js",
    "examples/drivers/mmcsd/mmcsd_file_io/.project/project.js",
    "examples/drivers/ospi/ospi_flash_io/.project/project.js",
    "examples/drivers/ospi/ospi_flash_dma/.project/project.js",
    "examples/drivers/ospi/ospi_flash_diag/.project/project.js",
    "examples/drivers/pcie/pcie_benchmark/pcie_benchmark_ep/.project/project.js",
    "examples/drivers/pcie/pcie_benchmark/pcie_benchmark_rc/.project/project.js",
    "examples/drivers/pcie/pcie_buf_transfer/pcie_buf_transfer_ep/.project/project.js",
    "examples/drivers/pcie/pcie_buf_transfer/pcie_buf_transfer_rc/.project/project.js",
    "examples/drivers/pcie/pcie_legacy_irq/pcie_legacy_irq_ep/.project/project.js",
    "examples/drivers/pcie/pcie_legacy_irq/pcie_legacy_irq_rc/.project/project.js",
    "examples/drivers/pcie/pcie_msi_irq/pcie_msi_irq_ep/.project/project.js",
    "examples/drivers/pcie/pcie_msi_irq/pcie_msi_irq_rc/.project/project.js",
    "examples/drivers/pcie/pcie_msix_irq/pcie_msix_irq_ep/.project/project.js",
    "examples/drivers/pcie/pcie_msix_irq/pcie_msix_irq_rc/.project/project.js",
    "examples/drivers/safety/reset_isolation/.project/project.js",
    "examples/drivers/safety/reset_isolation_ipc/.project/project.js",
    "examples/drivers/sciclient/sciclient_get_version/.project/project.js",
    "examples/drivers/sciclient/sciclient_set_boardcfg/.project/project.js",
	"examples/drivers/sciclient/sciclient_ccs_init/.project/project",
    "examples/drivers/uart/uart_echo/.project/project.js",
    "examples/drivers/uart/uart_echo_callback/.project/project.js",
    "examples/drivers/uart/uart_echo_dma/.project/project.js",
    "examples/drivers/uart/uart_echo_low_latency_interrupt/.project/project.js",
    "examples/drivers/uart/uart_echo_low_latency_polling/.project/project.js",
    "examples/drivers/udma/udma_adc_read/.project/project.js",
    "examples/drivers/udma/udma_chaining/.project/project.js",
    "examples/drivers/udma/udma_memcpy_interrupt/.project/project.js",
    "examples/drivers/udma/udma_memcpy_polling/.project/project.js",
    "examples/drivers/udma/udma_sw_trigger/.project/project.js",
    "examples/drivers/watchdog/watchdog_interrupt/.project/project.js",
    "examples/empty/.project/project_freertos.js",
    "examples/empty/.project/project_nortos.js",
    "examples/hello_world/.project/project.js",
    "examples/hello_world_cpp/.project/project.js",
    "examples/industrial_comms/ethercat_slave_beckhoff_ssc_demo/.project/project.js",
    "examples/industrial_comms/ethercat_slave_demo/cia402/.project/project.js",
    "examples/industrial_comms/ethercat_slave_demo/ctt/.project/project.js",
    "examples/industrial_comms/ethercat_slave_demo/simple/.project/project.js",
    "examples/industrial_comms/ethernetip_adapter_demo/mii/.project/project.js",
    "examples/industrial_comms/ethernetip_adapter_demo/rgmii/.project/project.js",
    "examples/industrial_comms/hsr_prp_demo/hsr_mii/.project/project.js",
    "examples/industrial_comms/hsr_prp_demo/hsr_rgmii/.project/project.js",
    "examples/industrial_comms/hsr_prp_demo/prp_mii/.project/project.js",
    "examples/industrial_comms/hsr_prp_demo/prp_rgmii/.project/project.js",
    "examples/industrial_comms/iolink_master_demo/.project/project.js",
    "examples/kernel/dpl/dpl_demo/.project/project.js",
    "examples/kernel/dpl/xip_benchmark/.project/project.js",
    "examples/kernel/freertos/posix_demo/.project/project.js",
    "examples/kernel/freertos/task_switch/.project/project.js",
    "examples/mathlib/benchmark/.project/project.js",
    "examples/motor_control/benchmark_demo/.project/project.js",
    "examples/motor_control/endat_diagnostic/single_channel/.project/project.js",
    "examples/motor_control/endat_diagnostic/multi_channel_load_share/.project/project.js",
    "examples/motor_control/endat_diagnostic/multi_channel_single_pru/.project/project.js",
    "examples/motor_control/hdsl_diagnostic/.project/project.js",
    "examples/motor_control/hdsl_diagnostic_with_traces/.project/project.js",
    "examples/motor_control/tamagawa_diagnostic/multi_channel/.project/project.js",
    "examples/motor_control/tamagawa_diagnostic/single_channel/.project/project.js",
    "examples/security/crypto/sa2ul_aes/crypto_aes_cbc_256/.project/project.js",
    "examples/security/crypto/sa2ul_aes/crypto_aes_cbc_128/.project/project.js",
    "examples/security/crypto/sa2ul_aes/crypto_aes_ecb_256/.project/project.js",
    "examples/security/crypto/sa2ul_aes/crypto_aes_ecb_128/.project/project.js",
    "examples/security/crypto/sa2ul_aes/crypto_aes_cmac_128/.project/project.js",
    "examples/security/crypto/sa2ul_aes/crypto_aes_cmac_256/.project/project.js",
    "examples/security/crypto/sa2ul_sha/.project/project.js",
    "examples/security/crypto/sa2ul_hmac_sha/crypto_hmac_sha1/.project/project.js",
    "examples/security/crypto/sa2ul_hmac_sha/crypto_hmac_sha256/.project/project.js",
    "examples/security/crypto/sa2ul_hmac_sha/crypto_hmac_sha512/.project/project.js",
    "examples/security/crypto/sa2ul_hmac_sha/crypto_hmac_sha256_multishot/.project/project.js",
    "examples/security/crypto/sa2ul_rng/.project/project.js",
    "examples/security/crypto/sa2ul_pka/rsa_encryption_decryption/.project/project.js",
    "examples/security/crypto/sa2ul_pka/rsa_signing_verification/.project/project.js",
    "examples/security/crypto/sa2ul_pka/ecdsa_signing_verification/.project/project.js",
    "examples/usb/device/cdc_echo/.project/project_nortos.js",
    "examples/usb/device/cdc_echo/.project/project_freertos.js",
    "source/motor_control/position_sense/endat/firmware/multi_channel_load_share/.project/project.js",
    "source/motor_control/position_sense/endat/firmware/single_channel/.project/project.js",
    "source/motor_control/position_sense/endat/firmware/multi_channel_single_pru/.project/project.js",
    "source/motor_control/position_sense/hdsl/firmware/freerun_225_mhz/.project/project.js",
    "source/motor_control/position_sense/hdsl/firmware/freerun_300_mhz/.project/project.js",
    "source/motor_control/position_sense/hdsl/firmware/sync_225_mhz/.project/project.js",
    "source/motor_control/position_sense/tamagawa/firmware/multi_channel/.project/project.js",
    "source/motor_control/position_sense/tamagawa/firmware/single_channel/.project/project.js",
    "test/board/eeprom/.project/project.js",
    "test/board/flash/.project/project.js",
    "test/board/led/.project/project.js",
    "test/drivers/adc/.project/project.js",
    "test/drivers/ddr/.project/project.js",
    "test/drivers/ecap/.project/project.js",
    "test/drivers/epwm/.project/project.js",
    "test/drivers/eqep/.project/project.js",
    "test/drivers/firewall/.project/project.js",
    "test/drivers/fsi/.project/project.js",
    "test/drivers/gpio/.project/project.js",
    "test/drivers/i2c/.project/project.js",
    "test/drivers/ipc_notify/.project/project.js",
    "test/drivers/ipc_rpmsg/.project/project.js",
    "test/drivers/mcan/.project/project.js",
    "test/drivers/mcspi/mcspi/.project/project.js",
    "test/drivers/mcspi/mcspi_eeprom/.project/project.js",
    "test/drivers/mcspi/mcspi_eeprom_dma/.project/project.js",
    "test/drivers/mcspi/mcspi_master_slave/.project/project.js",
    "test/drivers/mcspi/mcspi_master_slave_dma/.project/project.js",
    "test/drivers/mmcsd/.project/project.js",
    "test/drivers/ospi/.project/project.js",
    "test/drivers/sciclient/.project/project.js",
    "test/drivers/soc/soc_r5f/.project/project.js",
    "test/drivers/soc/soc_m4f/.project/project.js",
    "test/drivers/uart/.project/project.js",
    "test/kernel/dpl/.project/project.js",
    "test/kernel/freertos/.project/project.js",
    "test/networking/mbedtls_test/test_aes/.project/project.js",
    "test/networking/mbedtls_test/test_ssl_x509/.project/project.js",
    "test/networking/mbedtls_test/test_sha_entropy/.project/project.js",
    "test/networking/mbedtls_test/test_pka/.project/project.js",
    "test/syscfg/syscfg_combo_1/.project/project.js",
    "test/security/crypto/test_sa2ul_sha/.project/project.js",
    "test/security/crypto/test_sa2ul_aes/.project/project.js",
    "test/security/crypto/test_sa2ul_rng/.project/project.js",
    "test/security/crypto/test_sa2ul_pka/test_rsa_encryption_decryption/.project/project.js",
    "test/security/crypto/test_sa2ul_pka/test_rsa_signing_verification/.project/project.js",
    "test/security/crypto/test_sa2ul_pka/test_ecdsa_signing_verification/.project/project.js",
    "examples/networking/enet_loopback/.project/project.js",
    "examples/networking/lwip/cpsw_lwip_https/.project/project.js",
    "examples/networking/lwip/enet_lwip_cpsw/.project/project.js",
    "examples/networking/lwip/enet_cpsw_tcpclient/.project/project.js",
    "examples/networking/lwip/enet_cpsw_udpclient/.project/project.js",
    "examples/networking/lwip/enet_cpsw_tcpserver/.project/project.js",
    "examples/networking/lwip/enet_cpsw_rawhttpserver/.project/project.js",
    "examples/networking/lwip/enet_cpsw_socket/.project/project.js",
    "examples/networking/lwip/enet_cpsw_udp_igmp/.project/project.js",
    "examples/networking/lwip/enet_lwip_icssg/.project/project.js",
    "examples/networking/enet_layer2_cpsw/.project/project.js",
    "examples/networking/enet_layer2_icssg/.project/project.js",
    "examples/networking/enet_vlan_icssg/.project/project.js",
    "examples/networking/enet_icssg_tas/.project/project.js",
    "examples/networking/enet_layer2_multi_channel/.project/project.js",
    "examples/networking/enet_layer2_cpsw_switch/.project/project.js",
    "examples/networking/enet_cpsw_est/.project/project.js",
    "examples/pru_io/adc/ads85x8/.project/project.js",
    "examples/pru_io/adc/ads85x8/firmware/.project/project.js",
    "examples/pru_io/adc/ads127/.project/project.js",
    "examples/pru_io/adc/ads127/firmware/.project/project.js",
    "examples/pru_io/empty/firmware/.project/project.js",
    "examples/pru_io/mdio_fw/.project/project.js",
];

function getProjectSpecCpu(cpu) {
    let projectSpecCpu =
    {
        "r5fss0-0": "MAIN_PULSAR_Cortex_R5_0_0",
        "r5fss0-1": "MAIN_PULSAR_Cortex_R5_0_1",
        "r5fss1-0": "MAIN_PULSAR_Cortex_R5_1_0",
        "r5fss1-1": "MAIN_PULSAR_Cortex_R5_1_1",
        "m4fss0-0": "Cortex_M4F_0",
        "icssg0-pru0": "ICSS_G0_PRU_0",
        "icssg0-pru1": "ICSS_G0_PRU_1",
        "icssg0-rtupru0": "ICSS_G0_RTU_PRU_0",
        "icssg0-rtupru1": "ICSS_G0_RTU_PRU_1",
        "icssg0-txpru0": "ICSS_G0_TX_PRU_0",
        "icssg0-txpru1": "ICSS_G0_TX_PRU_1",
        "icssg1-pru0": "ICSS_G1_PRU_0",
        "icssg1-pru1": "ICSS_G1_PRU_1",
        "icssg1-rtupru0": "ICSS_G1_RTU_PRU_0",
        "icssg1-rtupru1": "ICSS_G1_RTU_PRU_1",
        "icssg1-txpru0": "ICSS_G1_TX_PRU_0",
        "icssg1-txpru1": "ICSS_G1_TX_PRU_1",
    }

    return projectSpecCpu[cpu];
}

function getComponentList() {
    return component_file_list;
}

function getExampleList() {
    return example_file_list;
}

function getSysCfgDevice(board) {
    switch (board) {
        case "am243x-lp":
            return "AM243x_ALX_beta";
        default:
        case "am243x-evm":
            return "AM243x_ALV_beta";
    }
}

function getProjectSpecDevice(board) {
    switch (board) {
        case "am243x":
            return "AM243x";
        case "am243x-lp":
            return "AM2434_ALX";
        default:
        case "am243x-evm":
            return "AM2434_ALV";
    }
}

function getSysCfgCpu(cpu) {
    return cpu;
}

function getSysCfgPkg(board) {
    switch (board) {
        case "am243x-lp":
            return "ALX";
        default:
        case "am243x-evm":
            return "ALV";
    }
}

function getSysCfgPart(board) {
    switch (board) {
        case "am243x-lp":
            return "ALX";
        default:
        case "am243x-evm":
            return "ALV";
    }
}

function getDevToolTirex(board) {
    switch (board) {
        case "am243x-lp":
            return "AM243x_LAUNCHPAD";
        default:
        case "am243x-evm":
            return "AM243x_GP_EVM";
    }
}

function getProperty() {
    let property = {};

    property.defines = device_defines;

    return property;
}

function getLinuxFwName(cpu) {

    switch(cpu) {
        case "r5fss0-0":
            return "am243-main-r5f0_0-fw";
        case "r5fss0-1":
            return "am243-main-r5f0_1-fw";
        case "r5fss1-0":
            return "am243-main-r5f1_0-fw";
        case "r5fss1-1":
            return "am243-main-r5f1_1-fw";
        case "m4fss0-0":
            return "am243-mcu-m4f0_0-fw";
    }
    return undefined;
}

function getProductNameProjectSpec() {
    return "MCU_PLUS_SDK_AM243X";
}

function getFlashAddr() {
    return 0x60000000;
}

module.exports = {
    getComponentList,
    getExampleList,
    getSysCfgDevice,
    getSysCfgCpu,
    getSysCfgPkg,
    getSysCfgPart,
    getProjectSpecDevice,
    getProjectSpecCpu,
    getDevToolTirex,
    getProperty,
    getLinuxFwName,
    getProductNameProjectSpec,
    getFlashAddr,
};
