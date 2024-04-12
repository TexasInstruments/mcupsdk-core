const common = require("../common.js");

const component_file_list = [
    "source/board/.project/project.js",
    "source/cmsis/.project/project.js",
    "source/dhrystone_benchmark/.project/project.js",
    "source/drivers/.project/project.js",
    "source/fs/freertos_fat/.project/project.js",
    "source/kernel/nortos/.project/project.js",
    "source/kernel/freertos/.project/project.js",
    "source/mathlib/.project/project.js",
    "source/pru_io/.project/project.js",
    "source/sdl/.project/project.js",
    "source/security/.project/project.js",
    "source/usb/cdn/.project/project_nortos.js",
    "source/usb/cdn/.project/project_freertos.js",
    "source/usb/tinyusb/.project/.cdc/project_nortos.js",
    "source/usb/tinyusb/.project/.cdc/project_freertos.js",
    "source/usb/tinyusb/.project/.dfu/project_nortos.js",
    "source/usb/tinyusb/.project/.dfu/project_freertos.js",
    "source/usb/tinyusb/.project/.ncm/project_nortos.js",
    "source/usb/tinyusb/.project/.ncm/project_freertos.js",
    "source/usb/tinyusb/.project/.rndis/project_nortos.js",
    "source/usb/tinyusb/.project/.rndis/project_freertos.js",
    "source/networking/enet/.project/project_cpsw.js",
    "source/networking/enet/.project/project_cpsw_lwipif_freertos.js",
    "source/networking/enet/.project/project_cpsw_lwipif_nortos.js",
    "source/networking/enet/.project/project_icssg.js",
    "source/networking/enet/.project/project_icssg_lwipif_freertos.js",
    "source/networking/enet/.project/project_icssg_lwipif_ic_freertos.js",
    "source/networking/enet/.project/project_icssg_lwipif_nortos.js",
    "source/networking/lwip/.project/project_stack_freertos.js",
    "source/networking/lwip/.project/project_stack_nortos.js",
    "source/networking/lwip/.project/project_contrib_freertos.js",
    "source/networking/lwip/.project/project_contrib_nortos.js",
    "source/networking/mbedtls_library/.project/project.js",
    "source/networking/tsn/.project/project_tsn_gptp.js",
    "source/networking/tsn/.project/project_tsn_combase.js",
    "source/networking/tsn/.project/project_tsn_uniconf.js",
    "source/networking/tsn/.project/project_tsn_unibase.js",
    "source/networking/tsn/.project/project_tsn_lldp.js",
    "source/networking/tsn/.project/project_tsn_l2.js",
    "test/unity/.project/project.js",
    "docs_src/docs/api_guide/doxy_samples/.project/project.js",
];

const device_defines = {
    common: [
        "SOC_AM64X",
    ],
};

const example_file_list = [
    "examples/benchmarks/benchmark_demo/.project/project.js",
    "examples/benchmarks/coremark_benchmark/.project/project.js",
    "examples/benchmarks/dhrystone_benchmark/.project/project.js",
    "examples/drivers/adc/adc_singleshot/.project/project.js",
    "examples/drivers/boot/sbl_emmc/.project/project.js",
    "examples/drivers/boot/sbl_emmc_linux/.project/project.js",
    "examples/drivers/boot/sbl_jtag_uniflash/.project/project.js",
    "examples/drivers/boot/sbl_null/.project/project.js",
    "examples/drivers/boot/sbl_ospi/.project/project.js",
    "examples/drivers/boot/sbl_ospi_linux/.project/project.js",
    "examples/drivers/boot/sbl_ospi_multi_partition/.project/project.js",
    "examples/drivers/boot/sbl_pcie/.project/project.js",
    "examples/drivers/boot/sbl_pcie_host/.project/project.js",
    "examples/drivers/boot/sbl_sd/.project/project.js",
    "examples/drivers/boot/sbl_uart/.project/project.js",
    "examples/drivers/boot/sbl_uart_uniflash/.project/project.js",
    "examples/drivers/boot/sbl_dfu_uniflash/.project/project.js",
    "examples/drivers/boot/sbl_dfu/.project/project.js",
    "examples/drivers/crc/crc_full_cpu/.project/project.js",
    "examples/drivers/ddr/ddr_ecc_test_main_esm/.project/project.js",
    "examples/drivers/ddr/ddr_ecc_test_mcu_esm/.project/project.js",
    "examples/drivers/ecap/ecap_apwm_mode/.project/project.js",
    "examples/drivers/ecap/ecap_epwm_loopback/.project/project.js",
    "examples/drivers/epwm/epwm_duty_cycle/.project/project.js",
    "examples/drivers/epwm/epwm_duty_cycle_sync/.project/project.js",
    "examples/drivers/eqep/eqep_capture/.project/project.js",
    "examples/drivers/fsi/fsi_loopback_interrupt/.project/project.js",
    "examples/drivers/fsi/fsi_loopback_polling/.project/project.js",
    "examples/drivers/gpio/gpio_input_interrupt/.project/project.js",
    "examples/drivers/gpio/gpio_led_blink/.project/project.js",
    "examples/drivers/gpmc/gpmc_flash_io/.project/project.js",
    "examples/drivers/i2c/i2c_led_blink/.project/project.js",
    "examples/drivers/i2c/i2c_read/.project/project.js",
    "examples/drivers/i2c/i2c_memory_read/.project/project.js",
    "examples/drivers/i2c/i2c_temperature/.project/project.js",
    "examples/drivers/i2c/i2c_led_blink_polling_lld/.project/project.js",
    "examples/drivers/i2c/i2c_led_blink_interrupt_lld/.project/project.js",
    "examples/drivers/i2c/i2c_led_read_write_lld/.project/project.js",
    "examples/drivers/i2c/i2c_memory_read_polling_lld/.project/project.js",
    "examples/drivers/i2c/i2c_memory_read_interrupt_lld/.project/project.js",
    "examples/drivers/i2c/i2c_temperature_polling_lld/.project/project.js",
    "examples/drivers/i2c/i2c_temperature_interrupt_lld/.project/project.js",
    "examples/drivers/ipc/ipc_notify_echo/.project/project.js",
    "examples/drivers/ipc/ipc_rpmsg_echo/.project/project.js",
    "examples/drivers/ipc/ipc_rpmsg_echo_linux/.project/project.js",
    "examples/drivers/ipc/ipc_safeipc_echo/.project/project.js",
    "examples/drivers/ipc/ipc_spinlock_sharedmem/.project/project.js",
    "examples/drivers/mcan/mcan_external_loopback_interrupt/.project/project.js",
    "examples/drivers/mcan/mcan_loopback_dma/.project/project.js",
    "examples/drivers/mcan/mcan_loopback_interrupt/.project/project.js",
    "examples/drivers/mcan/mcan_loopback_polling/.project/project.js",
    "examples/drivers/mcspi/mcspi_loopback/.project/project.js",
    "examples/drivers/mcspi/mcspi_loopback_dma/.project/project.js",
    "examples/drivers/mcspi/mcspi_loopback_polling_lld/.project/project.js",
    "examples/drivers/mcspi/mcspi_loopback_interrupt_lld/.project/project.js",
    "examples/drivers/mcspi/mcspi_loopback_dma_lld/.project/project.js",
    "examples/drivers/mcspi/mcspi_loopback_multi_instances_multi_channels_lld/.project/project.js",
    "examples/drivers/mcspi/mcspi_performance_8bit/.project/project.js",
    "examples/drivers/mcspi/mcspi_performance_32bit/.project/project.js",
    "examples/drivers/mmcsd/mmcsd_raw_io/.project/project.js",
    "examples/drivers/mmcsd/mmcsd_file_io/.project/project.js",
    "examples/drivers/ospi/ospi_flash_diag/.project/project.js",
    "examples/drivers/ospi/ospi_flash_dma/.project/project.js",
    "examples/drivers/ospi/ospi_flash_io/.project/project.js",
    "examples/drivers/ospi/ospi_flash_xip/.project/project.js",
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
    "examples/drivers/pmu/pmu_multievent/.project/project.js",
    "examples/drivers/safety/reset_isolation/.project/project.js",
    "examples/drivers/safety/reset_isolation_ipc/.project/project.js",
    "examples/drivers/sciclient/sciclient_get_version/.project/project.js",
	"examples/drivers/sciclient/sciclient_ccs_init/.project/project",
    "examples/drivers/spinlock/spinlock_example/.project/project_nortos.js",
    "examples/drivers/uart/uart_echo/.project/project.js",
    "examples/drivers/uart/uart_echo_callback/.project/project.js",
    "examples/drivers/uart/uart_echo_dma/.project/project.js",
    "examples/drivers/uart/uart_echo_dma_lld/.project/project.js",
    "examples/drivers/uart/uart_echo_interrupt_lld/.project/project.js",
    "examples/drivers/uart/uart_echo_low_latency_interrupt/.project/project.js",
    "examples/drivers/uart/uart_echo_low_latency_polling/.project/project.js",
    "examples/drivers/uart/uart_echo_polling_lld/.project/project.js",
    "examples/drivers/udma/udma_adc_read/.project/project.js",
    "examples/drivers/udma/udma_chaining/.project/project.js",
    "examples/drivers/udma/udma_memcpy_interrupt/.project/project.js",
    "examples/drivers/udma/udma_memcpy_polling/.project/project.js",
    "examples/drivers/udma/udma_sw_trigger/.project/project.js",
    "examples/drivers/watchdog/watchdog_interrupt/.project/project.js",
    "examples/drivers/mcspi/mcspi_loopback_turbo_mode/.project/project.js",
    "examples/empty/.project/project_freertos.js",
    "examples/empty/.project/project_nortos.js",
    "examples/hello_world/.project/project.js",
    "examples/hello_world_cpp/.project/project.js",
    "examples/kernel/dpl/dpl_demo/.project/project.js",
    "examples/kernel/dpl/xip_benchmark/.project/project.js",
    "examples/kernel/freertos/dhrystone_benchmark/.project/project.js",
    "examples/kernel/freertos/interrupt_profiling/.project/project.js",
    "examples/kernel/freertos/posix_demo/.project/project.js",
    "examples/kernel/freertos/task_switch/.project/project.js",
    "examples/kernel/freertos/smp_task_switch/.project/project.js",
    "examples/kernel/nortos/wfi_standby_demo/.project/project_am64x.js",
    "examples/mathlib/benchmark/.project/project.js",
    "examples/otp/runtime_swrev/.project/project.js",
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
    "examples/usb/device/dfu/.project/project_nortos.js",
    "examples/usb/device/ncm/.project/project_nortos.js",
    "examples/usb/device/rndis/.project/project_nortos.js",
    "examples/networking/enet_loopback/enet_cpsw_loopback/.project/project.js",
    "examples/networking/lwip/cpsw_lwip_https/.project/project.js",
    "examples/networking/lwip/cpsw_lwip_mqtt/.project/project.js",
    "examples/networking/lwip/enet_lwip_cpsw/.project/project.js",
    "examples/networking/lwip/enet_cpsw_tcpserver/.project/project.js",
    "examples/networking/lwip/enet_cpsw_rawhttpserver/.project/project.js",
    "examples/networking/lwip/enet_cpsw_udpclient/.project/project.js",
    "examples/networking/lwip/enet_cpsw_udp_igmp/.project/project.js",
    "examples/networking/lwip/enet_cpsw_tcpclient/.project/project.js",
    "examples/networking/lwip/enet_cpsw_socket/.project/project.js",
    "examples/networking/enet_layer2_cpsw/.project/project.js",
    "examples/networking/enet_layer2_multi_channel/.project/project.js",
    "examples/networking/enet_layer2_cpsw_switch/.project/project.js",
    "examples/networking/enet_cpsw_est/.project/project.js",
    "examples/networking/tsn/est_cpsw_app/.project/project.js",
    "examples/networking/tsn/gptp_cpsw_app/.project/project.js",
    "examples/networking/tsn/gptp_icssg_app/gptp_icssg_dualmac/.project/project.js",
    "examples/networking/tsn/gptp_icssg_app/gptp_icssg_switch/.project/project.js",
    "examples/networking/tsn/lldp_cpsw_app/.project/project.js",
    "examples/networking/tsn/gptp_lwip_cpsw/.project/project.js",
    "examples/networking/tsn/gptp_lwip_icssg/.project/project.js",
    "examples/networking/enet_loopback/enet_icssg_loopback/.project/project.js",
    "examples/networking/enet_icssg_tas/.project/project.js",
    "examples/networking/enet_layer2_icssg/icssg_layer2_dualmac/.project/project.js",
    "examples/networking/enet_layer2_icssg/icssg_layer2_switch/.project/project.js",
    "examples/networking/enet_vlan_icssg/.project/project.js",
    "examples/networking/lwip/enet_intercore_icssg/.project/project.js",
    "examples/networking/lwip/enet_icssg_tcpserver/.project/project.js",
    "examples/networking/lwip/enet_lwip_icssg/.project/project.js",
    "examples/pru_io/adc/ads85x8/.project/project.js",
    "examples/pru_io/adc/ads85x8/firmware/.project/project.js",
    "examples/pru_io/adc/ads127/.project/project.js",
    "examples/pru_io/adc/ads127/firmware/.project/project.js",
    "examples/pru_io/empty/.project/project.js",
    "examples/pru_io/adc/ads131/.project/project.js",
    "examples/pru_io/adc/ads131/firmware/.project/project.js",
    "examples/pru_io/empty/firmware/.project/project.js",
    "examples/otp/ext_otp/.project/project.js",
    "examples/sdl/mcrc/mcrc_full_cpu/.project/project.js",
    "examples/sdl/stog/.project/project.js",
    "examples/sdl/dcc/dcc_modes/.project/project.js",
    "examples/sdl/pbist/pbist_mpu/.project/project.js",
    "examples/sdl/esm/mcu_esm0/.project/project.js",
    "examples/sdl/pok/pok_mcu/.project/project.js",
    "examples/sdl/rom_checksum/.project/project.js",
    "examples/sdl/rti/UC1/.project/project.js",
    "examples/sdl/rti/UC2/.project/project.js",
    "examples/sdl/rti/UC3/.project/project.js",
    "examples/sdl/rti/UC4/.project/project.js",
    "examples/sdl/stog_r5f/.project/project.js",
    "examples/sdl/mtog/mtog_example/.project/project.js",
    "examples/sdl/ecc/.project/project.js",
    "examples/sdl/vtm/vtm_uc/.project/project.js",
    "test/sdl/mcrc/mcrcUt/.project/project.js",
    "test/sdl/mcrc/mcrcFuncTest/.project/project.js",
    "test/sdl/esm/esm_func_test/.project/project.js",
    "test/sdl/esm/esm_unit_test/.project/project.js",
	"test/sdl/esm/common/.project/project.js",
    "test/sdl/dcc/dcc_unit_test/.project/project.js",
    "test/sdl/dcc/dcc_func_test/.project/project.js",
    "test/sdl/stog/stog_func_test_r5f/.project/project.js",
    "test/sdl/stog/stog_unit_test/.project/project.js",
    "test/sdl/stog/stog_func_test/.project/project.js",
	"test/sdl/vtm/vtm_func_test/.project/project.js",
	"test/sdl/vtm/vtm_unit_test/.project/project.js",
	"test/sdl/mtog/mtog_func_test/.project/project.js",
	"test/sdl/mtog/mtog_unit_test/.project/project.js",
    "test/sdl/ecc/ecc_sdl_funcTest/.project/project.js",
    "test/sdl/ecc/ecc_sdl_unitTest/.project/project.js",
    "test/sdl/pbist/sdl_pbist_test/.project/project.js",
    "test/sdl/pbist/ip_pbist_test/.project/project.js",
	"test/sdl/pok/pokUt/.project/project.js",
    "test/sdl/pok/pokFuncTest/.project/project.js",
	"test/sdl/rti/rtiUt/.project/project.js",
    "test/sdl/rti/rtiFuncTest/.project/project.js",
    "test/sdl/lbist/.project/project.js",
    "test/sdl/rom_checksum/functional_test/.project/project.js",
    "test/sdl/rom_checksum/unit_test/.project/project.js",
    "test/board/eeprom/.project/project.js",
    "test/board/flash/.project/project.js",
    "test/board/led/.project/project.js",
    "test/drivers/adc/.project/project.js",
    "test/drivers/boot/boot_testapp_mb/.project/project.js",
    "test/drivers/ddr/.project/project.js",
    "test/drivers/ecap/.project/project.js",
    "test/drivers/epwm/.project/project.js",
    "test/drivers/eqep/.project/project.js",
    "test/drivers/firewall/.project/project.js",
    "test/drivers/fsi/.project/project.js",
    "test/drivers/gpio/.project/project.js",
    "test/drivers/gpmc/.project/project.js",
    "test/drivers/gtc/.project/project.js",
    "test/drivers/i2c/.project/project.js",
    "test/drivers/ipc_notify/.project/project.js",
    "test/drivers/ipc_rpmsg/.project/project.js",
    "test/drivers/ipc_rpmsg/ipc_safeipc_test/.project/project.js",
    "test/drivers/mcan/.project/project.js",
    "test/drivers/mcspi/mcspi/.project/project.js",
    "test/drivers/mcspi/mcspi_eeprom/.project/project.js",
    "test/drivers/mcspi/mcspi_eeprom_dma/.project/project.js",
    "test/drivers/mcspi/mcspi_controller_peripheral/.project/project.js",
    "test/drivers/mmcsd/.project/project.js",
    "test/drivers/ospi/.project/project.js",
    "test/drivers/sciclient/.project/project.js",
    "test/drivers/soc/soc_r5f/.project/project.js",
    "test/drivers/soc/soc_m4f/.project/project.js",
    "test/drivers/uart/.project/project.js",
    "test/drivers/udma/.project/project.js",
    "test/kernel/dpl/.project/project.js",
    "test/kernel/freertos/.project/project.js",
    "test/kernel/freertos_smp/.project/project.js",
    "test/kernel/test_smp_kernel/.project/project.js",
    "test/kernel/rov/.project/project.js",
    "test/networking/mbedtls_test/test_aes/.project/project.js",
    "test/networking/mbedtls_test/test_ssl_x509/.project/project.js",
    "test/networking/mbedtls_test/test_sha_entropy/.project/project.js",
    "test/networking/mbedtls_test/test_pka/.project/project.js",
    "test/networking/mbedtls_benchmark/.project/project.js",
    "test/networking/enet_unittest/testcase_singlepkt/.project/project.js",
    "test/networking/enet_layer2_icssg/icssg_layer2_switch/.project/project.js",
    "test/networking/enet_layer2_icssg/icssg_layer2_dualmac/.project/project.js",
    "test/networking/enet_layer2_cpsw/.project/project.js",
    "test/networking/lwip/enet_cpsw_tcpserver/.project/project.js",
    "test/networking/lwip/enet_icssg_tcpserver/.project/project.js",
    "test/networking/lwip/enet_multicore_lwip/.project/project.js",
    "test/networking/lwip/enet_intercore_icssg/.project/project.js",
    "test/networking/performance_benchmarks/ethernet_cpsw/layer2_performance/.project/project.js",    "test/syscfg/syscfg_combo_1/.project/project.js",
    "test/security/crypto/test_sa2ul_sha/.project/project.js",
    "test/security/crypto/test_sa2ul_aes/.project/project.js",
    "test/security/crypto/test_sa2ul_rng/.project/project.js",
    "test/security/crypto/test_sa2ul_pka/test_rsa_encryption_decryption/.project/project.js",
    "test/security/crypto/test_sa2ul_pka/test_rsa_signing_verification/.project/project.js",
    "test/security/crypto/test_sa2ul_pka/test_ecdsa_signing_verification/.project/project.js",
];

function getProjectSpecCpu(cpu) {
    let projectSpecCpu =
    {
        "r5fss0-0": "MAIN_PULSAR_Cortex_R5_0_0",
        "r5fss0-1": "MAIN_PULSAR_Cortex_R5_0_1",
        "r5fss1-0": "MAIN_PULSAR_Cortex_R5_1_0",
        "r5fss1-1": "MAIN_PULSAR_Cortex_R5_1_1",
        "m4fss0-0": "Cortex_M4F_0",
        "a53ss0-0": "CortexA53_0",
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
    return "AM64x";
}

function getProjectSpecDevice(board) {
    return "AM64x";
}

function getSysCfgCpu(cpu) {
    return cpu;
}

function getSysCfgPkg(board) {
    return "ALV";
}

function getSysCfgPart(board) {
    return "Default";
}

function getDevToolTirex(board) {
    switch (board) {
        case "am64x-sk":
            return "AM64x_SK_EVM";
        default:
        case "am64x-evm":
            return "AM64x_GP_EVM";
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
            return "am64-main-r5f0_0-fw";
        case "r5fss0-1":
            return "am64-main-r5f0_1-fw";
        case "r5fss1-0":
            return "am64-main-r5f1_0-fw";
        case "r5fss1-1":
            return "am64-main-r5f1_1-fw";
        case "m4fss0-0":
            return "am64-mcu-m4f0_0-fw";
    }
    return undefined;
}

function getProductNameProjectSpec() {
    return "MCU_PLUS_SDK_AM64X";
}

function getFlashAddr() {
    return 0x60000000;
}

function getEnableGccBuild() {
    const IsGccBuildEnabled = 0;
    return IsGccBuildEnabled;
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
    getEnableGccBuild,
};
