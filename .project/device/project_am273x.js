const common = require("../common.js");

const component_file_list = [
    "source/board/.project/project.js",
    "source/drivers/.project/project.js",
    "source/middleware/.project/project.js",
    "source/kernel/freertos/.project/project.js",
    "source/kernel/nortos/.project/project.js",
    "source/mathlib/.project/project.js",
    "source/networking/enet/.project/project_cpsw.js",
    "source/networking/enet/.project/project_cpsw_lwipif_freertos.js",
    "source/networking/enet/.project/project_cpsw_lwipif_nortos.js",
    "source/networking/lwip/.project/project_stack_freertos.js",
    "source/networking/lwip/.project/project_stack_nortos.js",
    "source/networking/lwip/.project/project_contrib_freertos.js",
    "source/networking/lwip/.project/project_contrib_nortos.js",
    "source/networking/mbedtls_library/.project/project.js",
    "source/sdl/.project/project.js",
    "source/security/.project/project.js",
    "test/unity/.project/project.js",
    "docs_src/docs/api_guide/doxy_samples/.project/project.js",
    "source/networking/tsn/.project/project_tsn_unibase.js",
    "source/networking/tsn/.project/project_tsn_combase.js",
    "source/networking/tsn/.project/project_tsn_gptp.js",
    "source/networking/tsn/.project/project_tsn_uniconf.js",
 ];

const device_defines = {
    common: [
        "SOC_AM273X",
    ],
};

const example_file_list = [
    "examples/benchmarks/coremark_benchmark/.project/project.js",
    "examples/benchmarks/dhrystone_benchmark/.project/project.js",
    "examples/drivers/boot/sbl_can_uniflash/.project/project.js",
    "examples/drivers/boot/sbl_jtag_uniflash/.project/project.js",
    "examples/drivers/boot/sbl_null/.project/project.js",
    "examples/drivers/boot/sbl_qspi/.project/project.js",
    "examples/drivers/boot/sbl_uart/.project/project.js",
    "examples/drivers/boot/sbl_uart_uniflash/.project/project.js",
    "examples/drivers/cbuff/cbuff_sw_trigger/.project/project.js",
    "examples/drivers/crc/crc_16bit/.project/project.js",
    "examples/drivers/csirx/csirx_internal_capture/.project/project.js",
    "examples/drivers/dsplib/add32/.project/project.js",
    "examples/drivers/ecap/ecap_epwm_loopback/.project/project.js",
    "examples/drivers/ecap/ecap_epwm_loopback_dsp/.project/project.js",
    "examples/drivers/edma/edma_chain_transfer/.project/project.js",
    "examples/drivers/edma/edma_interrupt_transfer/.project/project.js",
    "examples/drivers/edma/edma_link_transfer/.project/project.js",
    "examples/drivers/edma/edma_multimem_transfer/.project/project.js",
    "examples/drivers/edma/edma_polled_transfer/.project/project.js",
	"examples/drivers/epwm/epwm_duty_cycle/.project/project.js",
    "examples/drivers/gpadc/gpadc_group_channel_read/.project/project.js",
    "examples/drivers/gpadc/gpadc_single_channel_read/.project/project.js",
    "examples/drivers/gpadc/gpadc_temperature_sensor_read/.project/project.js",
    "examples/drivers/gpio/gpio_input_interrupt/.project/project.js",
    "examples/drivers/gpio/gpio_led_blink/.project/project.js",
    "examples/drivers/hsmclient/hsm_services/.project/project.js",
    "examples/drivers/hwa/hwa_real_fft/.project/project.js",
    "examples/drivers/hwa/hwa_bfp_compression/.project/project.js",
    "examples/drivers/i2c/i2c_read/.project/project.js",
    "examples/drivers/i2c/i2c_memory_read/.project/project.js",
    "examples/drivers/i2c/i2c_temperature/.project/project.js",
    "examples/drivers/i2c/i2c_memory_read_polling_lld/.project/project.js",
    "examples/drivers/i2c/i2c_memory_read_interrupt_lld/.project/project.js",
    "examples/drivers/i2c/i2c_temperature_polling_lld/.project/project.js",
    "examples/drivers/i2c/i2c_temperature_interrupt_lld/.project/project.js",
    "examples/drivers/ipc/ipc_notify_echo/.project/project.js",
    "examples/drivers/ipc/ipc_rpmsg_echo/.project/project.js",
    "examples/drivers/ipc/ipc_safeipc_echo/.project/project.js",
    "examples/drivers/mcan/mcan_external_loopback_interrupt/.project/project.js",
    "examples/drivers/mcan/mcan_loopback_interrupt/.project/project.js",
    "examples/drivers/mcan/mcan_loopback_polling/.project/project.js",
    "examples/drivers/mcan/canfd_loopback_polling/.project/project.js",
    "examples/drivers/mcan/canfd_loopback_interrupt/.project/project.js",
    "examples/drivers/mcan/canfd_loopback_dma/.project/project.js",
    "examples/drivers/mcasp/mcasp_loopback/.project/project.js",
    "examples/drivers/mcasp/mcasp_playback/.project/project.js",
    "examples/drivers/mibspi/mibspi_loopback/.project/project.js",
    "examples/drivers/mibspi/mibspi_loopback_icount/.project/project.js",
    "examples/drivers/mibspi/mibspi_loopback_dma/.project/project.js",
    "examples/drivers/pmu/pmu_multievent/.project/project.js",
    "examples/drivers/qspi/qspi_flash_dma_transfer/.project/project.js",
    "examples/drivers/qspi/qspi_flash_diag/.project/project.js",
    "examples/drivers/qspi/qspi_flash_transfer/.project/project.js",
    "examples/drivers/qspi/qspi_flash_interrupt/.project/project.js",
    "examples/drivers/qspi/qspi_flash_dma_lld/.project/project.js",
    "examples/drivers/qspi/qspi_flash_diag_lld/.project/project.js",
    "examples/drivers/qspi/qspi_flash_interrupt_lld/.project/project.js",
    "examples/drivers/rti/rti_led_blink/.project/project.js",
    "examples/drivers/uart/uart_echo/.project/project.js",
	"examples/drivers/uart/uart_echo_callback/.project/project.js",
    "examples/drivers/uart/uart_echo_dma/.project/project.js",
    "examples/drivers/watchdog/watchdog_interrupt/.project/project.js",
    "examples/drivers/watchdog/watchdog_reset/.project/project.js",
    "examples/empty/.project/project_freertos.js",
    "examples/empty/.project/project_nortos.js",
    "examples/hello_world/.project/project.js",
    "examples/hello_world_cpp/.project/project.js",
    "examples/kernel/dpl/dpl_demo/.project/project.js",
    "examples/kernel/freertos/posix_demo/.project/project.js",
    "examples/kernel/freertos/interrupt_profiling/.project/project.js",
    "examples/kernel/freertos/task_switch/.project/project.js",
    "examples/mathlib/benchmark/.project/project.js",
    "examples/networking/enet_loopback/enet_cpsw_loopback/loopback_mac_end/.project/project.js",
    "examples/networking/enet_loopback/enet_cpsw_loopback/loopback_phy_end/.project/project.js",
    "examples/networking/tsn/gptp_cpsw_app/.project/project.js",
    "examples/networking/tsn/lldp_cpsw_app/.project/project.js",
     "examples/networking/lwip/cpsw_lwip_https/.project/project.js",
    "examples/networking/lwip/cpsw_lwip_mqtt/.project/project.js",
    "examples/networking/lwip/enet_lwip_cpsw/.project/project.js",
    "examples/networking/lwip/enet_cpsw_tcpserver/.project/project.js",
    "examples/networking/lwip/enet_cpsw_rawhttpserver/.project/project.js",
    "examples/networking/lwip/enet_cpsw_udpclient/.project/project.js",
    "examples/networking/lwip/enet_cpsw_udp_igmp/.project/project.js",
    "examples/networking/lwip/enet_cpsw_tcpclient/.project/project.js",
    "examples/networking/lwip/enet_cpsw_socket/.project/project.js",
	"examples/networking/enet_layer2_multi_channel/.project/project.js",
    "examples/networking/enet_cpsw_est/.project/project.js",
    "examples/networking/enet_tx_scatter_gather/.project/project.js",
    "examples/sdl/hwa/hwa_main/.project/project.js",
    "examples/sdl/rti/UC1/.project/project.js",
    "examples/sdl/rti/UC2/.project/project.js",
    "examples/sdl/rti/UC3/.project/project.js",
    "examples/sdl/rti/UC4/.project/project.js",
    "examples/sdl/mcrc/mcrc_auto_cpu/.project/project.js",
    "examples/sdl/mcrc/mcrc_full_cpu/.project/project.js",
    "examples/sdl/mcrc/mcrc_semi_cpu/.project/project.js",
    "examples/sdl/ecc/sdl_ecc_r5_atcm0/.project/project.js",
    "examples/sdl/ecc/sdl_ecc_mss_l2/.project/project.js",
	"examples/sdl/ecc/sdl_ecc_mcan/.project/project.js",
	"examples/sdl/ecc/sdl_ecc_r5_btcm/.project/project.js",
	"examples/sdl/ecc/sdl_ecc_mss_tptc/.project/project.js",
	"examples/sdl/ecc/sdl_ecc_dss_tptc/.project/project.js",
    "examples/sdl/ecc/sdl_dss_ecc_hwa/.project/project.js",
    "examples/sdl/ecc/sdl_dss_l3_ecc/.project/project.js",
    "examples/sdl/ecc/sdl_dss_mailbox/.project/project.js",
	"examples/sdl/parity/sdl_tcm_parity/.project/project.js",
	"examples/sdl/pbist/pbist_mcu/.project/project.js",
	"examples/sdl/parity/sdl_dss_l2_parity/.project/project.js",
    "examples/sdl/stc/stc_mcu/.project/project.js",
    "examples/sdl/reset/reset_mcu/.project/project.js",
    "examples/sdl/ecc_bus_safety/ecc_bus_safety_main/.project/project.js",
    "examples/sdl/ccm/sdl_ccm_example/.project/project.js",
	"examples/sdl/parity/sdl_dma_parity/.project/project.js",
	"examples/sdl/ecc/sdl_dss_dsp_edc/.project/project.js",
	"examples/sdl/ecc/sdl_dss_l2_edc/.project/project.js",
	"examples/sdl/parity/sdl_dss_l1_parity/.project/project.js",
	"examples/sdl/ccm/sdl_ccm_vim/.project/project.js",
    "test/sdl/ecc_bus_safety/eccBussafetyFuncTest/.project/project.js",
    "test/sdl/ecc_bus_safety/eccBusSafetyUt/.project/project.js",
    "test/sdl/hwa/hwaFuncTest/.project/project.js",
    "test/sdl/hwa/hwaUt/.project/project.js",
    "test/sdl/rti/rtiUt/.project/project.js",
    "test/sdl/rti/rtiFuncTest/.project/project.js",
    "test/sdl/mcrc/mcrcUt/.project/project.js",
    "test/sdl/mcrc/mcrcFuncTest/.project/project.js",
	"test/sdl/mcrc/mcrcAutomodeTest/.project/project.js",
	"test/sdl/mcrc/mcrcSemimodeTest/.project/project.js",
    "test/sdl/esm/v1/esm_unit_test/.project/project.js",
    "test/sdl/ecc/common/.project/project.js",
    "test/sdl/ecc/ecc_sdl_funcTest/.project/project.js",
    "test/sdl/ecc/ecc_sdl_unitTest/.project/project.js",
    "test/sdl/pbist/sdl_pbist_test/.project/project.js",
    "test/sdl/pbist/ip_pbist_test/.project/project.js",
	"test/sdl/ccm/common/.project/project.js",
    "test/sdl/ccm/ccm_ip_test/.project/project.js",
    "test/sdl/ccm/ccm_func_test/.project/project.js",
    "test/sdl/stc/stc_mcu/stc_func_test/.project/project.js",
    "test/sdl/reset/reset_mcu/reset_func_test/.project/project.js",
    "test/sdl/reset/reset_mcu/reset_unit_test/.project/project.js",
	"test/sdl/parity/sdl_tcmparity_functest/.project/project.js",
	"test/sdl/parity/sdl_dmaparity_functest/.project/project.js",
    "examples/security/crypto/dthe_aes/crypto_aes_cbc_128/.project/project.js",
    "examples/security/crypto/dthe_aes/crypto_aes_cbc_256/.project/project.js",
    "examples/security/crypto/dthe_aes/crypto_aes_ecb_128/.project/project.js",
    "examples/security/crypto/dthe_aes/crypto_aes_ecb_256/.project/project.js",
    "examples/security/crypto/dthe_aes/crypto_aes_cmac_128/.project/project.js",
    "examples/security/crypto/dthe_aes/crypto_aes_cmac_256/.project/project.js",
    "examples/security/crypto/dthe_aes/crypto_aes_ctr/.project/project.js",
    "examples/security/crypto/dthe_aes/crypto_aes_stream/.project/project.js",
    "examples/security/crypto/dthe_aes/crypto_aes_cfb/.project/project.js",
    "examples/security/crypto/dthe_sha/crypto_sha_256/.project/project.js",
    "examples/security/crypto/dthe_sha/crypto_sha_512/.project/project.js",
    "examples/security/crypto/dthe_sha/crypto_hmac_sha256/.project/project.js",
    "examples/security/crypto/dthe_sha/crypto_hmac_sha512/.project/project.js",
    "test/board/eeprom/.project/project.js",
    "test/board/led/.project/project.js",
    "test/board/flash/.project/project.js",
    "test/drivers/boot/boot_testapp_mb/.project/project.js",
    "test/drivers/qspi/.project/project.js",
	"test/drivers/crc/.project/project.js",
    "test/drivers/csirx/.project/project.js",
	"test/drivers/ecap/.project/project.js",
    "test/drivers/edma/.project/project.js",
	"test/drivers/edma/edma_ut_am273x/.project/project.js",
	"test/drivers/epwm/.project/project.js",
    "test/drivers/esm/.project/project.js",
    "test/drivers/gpio/.project/project.js",
    "test/drivers/hwa/.project/project.js",
    "test/drivers/i2c/.project/project.js",
    "test/drivers/ipc_notify/.project/project.js",
    "test/drivers/ipc_notify/ipc_notify_ut/ipc_notify_ut_am273x/.project/project.js",
    "test/drivers/ipc_rpmsg/.project/project.js",
    "test/drivers/ipc_rpmsg/ipc_rpmsg_test_am273x/.project/project.js",
    "test/drivers/ipc_rpmsg/ipc_safeipc_test/.project/project.js",
    "test/drivers/mcan/.project/project.js",
    "test/drivers/mcasp/mcasp_multiinst_loopback/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/c66/ClockP_nortos_c66/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/c66/CycleCounterP_c66/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/c66/HwiP_c66_handlers_nortos/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/c66/HwiP_c66/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/c66/CacheP_c66/.project/project.js",
    "test/drivers/mibspi/mibspi_performance_16bit/.project/project.js",
    "test/drivers/mibspi/mibspi_peripheral/.project/project.js",
    "test/drivers/mibspi/mibspi_analog_loopback/.project/project.js",
    "test/drivers/mpu_firewall/mpu_firewall_test_am273x/.project/project.js",
    "test/drivers/soc/soc_r5f/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/AddrTranslateP_null/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/ClockP_nortos/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/DebugP_nortos/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/DebugP_shmLogWriter/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/DebugP_uartLogWriter/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/DebugP_uartScanf/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/Heap_internal/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/HeapP_nortos/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/QueueP_nortos/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/SemaphoreP_nortos/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/TaskP_nortos/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/TimerP_rti/.project/project.js",
    "test/kernel/dpl/dpl_ut/nortos/common/DebugP_memTraceLogWriter/.project/project.js",
    "test/kernel/dpl/dpl_ut/nortos/common/DebugP_log/.project/project.js",
    "test/kernel/dpl/dpl_ut/nortos/common/DebugP_shmLogReader/.project/project.js",
    "test/drivers/uart/.project/project.js",
	"test/kernel/dpl/.project/project.js",
    "test/kernel/freertos/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/r5/ClockP_nortos_r5/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/r5/CpuId_armv7r/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/r5/BootP_armv7r/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/r5/CacheP_armv7r/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/EventP_nortos/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/r5/HwiP_armv7r_handlers_nortos/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/r5/PmuP_armv7r/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/r5/HwiP_armv7r_vim/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/r5/MpuP_armv7r/.project/project.js",
    "test/kernel/rov/.project/project.js",
    "test/security/crypto/test_dthe_aes_cbc/.project/project.js",
    "test/security/crypto/test_dthe_aes_cbc_stream/.project/project.js",
    "test/security/crypto/test_dthe_aes_cfb/.project/project.js",
    "test/security/crypto/test_dthe_aes_ctr/.project/project.js",
    "test/security/crypto/test_dthe_aes_ctr_stream/.project/project.js",
    "test/security/crypto/test_dthe_aes_ecb/.project/project.js",
    "test/security/crypto/test_dthe_aes_ecb_stream/.project/project.js",
    "test/security/crypto/test_dthe_aes_cmac_stream/.project/project.js",
    "test/security/crypto/test_dthe_aes_cmac_stream_unaligned/.project/project.js",
    "test/security/crypto/test_dthe_aes_stream_state_test/.project/project.js",
    "test/security/crypto/test_dthe_sha/.project/project.js",
    "test/networking/mbedtls_test/test_aes/.project/project.js",
    "test/networking/mbedtls_test/test_ssl_x509/.project/project.js",
    "test/networking/mbedtls_test/test_sha_entropy/.project/project.js",
    "test/networking/mbedtls_test/test_pka/.project/project.js",
    "test/networking/mbedtls_benchmark/.project/project.js",
    "test/networking/cpsw_intervlan/.project/project.js",
    "test/networking/enet_unittest/testcase_singlepkt/.project/project.js",
    "test/networking/performance_benchmarks/ethernet_cpsw/layer2_performance/.project/project.js",
    "tools/flasher/jtag_uniflash/.project/project.js",
];

function getProjectSpecCpu(cpu) {
    let projectSpecCpu =
    {
        "r5fss0-0": "Cortex_R5_0",
        "r5fss0-1": "Cortex_R5_1",
        "c66ss0"  : "C66xx_DSP",
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
    return "AM273x";
}

function getProjectSpecDevice(board) {
    return "AM273x.evmAM273x";
}

function getSysCfgCpu(cpu) {
    return cpu;
}

function getSysCfgPkg(board) {
    return "ZCE";
}

function getSysCfgPart(board) {
    return "Default";
}

function getDevToolTirex(board) {
    return "AM273x_EVM";
}

function getProperty() {
    let property = {};

    property.defines = device_defines;

    return property;
}

function getLinuxFwName(cpu) {
    return undefined;
}

function getProductNameProjectSpec() {
    return "MCU_PLUS_SDK_AM273X";
}

function getFlashAddr() {
    return 0xC6000000;
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
