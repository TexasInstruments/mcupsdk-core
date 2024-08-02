const common = require("../common.js");

const component_file_list = [
    "source/board/.project/project.js",
    "source/calibration/.project/project.js",
    "source/drivers/.project/project.js",
    "source/fs/freertos_fat/.project/project.js",
    "source/fs/littlefs/.project/project.js",
    "source/middleware/.project/project.js",
    "source/kernel/nortos/.project/project.js",
    "source/kernel/freertos/.project/project.js",
    "source/mathlib/.project/project.js",
    "source/networking/enet/.project/project_cpsw.js",
    "source/networking/enet/.project/project_cpsw_lwipif_freertos.js",
    "source/networking/enet/.project/project_cpsw_lwipif_nortos.js",
    "source/networking/icss_timesync/.project/project.js",
    "source/networking/lwip/.project/project_stack_freertos.js",
    "source/networking/lwip/.project/project_stack_freertos_icss_emac.js",
    "source/networking/lwip/.project/project_stack_nortos.js",
    "source/networking/lwip/.project/project_contrib_freertos.js",
    "source/networking/lwip/.project/project_contrib_freertos_icss_emac.js",
    "source/networking/lwip/.project/project_contrib_nortos.js",
	"source/networking/icss_emac/.project/project.js",
    "source/networking/icss_emac/lwipif/.project/project.js",
    "source/networking/mbedtls_library/.project/project.js",
    "source/networking/tsn/.project/project_tsn_gptp.js",
    "source/networking/tsn/.project/project_tsn_combase.js",
    "source/networking/tsn/.project/project_tsn_uniconf.js",
    "source/networking/tsn/.project/project_tsn_unibase.js",
    "source/sdl/.project/project.js",
    "source/security/.project/project.js",
    "test/unity/.project/project.js",
    "docs_src/docs/api_guide/doxy_samples/.project/project.js",
];

const device_defines = {
    common: [
        "SOC_AM263PX",
    ],
};

const example_file_list = [
    "examples/benchmarks/coremark_benchmark/.project/project.js",
    "examples/benchmarks/dhrystone_benchmark/.project/project.js",
    "examples/benchmarks/ocmc_benchmarking/.project/project.js",
    "examples/drivers/adc/adc_safety_checker/.project/project.js",
    "examples/drivers/adc/adc_trigger_repeater_oversampling/.project/project.js",
    "examples/drivers/adc/adc_trigger_repeater_undersampling/.project/project.js",
	"examples/drivers/adc/adc_burst_mode_epwm/.project/project.js",
	"examples/drivers/adc/adc_burst_mode_oversampling/.project/project.js",
	"examples/drivers/adc/adc_differential_mode/.project/project.js",
	"examples/drivers/adc/adc_early_interrupt_offset/.project/project.js",
	"examples/drivers/adc/adc_high_priority_soc/.project/project.js",
	"examples/drivers/adc/adc_multiple_soc_epwm/.project/project.js",
	"examples/drivers/adc/adc_open_shorts_detection/.project/project.js",
	"examples/drivers/adc/adc_ppb_delay/.project/project.js",
	"examples/drivers/adc/adc_ppb_epwm_trip/.project/project.js",
	"examples/drivers/adc/adc_ppb_limits/.project/project.js",
	"examples/drivers/adc/adc_ppb_offset/.project/project.js",
	"examples/drivers/adc/adc_soc_continuous/.project/project.js",
	"examples/drivers/adc/adc_soc_continuous_dma/.project/project.js",
	"examples/drivers/adc/adc_alternate_dma_trigger/.project/project.js",
	"examples/drivers/adc/adc_soc_epwm/.project/project.js",
	"examples/drivers/adc/adc_soc_ecap/.project/project.js",
	"examples/drivers/adc/adc_soc_rti/.project/project.js",
	"examples/drivers/adc/adc_global_force/.project/project.js",
	"examples/drivers/adc/adc_dac_loopback/.project/project.js",
	"examples/drivers/adc/adc_soc_oversampling/.project/project.js",
	"examples/drivers/adc/adc_soc_software/.project/project.js",
	"examples/drivers/adc/adc_soc_software_sync/.project/project.js",
	"examples/drivers/adc/adc_sw_interleaved_averaging/.project/project.js",
	"examples/drivers/boot/sbl_can/.project/project.js",
    "examples/drivers/boot/sbl_can_multicore_elf/.project/project.js",
    "examples/drivers/boot/sbl_can_uniflash/.project/project.js",
    "examples/drivers/boot/sbl_can_uniflash_multicore_elf/.project/project.js",
	"examples/drivers/boot/sbl_null/.project/project.js",
    "examples/drivers/boot/sbl_jtag_uniflash/.project/project.js",
	"examples/drivers/boot/sbl_ospi/.project/project.js",
    "examples/drivers/boot/sbl_ospi_fastboot/.project/project.js",
    "examples/drivers/boot/sbl_ospi_multicore_elf/.project/project.js",
    "examples/drivers/boot/sbl_ospi_swap/.project/project.js",
    "examples/drivers/boot/sbl_sd/.project/project.js",
    "examples/drivers/boot/sbl_sd_multicore_elf/.project/project.js",
	"examples/drivers/boot/sbl_uart/.project/project.js",
    "examples/drivers/boot/sbl_uart_multicore_elf/.project/project.js",
    "examples/drivers/boot/sbl_uart_uniflash/.project/project.js",
    "examples/drivers/boot/sbl_uart_uniflash_multicore_elf/.project/project.js",
	"examples/drivers/cmpss/cmpss_asynchronous_trip/.project/project.js",
	"examples/drivers/cmpss/cmpss_digital_filter_trip/.project/project.js",
	"examples/drivers/dac/dac_constant_voltage/.project/project.js",
	"examples/drivers/dac/dac_epwm_sync/.project/project.js",
	"examples/drivers/dac/dac_ramp_wave/.project/project.js",
	"examples/drivers/dac/dac_random_voltage/.project/project.js",
	"examples/drivers/dac/dac_sine_dma/.project/project.js",
	"examples/drivers/dac/dac_sine_wave/.project/project.js",
	"examples/drivers/dac/dac_square_wave/.project/project.js",
	"examples/drivers/ecap/ecap_apwm_mode/.project/project.js",
	"examples/drivers/ecap/ecap_capture_pwm/.project/project.js",
	"examples/drivers/ecap/ecap_signal_monitoring/.project/project.js",
	"examples/drivers/ecap/ecap_edma/.project/project.js",
	"examples/drivers/edma/edma_chain_transfer/.project/project.js",
	"examples/drivers/edma/edma_interrupt_transfer/.project/project.js",
	"examples/drivers/edma/edma_link_transfer/.project/project.js",
    "examples/drivers/edma/edma_multimem_transfer/.project/project.js",
	"examples/drivers/edma/edma_polled_transfer/.project/project.js",
	"examples/drivers/epwm/epwm_chopper/.project/project.js",
	"examples/drivers/epwm/epwm_configure_signal/.project/project.js",
	"examples/drivers/epwm/epwm_deadband/.project/project.js",
    "examples/drivers/epwm/epwm_dccap/.project/project.js",
	"examples/drivers/epwm/epwm_dma/.project/project.js",
    "examples/drivers/epwm/epwm_xcmp_dma/.project/project.js",
	"examples/drivers/epwm/epwm_hr_duty_cycle/.project/project.js",
	"examples/drivers/epwm/epwm_hr_updown/.project/project.js",
	"examples/drivers/epwm/epwm_illegal_combo_logic/.project/project.js",
    "examples/drivers/epwm/hrpwm_deadband_sfo/.project/project.js",
    "examples/drivers/epwm/hrpwm_duty_cycle_sfo/.project/project.js",
    "examples/drivers/epwm/hrpwm_phase_shift_sfo/.project/project.js",
	"examples/drivers/epwm/epwm_minimum_deadband/.project/project.js",
	"examples/drivers/epwm/epwm_protection_pru/.project/project.js",
    "examples/drivers/epwm/epwm_synchronization/.project/project.js",
	"examples/drivers/epwm/epwm_trip_zone/.project/project.js",
	"examples/drivers/epwm/epwm_valley_switching/.project/project.js",
    "examples/drivers/epwm/epwm_xcmp_multiple_edges/.project/project.js",
    "examples/drivers/epwm/epwm_diode_emulation/.project/project.js",
    "examples/drivers/epwm/epwm_up_aq/.project/project.js",
	"examples/drivers/eqep/eqep_frequency_measurement/.project/project.js",
	"examples/drivers/eqep/eqep_position_speed/.project/project.js",
    "examples/drivers/flc/.project/project_am263px.js",
    "examples/drivers/fss/swap_to_b/.project/project.js",
	"examples/drivers/fss/fota_over_can/.project/project.js",
    "examples/drivers/fsi/fsi_loopback_dma/.project/project.js",
	"examples/drivers/fsi/fsi_loopback_interrupt/.project/project.js",
	"examples/drivers/fsi/fsi_loopback_polling/.project/project.js",
	"examples/drivers/gpio/gpio_input_interrupt/.project/project.js",
	"examples/drivers/gpio/gpio_led_blink/.project/project.js",
	"examples/drivers/gpio/gpio_multi_led_blink/.project/project.js",
	"examples/drivers/hsmclient/hsm_services/.project/project.js",
    "examples/drivers/hsmclient/mpu_firewall_services_demo/.project/project.js",
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
    "examples/drivers/i2c/i2c_peripheral_transfer/.project/project.js",
	"examples/drivers/ipc/ipc_notify_echo/.project/project.js",
	"examples/drivers/ipc/ipc_rpmsg_echo/.project/project.js",
	"examples/drivers/ipc/ipc_spinlock_sharedmem/.project/project.js",
	"examples/drivers/lin/lin_external/.project/project.js",
	"examples/drivers/lin/lin_loopback_interrupts/.project/project.js",
	"examples/drivers/lin/lin_loopback_polling/.project/project.js",
	"examples/drivers/lin/lin_sci_dma/.project/project.js",
	"examples/drivers/lin/lin_sci_loopback/.project/project.js",
	"examples/drivers/mcan/mcan_external_read_write/.project/project.js",
    "examples/drivers/mcan/mcan_loopback_interrupt/.project/project.js",
    "examples/drivers/mcan/mcan_loopback_polling/.project/project.js",
    "examples/drivers/mcan/canfd_loopback_polling/.project/project.js",
    "examples/drivers/mcan/canfd_loopback_interrupt/.project/project.js",
    "examples/drivers/mcan/canfd_loopback_dma/.project/project.js",
	"examples/drivers/mcspi/mcspi_loopback/.project/project.js",
	"examples/drivers/mcspi/mcspi_loopback_dma/.project/project.js",
    "examples/drivers/mcspi/mcspi_loopback_polling_lld/.project/project.js",
    "examples/drivers/mcspi/mcspi_loopback_interrupt_lld/.project/project.js",
    "examples/drivers/mcspi/mcspi_loopback_dma_lld/.project/project.js",
    "examples/drivers/mcspi/mcspi_loopback_multi_instances_multi_channels_lld/.project/project.js",
	"examples/drivers/mcspi/mcspi_performance_32bit/.project/project.js",
	"examples/drivers/mcspi/mcspi_performance_8bit/.project/project.js",
    "examples/drivers/mcspi/mcspi_loopback_turbo_mode/.project/project.js",
	"examples/drivers/mmcsd/mmcsd_file_io/.project/project.js",
	"examples/drivers/mmcsd/mmcsd_raw_io/.project/project.js",
    "examples/drivers/mmcsd/mmcsd_raw_io_sd_lld/.project/project.js",
    "examples/drivers/mmcsd/mmcsd_raw_io_sd_intr_lld/.project/project.js",
    "test/drivers/mpu_firewall/mpu_firewall_test_am263px/.project/project.js",
	"examples/drivers/ospi/ospi_flash_diag/.project/project.js",
	"examples/drivers/ospi/ospi_flash_dma/.project/project.js",
	"examples/drivers/ospi/ospi_flash_io/.project/project.js",
    "examples/drivers/ospi/ospi_nand_flash_io/.project/project.js",
    "examples/drivers/ospi/ospi_flash_file_io/.project/project.js",
	"examples/drivers/pmu/pmu_multievent/.project/project.js",
    "examples/drivers/rat/.project/project_nortos.js",
	"examples/drivers/resolver/resolver_angle_speed/.project/project.js",
    "examples/drivers/rl2/.project/project_nortos.js",
	"examples/drivers/rti/rti_led_blink/.project/project.js",
	"examples/drivers/sdfm/sdfm_epwm_sync_cpuread/.project/project.js",
	"examples/drivers/sdfm/sdfm_ecap_loop_back/.project/project.js",
	"examples/drivers/sdfm/sdfm_filter_sync_cpuread/.project/project.js",
	"examples/drivers/sdfm/sdfm_filter_sync_cpuread_single_channel/.project/project.js",
	"examples/drivers/sdfm/sdfm_filter_sync_dmaread/.project/project.js",
    "examples/drivers/spinlock/spinlock_example/.project/project_nortos.js",
    "examples/drivers/tmu/tmu_context_save/.project/project.js",
    "examples/drivers/tmu/tmu_cores_support/.project/project.js",
    "examples/drivers/tmu/tmu_operations/.project/project.js",
    "examples/drivers/tmu/tmu_park_transform/.project/project.js",
	"examples/drivers/uart/uart_echo/.project/project.js",
	"examples/drivers/uart/uart_echo_callback/.project/project.js",
	"examples/drivers/uart/uart_echo_dma/.project/project.js",
    "examples/drivers/uart/uart_echo_dma_lld/.project/project.js",
    "examples/drivers/uart/uart_echo_interrupt_lld/.project/project.js",
    "examples/drivers/uart/uart_echo_low_latency_interrupt/.project/project.js",
    "examples/drivers/uart/uart_echo_low_latency_polling/.project/project.js",
    "examples/drivers/uart/uart_echo_polling_lld/.project/project.js",
	"examples/drivers/watchdog/watchdog_interrupt/.project/project.js",
	"examples/drivers/watchdog/watchdog_reset/.project/project.js",
    "examples/empty/.project/project_freertos.js",
    "examples/empty/.project/project_nortos.js",
	"examples/hello_world/.project/project.js",
	"examples/hello_world_cpp/.project/project.js",
    "examples/mathlib/benchmark/.project/project.js",
	"examples/kernel/dpl/dpl_demo/.project/project.js",
	"examples/kernel/dpl/dpl_low_latency_interrupt/.project/project.js",
	"examples/kernel/dpl/interrupt_prioritization/.project/project.js",
    "examples/kernel/dpl/test_data_abort/.project/project.js",
    "examples/kernel/dpl/test_prefetch_abort/.project/project.js",
    "examples/kernel/dpl/test_undef/.project/project.js",
	"examples/kernel/freertos/posix_demo/.project/project.js",
	"examples/kernel/freertos/task_switch/.project/project.js",
    "examples/kernel/nortos/wfi_standby_demo/.project/project.js",
    "examples/kernel/nortos/basic_smart_placement/.project/project_nortos.js",
    "examples/security/crypto/dthe_aes/crypto_aes_cbc_128/.project/project.js",
	"examples/security/crypto/dthe_aes/crypto_aes_cbc_256/.project/project.js",
	"examples/security/crypto/dthe_aes/crypto_aes_cmac_128/.project/project.js",
	"examples/security/crypto/dthe_aes/crypto_aes_cmac_256/.project/project.js",
	"examples/security/crypto/dthe_aes/crypto_aes_ecb_128/.project/project.js",
	"examples/security/crypto/dthe_aes/crypto_aes_ecb_256/.project/project.js",
    "examples/security/crypto/dthe_aes/crypto_aes_stream/.project/project.js",
    "examples/security/crypto/dthe_aes/crypto_aes_ctr/.project/project.js",
    "examples/security/crypto/dthe_aes/crypto_aes_cfb/.project/project.js",
	"examples/security/crypto/dthe_sha/crypto_hmac_sha256/.project/project.js",
	"examples/security/crypto/dthe_sha/crypto_hmac_sha512/.project/project.js",
	"examples/security/crypto/dthe_sha/crypto_sha_256/.project/project.js",
	"examples/security/crypto/dthe_sha/crypto_sha_512/.project/project.js",
    "examples/networking/enet_loopback/enet_cpsw_loopback/loopback_mac_end/.project/project.js",
    "examples/networking/enet_loopback/enet_cpsw_loopback/loopback_phy_end/.project/project.js",
    "examples/networking/enet_cpsw_est/.project/project.js",
    "examples/networking/enet_layer2_cpsw/.project/project.js",
    "examples/networking/enet_layer2_cpsw_switch/.project/project.js",
	"examples/networking/lwip/icss_emac_lwip/.project/project.js",
    "examples/networking/lwip/cpsw_lwip_https/.project/project.js",
    "examples/networking/lwip/enet_cpsw_rawhttpserver/.project/project.js",
    "examples/networking/lwip/enet_cpsw_socket/.project/project.js",
    "examples/networking/lwip/enet_cpsw_tcpclient/.project/project.js",
    "examples/networking/lwip/enet_cpsw_tcpserver/.project/project.js",
    "examples/networking/lwip/enet_cpsw_udpclient/.project/project.js",
    "examples/networking/lwip/enet_lwip_cpsw/.project/project.js",
    "examples/networking/lwip/enet_cpsw_udp_igmp/.project/project.js",
    "examples/networking/tsn/gptp_cpsw_app/.project/project.js",
    "examples/networking/tsn/gptp_lwip_cpsw/.project/project.js",
    "examples/networking/tsn/lldp_cpsw_app/.project/project.js",
    "examples/real_time_debug/.project/project.js",
    "examples/sdl/dcc/dcc_modes1/.project/project.js",
    "examples/sdl/ecc/sdl_ecc_icssm/.project/project.js",
    "examples/sdl/ecc/sdl_ecc_mcan/.project/project.js",
    "examples/sdl/ecc/sdl_ecc_mss_l2/.project/project.js",
    "examples/sdl/ecc/sdl_ecc_mss_tptc/.project/project.js",
    "examples/sdl/ecc/sdl_ecc_r5_atcm0/.project/project.js",
    "examples/sdl/ecc/sdl_ecc_r5_btcm/.project/project.js",
    "examples/sdl/pbist/pbist_mcu/.project/project.js",
    "examples/sdl/rti/UC1/.project/project.js",
    "examples/sdl/rti/UC2/.project/project.js",
    "examples/sdl/rti/UC3/.project/project.js",
    "examples/sdl/rti/UC4/.project/project.js",
    "examples/sdl/ecc_bus_safety/ecc_bus_safety_main/.project/project.js",
    "examples/sdl/ccm/sdl_ccm_example/.project/project.js",
    "examples/sdl/ccm/sdl_ccm_vim/.project/project.js",
    "examples/sdl/ccm/sdl_ccm_tmu_example/.project/project.js",
    "examples/sdl/ccm/sdl_ccm_rl2_example/.project/project.js",
    "examples/sdl/stc/stc_mcu/.project/project.js",
    "examples/sdl/mcrc/mcrc_auto_cpu/.project/project.js",
    "examples/sdl/mcrc/mcrc_full_cpu/.project/project.js",
    "examples/sdl/mcrc/mcrc_semi_cpu/.project/project.js",
    "examples/sdl/parity/sdl_dma_parity/.project/project.js",
    "examples/sdl/parity/sdl_tcm_parity/.project/project.js",
    "examples/sdl/tmu_rom_checksum/.project/project.js",
    "examples/sdl/r5f_cpu_utils/.project/project.js",
    "examples/sdl/parity/sdl_tmu_parity/.project/project.js",
    "examples/sdl/stog/.project/project.js",
    "examples/sdl/vtm/vtm_mcu/.project/project.js",
    "examples/networking/enet_tx_scatter_gather/.project/project.js",
    "test/examples/drivers/ospiPhy/.project/project_nortos.js",
    "test/sdl/pbist/sdl_pbist_test/.project/project.js",
    "test/sdl/pbist/ip_pbist_test/.project/project.js",
    "test/sdl/dcc/dcc_mcu/dcc_func_test/.project/project.js",
    "test/sdl/dcc/dcc_mcu/dcc_unit_test/.project/project.js",
    "test/sdl/mcrc/mcrcUt/.project/project.js",
    "test/sdl/mcrc/mcrcFuncTest/.project/project.js",
	"test/sdl/mcrc/mcrcAutomodeTest/.project/project.js",
	"test/sdl/mcrc/mcrcSemimodeTest/.project/project.js",
    "test/sdl/rti/rtiUt/.project/project.js",
    "test/sdl/rti/rtiFuncTest/.project/project.js",
    "test/sdl/esm/v0/esm_func_test/.project/project.js",
    "test/sdl/esm/v0/esm_unit_test/.project/project.js",
    "test/sdl/ecc/common/.project/project.js",
    "test/sdl/ecc/ecc_sdl_funcTest/.project/project.js",
    "test/sdl/ecc/ecc_sdl_unitTest/.project/project.js",
    "test/sdl/ecc_bus_safety/eccBussafetyFuncTest/.project/project.js",
    "test/sdl/ecc_bus_safety/eccBusSafetyUt/.project/project.js",
    "test/sdl/ccm/common/.project/project.js",
    "test/sdl/ccm/ccm_ip_test/.project/project.js",
    "test/sdl/ccm/ccm_func_test/.project/project.js",
    "test/sdl/stc/stc_mcu/stc_func_test/.project/project.js",
    "test/sdl/parity/sdl_dmaparity_functest/.project/project.js",
    "test/sdl/parity/sdl_tcmparity_functest/.project/project.js",
    "test/sdl/utils/.project/project.js",
    "test/sdl/stog/stog_unit_test/.project/project.js",
    "test/sdl/stog/stog_func_test/.project/project.js",
	"test/board/eeprom/.project/project.js",
	"test/board/flash/.project/project.js",
    "test/board/nand_flash/.project/project.js",
	"test/board/led/.project/project.js",
	"test/drivers/boot/boot_testapp_mb/.project/project.js",
	"test/drivers/adc/.project/project.js",
	"test/drivers/dac/.project/project.js",
	"test/drivers/cmpss/.project/project.js",
	"test/drivers/ecap/.project/project.js",
	"test/drivers/edma/.project/project.js",
	"test/drivers/epwm/.project/project.js",
	"test/drivers/eqep/.project/project.js",
	"test/drivers/fsi/.project/project.js",
	"test/drivers/gpio/.project/project.js",
	"test/drivers/i2c/.project/project.js",
	"test/drivers/ipc_notify/.project/project.js",
	"test/drivers/ipc_notify/ipc_notify_ut/ipc_notify_ut_am263px/.project/project.js",
	"test/drivers/ipc_rpmsg/.project/project.js",
	"test/drivers/ipc_rpmsg/ipc_rpmsg_test_am263px/.project/project.js",
	"test/drivers/lin/.project/project.js",
    "test/drivers/mcan/.project/project.js",
    "test/drivers/mcspi/mcspi/.project/project_am263px.js",
	"test/drivers/mcspi/mcspi_controller_peripheral/.project/project.js",
	"test/drivers/mcspi/mcspi_controller_peripheral_dma/.project/project.js",
    "test/drivers/optiflash/.project/project.js",
	"test/drivers/ospi/.project/project.js",
    "test/benchmarks/memory_access_latency/.project/project.js",
    "test/benchmarks/flsopskd_benchmark/.project/project.js",
	"test/drivers/sdfm/.project/project.js",
	"test/drivers/resolver/.project/project.js",
    "test/drivers/rti/.project/project.js",
	"test/drivers/soc/soc_r5f/.project/project.js",
	"test/drivers/uart/.project/project.js",
    "test/examples/fss_switch_b/.project/project.js",
	"test/kernel/dpl/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/AddrTranslateP_null/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/ClockP_nortos/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/DebugP_log/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/DebugP_memTraceLogWriter/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/DebugP_nortos/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/DebugP_shmLogWriter/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/DebugP_uartLogWriter/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/DebugP_uartScanf/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/EventP_nortos/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/HeapP_nortos/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/Heap_internal/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/QueueP_nortos/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/SemaphoreP_nortos/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/TaskP_nortos/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/common/TimerP_rti/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/r5/BootP_armv7r/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/r5/CacheP_armv7r/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/r5/ClockP_nortos_r5/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/r5/CpuId_armv7r/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/r5/HwiP_armv7r_handlers_nortos/.project/project.js",
	"test/kernel/dpl/dpl_ut/nortos/r5/PmuP_armv7r/.project/project.js",
	"test/kernel/freertos/.project/project.js",
    "test/middleware/tielfup32/.project/project.js",
    "test/networking/performance_benchmarks/ethernet_cpsw/layer2_performance/.project/project.js",
    "test/security/crypto/test_dthe_aes_cbc/.project/project.js",
    "test/security/crypto/test_dthe_aes_cbc_stream/.project/project.js",
    "test/security/crypto/test_dthe_aes_cfb/.project/project.js",
    "test/security/crypto/test_dthe_aes_ctr/.project/project.js",
    "test/security/crypto/test_dthe_aes_ctr_stream/.project/project.js",
    "test/security/crypto/test_dthe_aes_ecb/.project/project.js",
    "test/security/crypto/test_dthe_aes_ecb_stream/.project/project.js",
    "test/security/crypto/test_dthe_aes_cmac/.project/project.js",
    "test/security/crypto/test_dthe_aes_cmac_stream/.project/project.js",
    "test/security/crypto/test_dthe_aes_cmac_stream_unaligned/.project/project.js",
    "test/security/crypto/test_dthe_aes_stream_state_test/.project/project.js",
    "test/security/crypto/test_dthe_sha/.project/project.js",
    "test/security/crypto/test_dthe_hmac_sha/.project/project.js",
    "test/syscfg/syscfg_combo_1/.project/project.js",
    "tools/flasher/jtag_uniflash/.project/project.js",
    "examples/pru_io/empty/.project/project.js",
    "examples/pru_io/empty/firmware/.project/project.js",
];

function getProjectSpecCpu(cpu) {
    let projectSpecCpu =
    {
        "r5fss0-0": "Cortex_R5_0",
        "r5fss0-1": "Cortex_R5_1",
        "r5fss1-0": "Cortex_R5_2",
        "r5fss1-1": "Cortex_R5_3",
        "icss_m0_pru0": "ICSSM_PRU_0",
        "icss_m0_pru1": "ICSSM_PRU_1",
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
        case "am263px-lp":
            return "AM263Px";
        default:
        case "am263px-cc":
            return "AM263Px";
        case "am263px-cc-addon-ind":
            return "AM263Px";
        case "am263px-cc-addon-auto":
            return "AM263Px";
    }
}

function getProjectSpecDevice(board) {
    switch (board) {
        case "am263px":
            return "AM263Px";
        case "am263px-lp":
            return "AM263Px";
        default:
        case "am263px-cc":
            return "AM263Px";
        case "am263px-cc-addon-ind":
            return "AM263Px";
        case "am263px-cc-addon-auto":
            return "AM263Px";
    }
}

function getSysCfgCpu(cpu) {
    return cpu;
}

function getSysCfgPkg(board) {
    switch (board) {
        case "am263px-lp":
            return "ZCZ_C";
        default:
        case "am263px-cc":
            return "ZCZ_S";
        case "am263px-cc-addon-ind":
            return "ZCZ_S";
        case "am263px-cc-addon-auto":
            return "ZCZ_S";
    }
}

function getSysCfgPart(board) {
    switch (board) {
        case "am263px-lp":
            return "AM263P4";
        default:
        case "am263px-cc":
            return "AM263P4";
        case "am263px-cc-addon-ind":
            return "AM263P4";
        case "am263px-cc-addon-auto":
            return "AM263P4";
    }
}

function getDevToolTirex(board) {
    switch (board) {
        case "am263px-lp":
            return "LP-AM263P";
        default:
        case "am263px-cc":
            return "TMDSCNCD263P";
        case "am263px-cc-addon-ind":
            return "TMDSCNCD263P";
        case "am263px-cc-addon-auto":
            return "TMDSCNCD263P";
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
            return "am263p-main-r5f0_0-fw";
        case "r5fss0-1":
            return "am263p-main-r5f0_1-fw";
        case "r5fss1-0":
            return "am263p-main-r5f1_0-fw";
        case "r5fss1-1":
            return "am263p-main-r5f1_1-fw";
    }
    return undefined;
}

function getProductNameProjectSpec() {
    return "MCU_PLUS_SDK_AM263PX";
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
