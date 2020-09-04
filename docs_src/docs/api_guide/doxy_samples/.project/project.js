let path = require('path');

const files = {
    common: [
        "Adc_sample.c",
        "AddrTranslateP_sample.c",
        "Bootloader_sample.c",
        "CacheP_sample.c",
        "CpuIdP_sample.c",
        "ClockP_sample.c",
        "Crc_sample.c",
        "CycleCounterP_sample.c",
        "Ddr_sample.c",
        "DebugP_sample.c",
        "Ecap_sample.c",
        "Eeprom_sample.c",
        "Epwm_sample.c",
        "Eqep_sample.c",
        "Ethphy_sample.c",
        "EventP_sample.c",
        "Flash_sample.c",
        "Fsi_sample.c",
        "Gpio_sample.c",
        "HeapP_sample.c",
        "HwiP_sample.c",
        "I2c_sample.c",
        "Icss_emac_sample.c",
        "Icss_timesync_sample.c",
        "IpcNotify_sample.c",
        "IpcRPMessage_sample.c",
        "Led_sample.c",
        "Mcan_sample.c",
        "Mcspi_sample.c",
        "Mdio_sample.c",
        "MpuP_arm_v7_sample.c",
        "Ospi_sample.c",
        "Pinmux_sample.c",
        "Pruicss_sample_g_v0.c",
        "Sciclient_sample.c",
        "SemaphoreP_sample.c",
        "Spinlock_sample.c",
        "sa2ul_sha_sample.c",
        "sa2ul_aes_cbc_sample.c",
        "Sa2ul_pka_rsa_encrypt_decrypt_sample.c",
        "QueueP_sample.c",
        "TaskP_sample.c",
        "TimerP_sample.c",
        "Uart_sample.c",
        "Udma_sample.c",
        "Pru_ipc_sample.c",
    ],
};

const files_am273x = {
    common: [
        "AddrTranslateP_sample.c",
        "Bootloader_sample_v1.c",
        "CacheP_sample.c",
        "CpuIdP_sample.c",
        "ClockP_sample.c",
        "Crc_sample.c",
        "Csirx_sample.c",
        "CycleCounterP_sample.c",
        "DebugP_sample.c",
        "Edma_sample.c",
        "Esm_sample.c",
        "Gpio_sample_v1.c",
        "EventP_sample.c",
        "HeapP_sample.c",
        "Hwa_sample.c",
        "HwiP_sample.c",
        "IpcNotify_sample.c",
        "IpcRPMessage_sample.c",
        "MpuP_arm_v7_sample.c",
        "Mibspi_sample.c",
        "QueueP_sample.c",
        "SemaphoreP_sample.c",
        "TaskP_sample.c",
        "TimerP_sample.c",
        "Uart_sample.c",
    ],
};

const files_am263x = {
    common: [
        "AddrTranslateP_sample.c",
        "Bootloader_sample_v2.c",
        "CacheP_sample.c",
        "CpuIdP_sample.c",
        "ClockP_sample.c",
        "Cmpss_sample.c",
        "Dac_sample.c",
        "DebugP_sample.c",
        "Edma_sample.c",
        "Ethphy_sample.c",
        "EventP_sample.c",
        "HeapP_sample.c",
        "HwiP_sample.c",
        "Icss_emac_sample.c",
        "IpcNotify_sample.c",
        "IpcRPMessage_sample.c",
        "Mcspi_sample.c",
        "Mdio_sample.c",
        "MpuP_arm_v7_sample.c",
        "Pruicss_sample_m_v0.c",
        "QueueP_sample.c",
        "Sdfm_sample.c",
        "SemaphoreP_sample.c",
        "TaskP_sample.c",
        "TimerP_sample.c",
        "Uart_sample.c",
        "Watchdog_sample.c",
    ],
};

const files_am62x = {
    common: [
        "AddrTranslateP_sample.c",
        "ClockP_sample.c",
        "CycleCounterP_sample.c",
        "DebugP_sample.c",
        "HeapP_sample.c",
        "HwiP_m4_sample.c",
        "QueueP_sample.c",
        "SemaphoreP_sample.c",
        "TaskP_sample.c",
        "IpcRPMessage_linux_sample.c",
        "MpuP_arm_v7_sample.c",
    ],
};


const filedirs = {
    common: [
        "board",
        "drivers",
        "kernel/dpl",
        "networking",
        "pru_io",
        "security/crypto/sa2ul",
    ],
};

const buildOptionCombos = [
    { device: device,  cpu: "r5f", cgt: "ti-arm-clang"},
];

const buildOptionCombos_am62x = [
    { device: device,  cpu: "m4f", cgt: "ti-arm-clang"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "doxy_samples";
    property.isInternal = true;
    property.buildOptionCombos = buildOptionCombos;

    if(buildOption.device=="am62x")
    {
        property.buildOptionCombos = buildOptionCombos_am62x;
    }
    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    if(buildOption.device=="am263x")
    {
        build_property.files = files_am263x;
    }
    if(buildOption.device=="am64x" || buildOption.device=="am243x")
    {
        build_property.files = files;
        build_property.files.common.push("Soc_am64x_sample.c");
    }
    if(buildOption.device=="am273x" || buildOption.device=="awr294x")
    {
        build_property.files = files_am273x;
        build_property.files.common.push("CacheP_c6x_sample.c");
        if(buildOption.device=="awr294x")
        {
            build_property.files.common.push("Adcbuf_sample.c");
            build_property.files.common.push("Mailbox_sample.c");
        }
        if(buildOption.device=="am273x")
        {
            build_property.files.common.push("Mcasp_sample.c");
        }
    }
    if(buildOption.device=="am62x")
    {
        build_property.files = files_am62x;
    }
    build_property.filedirs = filedirs;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
