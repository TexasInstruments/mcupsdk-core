#  Migration Guides {#MIGRATION_GUIDES}

[TOC]

Some TI SOCs offer the Processor SDK RTOS package for developing application on the RTOS/NORTOS CPUs.
If you are a user of Processor SDK RTOS, the references listed on this page point to information that will assist you in
migration to MCU+ SDK.

-# Change list between Processor SDK RTOS and MCU+ SDK, see this \htmllink{../mcu_plus_sdk_and_processor_sdk_rtos_change_list.xlsx, XLS}
-# OS Kernel
   -# FreeRTOS comparison with SysBIOS, see \ref KERNEL_FREERTOS_PAGE
   -# \subpage OSAL_MIGRATION_GUIDE
-# SOC Peripheral Drivers
   -# \subpage GPIO_MIGRATION_GUIDE
   -# \subpage I2C_MIGRATION_GUIDE
   -# \subpage IPC_MIGRATION_GUIDE
   -# \subpage PINMUX_MIGRATION_GUIDE
   -# \subpage UART_MIGRATION_GUIDE
   -# \subpage UDMA_MIGRATION_GUIDE