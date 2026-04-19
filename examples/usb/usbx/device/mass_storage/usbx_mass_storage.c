/*
 *  Copyright (C) 2018-2026 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include <tx_api.h>
#include <fx_api.h>
#include <ux_api.h>
#include <ux_system.h>
#include <ux_utility.h>
#include <ux_device_stack.h>
#include "ux_device_class_storage.h"

#include <usb/cdn/include/usb_init.h>
#include <usb/cdn/include/cdn_print.h>

#include "ux_dcd_am243x.h"

#define MESSAGE  "Hello world!"

/* Definitions and objects for the driver threads. */
#define CDN_DRV_TASK_PRI  (18)
#define CDN_DRV_STACK_SIZE (8192U)

#define UX_DCD_TASK_PRI  (19)
#define UX_DCD_STACK_SIZE (8192U)

static TX_THREAD cdn_drv_thread;
static TX_THREAD ux_dcd_thread;

static UCHAR cdn_drv_stack[CDN_DRV_STACK_SIZE];
static UCHAR ux_dcd_stack[CDN_DRV_STACK_SIZE];

void cdn_drv_thread_entry(ULONG arg);
void ux_dcd_thread_entry(ULONG arg);


/* MSC callback functions declarations. */
UINT demo_media_read(VOID *storage, ULONG lun, UCHAR * data_pointer, ULONG number_blocks, ULONG lba, ULONG *media_status);
UINT demo_media_write(VOID *storage, ULONG lun, UCHAR * data_pointer, ULONG number_blocks, ULONG lba, ULONG *media_status);
UINT demo_media_status(VOID *storage, ULONG lun, ULONG media_id, ULONG *media_status);

/* Device descriptors from usb_descriptor.c */
extern unsigned char cdc_acm_device_framework_full_speed[];
extern unsigned char cdc_acm_device_framework_high_speed[];
extern unsigned char cdc_acm_string_framework[];
extern unsigned char cdc_acm_language_id_framework[];

extern const size_t cdc_acm_device_framework_full_speed_sz;
extern const size_t cdc_acm_device_framework_high_speed_sz;
extern const size_t cdc_acm_string_framework_sz;
extern const size_t cdc_acm_language_id_framework_sz;

/* MSC instance pointer and parameters. */
UX_SLAVE_CLASS_STORAGE_PARAMETER storage_parameter;

/* RAMDISK objects and declarations. */
#define RAMDKISK_SIZE (8u*1024u*1024u)
#define RAM_DISK_LAST_LBA ((RAMDKISK_SIZE / 512u) - 1u)
uint8_t ramdisk_memory[RAMDKISK_SIZE];
UCHAR buffer[512u];
FX_MEDIA ram_disk;
VOID _fx_ram_driver(FX_MEDIA *media_ptr);

///////////////////////////////
/*
 * USB
 */
#define USB_CLK_SEL_HFOSC0_CLKOUT   (0x00U)
#define USB_CLK_SEL_HFOSC1_CLKOUT   (0x01U)
/* USB init Parameters */
static usb_init_param_t gUsbInitParam = {
    0,                          /*< for AM64x/AM243x only instance 0 is available */
    USB_CLK_SEL_HFOSC0_CLKOUT,  /*< select HFOSC0_CLKOUT for USB clock source */
    6,                          /*< for AM64x/AM243x EVM set PHY ref clock selection to 6 for 25Mhz */
    1,                          /*< enable USB 2.0 only (disable USB 3.0) */
    0,                          /*< no invert polarity */
    1,                          /*< VBUS_SEL for external divider */
    0                           /*< set to device mode */
};

////////////////////////////

/* Memory for the USBX Device instance. */
uint8_t usb_mem[128*1024];

/* Dedicated dma memory for the USBX Device instance. */
uint8_t usb_dma_mem[128*1024] __attribute__((aligned(8), section(".bss.nocache")));

void usbx_main(ULONG args)
{
    int32_t res;
    UINT tx_status;

    Drivers_open();

    res = Board_driversOpen();
    DebugP_assert(res == SystemP_SUCCESS);

    DebugP_log("Application start\r\n");

    DebugP_log("Initializing USBX\r\n");

    /*
     * Start by initializing USBX and FILEX so it can be ready to respond to USB Setup request as soon
     * as possible in case the device is already connected.
     */

    /* Initialize FileX and the RAMDISK.  */
    fx_system_initialize();

    ux_utility_memory_set(ramdisk_memory, 0, sizeof(ramdisk_memory));

    tx_status = fx_media_format(&ram_disk, _fx_ram_driver, ramdisk_memory, buffer, 512, "RAM DISK", 2, 512, 0, sizeof(ramdisk_memory)/512, 512, 1, 1, 1);
    if(tx_status != FX_SUCCESS) {
        DebugP_log("Error initializing RAMDISK with status %u\r\n", tx_status);
        DebugP_assert(tx_status == FX_SUCCESS);
    }

    tx_status = fx_media_open(&ram_disk, "RAM DISK", _fx_ram_driver, ramdisk_memory, buffer, 512);
    if(tx_status != FX_SUCCESS) {
        DebugP_log("Error opening RAMDISK with status %u\r\n", tx_status);
        DebugP_assert(tx_status == FX_SUCCESS);
    }

    /* Initialize USBX system. */
    tx_status = ux_system_initialize(usb_mem, sizeof(usb_mem), usb_dma_mem, sizeof(usb_dma_mem));
    if(tx_status != UX_SUCCESS) {
        DebugP_log("Error initializing USB system with status %u\r\n", tx_status);
        DebugP_assert(tx_status == UX_SUCCESS);
    }

    /* Initialize the USBX device stack with our device descriptors. */
    res = _ux_device_stack_initialize(cdc_acm_device_framework_high_speed, cdc_acm_device_framework_high_speed_sz,
                                      cdc_acm_device_framework_full_speed, cdc_acm_device_framework_full_speed_sz,
                                      cdc_acm_string_framework, cdc_acm_string_framework_sz,
                                      cdc_acm_language_id_framework, cdc_acm_language_id_framework_sz, NULL);

    if(res != UX_SUCCESS) {
        DebugP_log("Error initializing USB device stack with status %u\r\n", res);
        DebugP_assert(tx_status == UX_SUCCESS);
    }


    /* Initializes the MSC LUN.  */
    storage_parameter.ux_slave_class_storage_parameter_number_lun = 1u;
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_last_lba = RAM_DISK_LAST_LBA;
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_block_length = 512u;
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_type = 0;
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_removable_flag  = 0x80;
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_read = demo_media_read;
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_write = demo_media_write;
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_status = demo_media_status;
    storage_parameter.ux_slave_class_storage_parameter_vendor_id = (UCHAR *)"TI USBX MSC Device";
    storage_parameter.ux_slave_class_storage_parameter_product_id = (UCHAR *)"USBX MSC Device";
    storage_parameter.ux_slave_class_storage_parameter_product_rev = (UCHAR *)"0";
    storage_parameter.ux_slave_class_storage_parameter_product_serial = (UCHAR *)"0";

    /* Register the MSC Class. */
    res = ux_device_stack_class_register(_ux_system_slave_class_storage_name, ux_device_class_storage_entry,
                                                1, 0, (VOID *)&storage_parameter);

    if(res != UX_SUCCESS) {
        DebugP_log("Error initializing USB MSC class with status %u\r\n", res);
        DebugP_assert(tx_status == UX_SUCCESS);
    }

    /* The USB Software stack is initialized, now initialize the hardware and driver. */

    DebugP_log("Initializing USB hardware.\r\n");

    /* initialize USB HW for TI SOC */
    usb_init(&gUsbInitParam);

    DebugP_log("Initialization complete, launching USB threads. \r\n");

    tx_thread_sleep(10u);

    tx_status = tx_thread_create(&cdn_drv_thread, "Cadence USB Device Driver Thread", cdn_drv_thread_entry, 0,
                           cdn_drv_stack, CDN_DRV_STACK_SIZE, CDN_DRV_TASK_PRI, CDN_DRV_TASK_PRI, 0, TX_AUTO_START);
    if(tx_status != TX_SUCCESS) {
        DebugP_log("Error creating thread with status %u.\r\n", tx_status);
        DebugP_assert(tx_status == TX_SUCCESS);
    }

    tx_status = tx_thread_create(&ux_dcd_thread, "USBX Controller Driver Thread", ux_dcd_thread_entry, 0,
                           ux_dcd_stack, UX_DCD_STACK_SIZE, UX_DCD_TASK_PRI, UX_DCD_TASK_PRI, 0, TX_AUTO_START);
    if(tx_status != TX_SUCCESS) {
        DebugP_log("Error creating thread with status %u.\r\n", tx_status);
        DebugP_assert(tx_status == TX_SUCCESS);
    }

    /* Loop forever while the USB example is running. */
    for(;;) {
        tx_thread_sleep(1000u);
    }

    Board_driversClose();
    Drivers_close();
}

UINT demo_media_status(VOID *storage, ULONG lun, ULONG media_id, ULONG *media_status)
{
    *media_status = 0u;

    return(UX_SUCCESS);
}

UINT demo_media_read(VOID *storage, ULONG lun, UCHAR * data_pointer, ULONG number_blocks, ULONG lba, ULONG *media_status)
{
    UINT status = 0;

    ram_disk.fx_media_driver_logical_sector = lba;
    ram_disk.fx_media_driver_sectors = number_blocks;
    ram_disk.fx_media_driver_request = FX_DRIVER_READ;
    ram_disk.fx_media_driver_buffer = data_pointer;
    _fx_ram_driver(&ram_disk);

    *media_status = 0u;

    status = ram_disk.fx_media_driver_status;

    return(status);
}

UINT demo_media_write(VOID *storage, ULONG lun, UCHAR * data_pointer, ULONG number_blocks, ULONG lba, ULONG *media_status)
{
    UINT status = 0;

    ram_disk.fx_media_driver_logical_sector = lba;
    ram_disk.fx_media_driver_sectors = number_blocks;
    ram_disk.fx_media_driver_request = FX_DRIVER_WRITE;
    ram_disk.fx_media_driver_buffer = data_pointer;
    _fx_ram_driver(&ram_disk);

    *media_status = 0u;

    status = ram_disk.fx_media_driver_status;

    return(status);
}

void cdn_drv_thread_entry(ULONG arg)
{
    while(1) {
        cusbd_dsr();
    }
}

void ux_dcd_thread_entry(ULONG arg)
{
    while(1) {
        _ux_dcd_am243x_interrupt_thread(0u);
    }
}
