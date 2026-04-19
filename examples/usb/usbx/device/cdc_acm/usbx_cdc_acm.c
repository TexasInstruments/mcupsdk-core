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
#include <fx_api.h>
#include <ux_api.h>
#include <ux_system.h>

#include <tx_api.h>
#include <ux_api.h>
#include <ux_system.h>
#include <ux_utility.h>
#include <ux_device_stack.h>
#include <ux_device_class_cdc_acm.h>

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


/* Definitions and objects for the CDC demonstration threads. */
#define UX_DEMO_STACK_SIZE (8192U)

static UCHAR cdc_acm_demo_stack[UX_DEMO_STACK_SIZE];
static TX_THREAD cdc_acm_demo_thread;
void cdc_acm_demo_thread_entry(ULONG arg);

/* CDC callback functions declarations. */
static void tx_demo_cdc_instance_activate(void *cdc_instance);
static void tx_demo_cdc_instance_deactivate(void *cdc_instance);

/* Device descriptor from usb_descriptor.c */
extern unsigned char cdc_acm_device_framework_full_speed[];
extern unsigned char cdc_acm_device_framework_high_speed[];
extern unsigned char cdc_acm_string_framework[];
extern unsigned char cdc_acm_language_id_framework[];

extern const size_t cdc_acm_device_framework_full_speed_sz;
extern const size_t cdc_acm_device_framework_high_speed_sz;
extern const size_t cdc_acm_string_framework_sz;
extern const size_t cdc_acm_language_id_framework_sz;

/* CDC ACM instance pointer and parameters. */
static UX_SLAVE_CLASS_CDC_ACM *cdc_acm_slave = UX_NULL;
static UX_SLAVE_CLASS_CDC_ACM_PARAMETER parameter;

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
     * Start by initializing USBX so it can be ready to respond to USB Setup request as soon
     * as possible in case the device is already connected.
     */

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


    /* Register the CDC instance activate and deactivate callbacks. */
    parameter.ux_slave_class_cdc_acm_instance_activate = tx_demo_cdc_instance_activate;
    parameter.ux_slave_class_cdc_acm_instance_deactivate = tx_demo_cdc_instance_deactivate;

    /* Register the CDC Class. */
    res = _ux_device_stack_class_register(_ux_system_slave_class_cdc_acm_name, _ux_device_class_cdc_acm_entry,
                                           1,0,  &parameter);

    if(res != UX_SUCCESS) {
        DebugP_log("Error initializing USB cdc class with status %u\r\n", res);
        DebugP_assert(tx_status == UX_SUCCESS);
    }

    /* Create the CDC demonstration thread. */
    res = tx_thread_create(&cdc_acm_demo_thread, "CDC demo", cdc_acm_demo_thread_entry, 0,
        cdc_acm_demo_stack, UX_DEMO_STACK_SIZE, 20, 20, 1, TX_AUTO_START);

    if(res != TX_SUCCESS) {
        DebugP_log("Error creating thread with status %u.\r\n", tx_status);
        DebugP_assert(tx_status == TX_SUCCESS);
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


void cdc_acm_demo_thread_entry(ULONG arg)
{
    ULONG actual_length;
    CHAR test_buffer[128];
    UINT i = 0;

    while(1)
    {

        tx_thread_sleep(1000u);

        /* Ensure the CDC class is mounted.  */
        if(cdc_acm_slave != UX_NULL)
        {
            DebugP_log(1, "CDC ACM instance activated.\r\n");

            while(cdc_acm_slave != NULL) {
                tx_thread_sleep(1000u);

                sprintf(test_buffer, "Test ACM output %u\r\n", i++);

                ux_device_class_cdc_acm_write(cdc_acm_slave, (UCHAR *)test_buffer, strlen(test_buffer), &actual_length);
            }

            DebugP_log(1, "CDC ACM instance de-activated.\r\n");
        }
    }
}


static void tx_demo_cdc_instance_activate(void *cdc_instance)
{
    /* Save the CDC instance.  */
    cdc_acm_slave = (UX_SLAVE_CLASS_CDC_ACM *) cdc_instance;
}

static void tx_demo_cdc_instance_deactivate(void *cdc_instance)
{
    /* Reset the CDC instance.  */
    cdc_acm_slave = UX_NULL;
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
