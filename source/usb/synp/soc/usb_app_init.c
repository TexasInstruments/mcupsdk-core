/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

/** @file
 */

#include <string.h>
#include "usb_drv.h"

#define ALIGN_NO_CACHE(x) __attribute__ ((aligned (x))) __attribute__((section(".usb_dfu_nc_data")))

/**
 * The simulation environment doesn't have a malloc() function, so all data
 * structures are statically allocated
 */

/** Driver context struct */
dwc_usb3_device_t                g_usb3_dev __attribute__((section(".usbdfuCxtRam")));

/** @{ */
/** Endpoint context structs */
dwc_usb3_pcd_ep_t                g_ep0 __attribute__((section(".usbdfuCxtRam")));
dwc_usb3_pcd_ep_t                g_out_ep[DWC_MAX_EPS - 1U] __attribute__((section(".usbdfuCxtRam")));
dwc_usb3_pcd_ep_t                g_in_ep[DWC_MAX_EPS - 1U] __attribute__((section(".usbdfuCxtRam")));
/** @} */
/** EP0 PCD request */
dwc_usb3_pcd_req_t               g_ep0_req __attribute__((section(".usbdfuCxtRam")));
/** @{ */
/** PCD request pool */
dwc_usb3_pcd_req_t               g_pcd_req[32] __attribute__((section(".usbdfuCxtRam")));
uint32_t                         g_pcd_req_bm __attribute__((section(".usbdfuCxtRam")));
/** @} */

/** Driver options struct, default values are defined here */
const dwc_usb3_core_params_t usb3ss_module_params = {
        .burst = 1,
        .newcore = 0,
        .phy = 2,
        .wakeup = 0,
        .pwrctl = 0,
        .lpmctl = 0,
        .phyctl = 0,
        .usb2mode = 2,
        .hibernate = 0,
        .hiberdisc = 0,
        .clkgatingen = 0,
        .ssdisquirk = 0,
        .nobos = 0,
        .loop = 0,
        .nump = 4,
        .newcsr = 0,
        .rxfsz = 0,
        .txfsz = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        .txfsz_cnt = 0,
        .baseline_besl = 0,
        .deep_besl = 0,
        .besl = 0,
};

/*** The following data structures must be in coherent DMA capable memory.
 *** All memory in the simulation environment is such, and physical address
 *** == virtual address, so we can just allocate these statically.
 ***/

/** Event buffer */
uint32_t g_event_buf[DWC_EVENT_BUF_SIZE] ALIGN_NO_CACHE(64);

/** EP0 SETUP packet buffer */
char  g_ep0_setup_pkt[8] ALIGN_NO_CACHE(64);

/** Data packet buffer used to return data for GET_STATUS and
 *  GET_DESCRIPTOR requests, up to 512 bytes in length
 */
uint8_t    g_ep0_status_buf[DWC_STATUS_BUF_SIZE] ALIGN_NO_CACHE(64);

uint8_t    ep_in_buf[DWC_MAX_EPS - 1U][DWC_MAX_PACKET_SIZE] ALIGN_NO_CACHE(64);
uint8_t    ep_out_buf[DWC_MAX_EPS - 1U][DWC_MAX_PACKET_SIZE] ALIGN_NO_CACHE(64);

/** DMA descriptor (TRB) for EP0 SETUP packets */
UALIGNED16 dwc_usb3_dma_desc_t   g_ep0_setup_desc ALIGN_NO_CACHE(64);

/** DMA descriptor (TRB) for EP0 Data Out and Status Out phases */
UALIGNED16 dwc_usb3_dma_desc_t   g_ep0_out_desc ALIGN_NO_CACHE(64);;

/** DMA descriptor (TRB) for EP0 Data In and Status In phases */
UALIGNED16 dwc_usb3_dma_desc_t   g_ep0_in_desc ALIGN_NO_CACHE(64);

/** @{ */
/** DMA descriptor (TRB) pools for non-EP0 EPs */
UALIGNED16 dwc_usb3_dma_desc_t  g_out_trb_pool[DWC_MAX_EPS - 1U][DWC_NUM_ISOC_TRBS + 1] ALIGN_NO_CACHE(64);

UALIGNED16 dwc_usb3_dma_desc_t  g_in_trb_pool[DWC_MAX_EPS - 1U][DWC_NUM_ISOC_TRBS + 1] ALIGN_NO_CACHE(64);
/** @} */

/** @{ */
/** Scratchpad buffers for hibernation support */
char                             g_hiber_scratchpad[15][64] ALIGN_NO_CACHE(64);
struct dwc_hiber_scratchpad_array g_hiber_scratchpad_array ALIGN_NO_CACHE(64);
/** @} */


/*
 * Hook to override the default Phy configuration in dwc_usb3_pcd_device_init()
 * with a HAPS-specific one
 */
static void haps_phy_config_hook(struct dwc_usb3_device *dev, int soft_reset,
                                 int restore)
{
        dwc_usb3_core_global_regs_t __iomem *global_regs =
                                                dev->core_global_regs;
        u32 temp;

        switch (dev->core_params->phy) {
        case 3:         // 16-bit UTMI+ SNPS Phy
                temp = dwc_rd32(dev, &global_regs->gusb2phycfg[0]);
                temp &= ~DWC_USB2PHYCFG_USB_TRD_TIM_BITS;
                temp |= DWC_USB2PHYCFG_16B_PHY_IF_BIT;
                temp |= 5U << DWC_USB2PHYCFG_USB_TRD_TIM_SHIFT;
                dwc_wr32(dev, &global_regs->gusb2phycfg[0], temp);
                break;
        case 2:         // 8-bit UTMI+ / ULPI TI or SNPS Phy
        case 1:         // old 8-bit UTMI+ SNPS Phy
                temp = dwc_rd32(dev, &global_regs->gusb2phycfg[0]);
                temp &= ~DWC_USB2PHYCFG_USB_TRD_TIM_BITS;
                temp &= ~DWC_USB2PHYCFG_16B_PHY_IF_BIT;
                temp |= 9U << DWC_USB2PHYCFG_USB_TRD_TIM_SHIFT;
                dwc_wr32(dev, &global_regs->gusb2phycfg[0], temp);
                break;
        default:        // RocketIO Phy
                if (dev->core_params->usb2mode == 0) {
                        /* Set rx-eq, differential swing */
                        dwc_wr32(dev, (volatile u32 __iomem *)
                                 (dev->base + dev->gasket_ofs + 8), 0x41);
#ifdef LECROY
                        /* Rx-detect for LeCroy */
                        dwc_wr32(dev, (volatile u32 __iomem *)
                                 (dev->base + dev->gasket_ofs + 4), 0x200);
#else
                        dwc_wr32(dev, (volatile u32 __iomem *)
                                 (dev->base + dev->gasket_ofs + 4), 0);
#endif
                }
                break;
        }
}

/**
 * This routine is the top level interrupt handler for the Common
 * (Core and Device) interrupts
 */
void dwc_usb3_common_irq(int irq, void *dev)
{
        dwc_usb3_device_t *usb3_dev = dev;

        dwc_acquire_spinlock(usb3_dev, &usb3_dev->pcd.lock);
        dwc_usb3_irq(usb3_dev, irq);
        dwc_release_spinlock(usb3_dev, &usb3_dev->pcd.lock);
}

/**
 * This routine de-initializes the driver
 */
void dwc_usb3_driver_remove(void)
{
        dwc_usb3_device_t *usb3_dev = &g_usb3_dev;
        u32 *event_buf;
        dwc_dma_t event_buf_dma __attribute__((unused));

        dwc_debug0(usb3_dev, "usb3ss_driver_remove()\n");

        if (usb3_dev->cmn_irq_installed > 0U) {
                usb3_dev->cmn_irq_installed = 0;
        }

        if (usb3_dev->pcd_initialized > 0U) {
                usb3_dev->pcd_initialized = 0;
                dwc_usb3_pcd_remove(usb3_dev);
        }

        if (usb3_dev->gadget_initialized > 0U) {
                usb3_dev->gadget_initialized = 0;
                //dwc_usb3_function_remove(usb3_dev);
                //dwc_usb3_gadget_remove(usb3_dev);
        }

        if (usb3_dev->cmn_initialized > 0U) {
                usb3_dev->cmn_initialized = 0;
                dwc_usb3_pcd_common_remove(usb3_dev);
        }

        event_buf = usb3_dev->event_buf[0];
        event_buf_dma = usb3_dev->event_buf_dma[0];
        if (event_buf != NULL) {
                dwc_usb3_dis_flush_eventbuf_intr(usb3_dev, 0);
                usb3_dev->event_buf[0] = NULL;
        }

        if (usb3_dev->sysfs_initialized > 0U) {
                usb3_dev->sysfs_initialized = 0;
        }


        return;
}

/**
 * This routine initializes the driver
 *
 * This routine must be called in a context that allows <strong>dwc_msleep()
 * </strong> to be used, because that function is called while waiting for the
 * core to come out of reset.
 */
dwc_usb3_device_t *dwc_usb3_driver_init(u32 base_addr_dwc)
{
        dwc_usb3_device_t *usb3_dev = &g_usb3_dev;
        u32 addr_ofs = 0xc000, i;
        int retval;

        dwc_debug0(usb3_dev, "usb3ss_driver_init()\n");

        memset(usb3_dev, 0, sizeof(*usb3_dev));
        dwc_init_spinlock(usb3_dev, &usb3_dev->pcd.lock);

        usb3_dev->base = (volatile u8 __iomem *)(long)base_addr_dwc;

        /*
         * Checks that this device is really a DWC_usb3 controller. Also saves
         * the SNPSID register value in usb3_dev->snpsid for later use by the
         * PCD.
         */
        retval = dwc_usb3_pcd_check_snpsid(usb3_dev, addr_ofs);
        if (retval != 0) {
                dwc_error0(&dev->dev, "bad value for SNPSID!\n");
                goto fail;
        }

        if ((usb3ss_module_params.newcore > 0) && usb3_dev->snpsid < 0x5533109aU) {
                usb3_dev->snpsid = 0x5533109aU;
        }

        /*
         * Up to 32 Event Buffers are supported by the hardware, but we only
         * use 1
         */
        usb3_dev->event_buf[0] = g_event_buf;
        usb3_dev->event_buf_dma[0] = (dwc_dma_t)g_event_buf;

        /*
         * "allocate" all the data structures that must be in DMA memory
         */
        usb3_dev->pcd.ep0_setup_desc = &g_ep0_setup_desc;
        usb3_dev->pcd.ep0_setup_desc_dma = (dwc_dma_t)&g_ep0_setup_desc;
        usb3_dev->pcd.ep0_out_desc = &g_ep0_out_desc;
        usb3_dev->pcd.ep0_out_desc_dma = (dwc_dma_t)&g_ep0_out_desc;
        usb3_dev->pcd.ep0_in_desc = &g_ep0_in_desc;
        usb3_dev->pcd.ep0_in_desc_dma = (dwc_dma_t)&g_ep0_in_desc;
        usb3_dev->pcd.ep0_status_buf = g_ep0_status_buf;
        usb3_dev->pcd.ep0_status_buf_dma = (dwc_dma_t)g_ep0_status_buf;
        usb3_dev->pcd.ep0_setup_pkt = (union dwc_setup_pkt *)g_ep0_setup_pkt;
        usb3_dev->pcd.ep0_setup_pkt_dma = (dwc_dma_t)g_ep0_setup_pkt;

        for (i = 0; i < 15U; i++) {
                g_hiber_scratchpad_array.dma_addr[i] = (u64)(long)&g_hiber_scratchpad[i];
                usb3_dev->pcd.hiber_scratchpad[i] = &g_hiber_scratchpad[i];
        }

        usb3_dev->pcd.hiber_scratchpad_array = &g_hiber_scratchpad_array;
        usb3_dev->pcd.hiber_scratchpad_array_dma = (dwc_dma_t)&g_hiber_scratchpad_array;

        /*
         * Now "allocate" the remaining data structures
         */
        memset(&g_ep0_req, 0, sizeof(g_ep0_req));   // added to use with the customized EP0 call back

        usb3_dev->pcd.ep0_req = &g_ep0_req;
        usb3_dev->pcd.ep0 = &g_ep0;
        for (i = 0; i < DWC_MAX_EPS - 1U; i++) {
                usb3_dev->pcd.out_ep[i] = &g_out_ep[i];
                usb3_dev->pcd.in_ep[i] = &g_in_ep[i];
        }

        g_pcd_req_bm = 0xffffffffU;

        /*Initilize event queue*/
        DWC_SIMPLEQ_INIT(&usb3_dev->pcd.event_q);
        /*
         * Add our hook to override the default Phy register setup
         */
        usb3_dev->phy_config_hook = haps_phy_config_hook;

        /*
         * Initialize the DWC_usb3 core
         */
        retval = dwc_usb3_pcd_common_init(usb3_dev, usb3_dev->base + addr_ofs,
                                          &usb3ss_module_params);
        if (retval != 0) {
                dwc_error0(usb3_dev, "CIL initialization failed!\n");
                goto fail;
        }

        usb3_dev->cmn_initialized = 1;

        /*
         * Initialize the Function Driver interface
         */
        retval = dwc_usb3_gadget_init(usb3_dev);
        if (retval != 0) {
                dwc_error0(usb3_dev, "gadget initialization failed!\n");
                goto fail;
        }

        /*
         * Initialize the Function Driver
         */
        //retval = dwc_usb3_function_init(usb3_dev);
        if (retval != 0) {
                dwc_error0(usb3_dev, "loopback gadget initialization failed!\n");
                dwc_usb3_gadget_remove(usb3_dev);
                goto fail;
        }

        usb3_dev->gadget_initialized = 1;

        /*
         * Initialize the PCD
         */
        retval = dwc_usb3_pcd_init(usb3_dev);
        if (retval != 0) {
                dwc_error0(usb3_dev, "PCD initialization failed!\n");
                goto fail;
        }

        usb3_dev->pcd_initialized = 1;

#if 0 // Interrupt handler is hooked up before this routine is called
        // Register interrupt service routine for DWC interrupt
        interrupt_priority_set(USB3SS_DWC_INT, 5);
        interrupt_target_set(USB3SS_DWC_INT, CPU0, 1);
        intRegister(USB3SS_DWC_INT, usb3ss_common_irq, usb3_dev);
        interrupt_enable(USB3SS_DWC_INT, 1);
#endif
        usb3_dev->cmn_irq_installed = 1;

        return usb3_dev;

fail:
        dwc_usb3_driver_remove();

        return NULL;
}



#if defined (CONFIG_SOC_EDISON)
 /* set USB Phy clocks -
  * For EDISON:  set bit 29 in BOOTCFG USB PHY4 
  * */
#define DEVICE_REG_USB_PHYCTL(port,reg)  *((volatile unsigned int *)(0x2620738 + ((port)*0x18) + ((reg)*4)))
void chipUsbEnablePhyRefClk(unsigned int port)
{
    unsigned int reg;

    reg = DEVICE_REG_USB_PHYCTL(port, 4);

    reg = reg | (1<<29); 

    DEVICE_REG_USB_PHYCTL(port, 4) = reg;
}

#endif



