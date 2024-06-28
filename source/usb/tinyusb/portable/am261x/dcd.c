/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018, hathach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */
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

/* Adapted by TI for running on its platform and SDK */

#include "tusb_option.h"
#include <stdarg.h>
#include <stdio.h>
#include "device/dcd.h"
#include <usb_drv.h>
#include "soc/device_wrapper.h"
#include <kernel/dpl/DebugP.h> 
#include <kernel/nortos/dpl/common/printf.h>

/*
--------------------------------------------------------------------+
 MACRO TYPEDEF CONSTANT ENUM DECLARATION
--------------------------------------------------------------------+
*/

//! \brief The dwc_usb3_device structure in ram
extern  usb_handle_t usb_handle;

/*------------------------------------------------------------------*/
/* Device API
 *------------------------------------------------------------------*/

/* Initialize controller to device mode */
void dcd_init (uint8_t rhport)
{
  TU_LOG2("[dcd_init]\n");

  /* Configure the USB driver */
  usb_handle.dwc_usb3_dev = NULL;
  usb_handle.dwc_usb3_dev = dwc_usb3_driver_init(usb_handle.cfg_base);
  /* Enable USB_OTG_MAIN0_INT interrupt so that enumeration process can start */
  USB_configureInterrupt(USB_OTG_MAIN0_INT);
  return;
}

/* Enable device interrupt */
void dcd_int_enable (uint8_t rhport)
{
  /* Enable USB interrupt */
  HwiP_enableInt(USB20_MAIN0_INT);
  (void) rhport;
}

/* Disable device interrupt */
void dcd_int_disable (uint8_t rhport)
{
  /* Disable USB interrupt */
  HwiP_disableInt(USB20_MAIN0_INT);
  (void) rhport;
}

/* Receive Set Address request, mcu port must also include status IN response */
void dcd_set_address (uint8_t rhport, uint8_t dev_addr)
{
    /* Not needed - peripheral automatically changes address during enumeration */
}

/* Wake up host */
void dcd_remote_wakeup (uint8_t rhport)
{
  TU_LOG2("[dcd_remote_wakeup]\n");
  (void) rhport;
}

/* Connect by enabling internal pull-up resistor on D+/D- */
void dcd_connect(uint8_t rhport)
{
}

/* Disconnect by disabling internal pull-up resistor on D+/D- */
void dcd_disconnect(uint8_t rhport)
{
}

void dcd_sof_enable(uint8_t rhport, bool en)
{

}
/*
--------------------------------------------------------------------+
 Endpoint API
--------------------------------------------------------------------+
*/
/* Configure endpoint's registers according to descriptor */
bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const * ep_desc)
{
    usb_ep_t *ep;
    TU_LOG2("[dcd_edpt_open] ep_addr=%d\n", ep_desc->bEndpointAddress);

    /* Open the specific endpoint according to descriptor */
    ep = dwc_usb3_ep_enable(usb_handle.dwc_usb3_dev, ep_desc, NULL);
    if (ep == NULL)
    {
        return false;
    }

    return true;
}

void dcd_edpt_close_all (uint8_t rhport)
{
  (void) rhport;
  /* TODO implement dcd_edpt_close_all() */
}

void epXferCmplCb(usb_ep_t *ep, usb_request_t *req) {
    uint8_t dir   = tu_edpt_dir(ep->address);

    TU_LOG2("[epXferCmplCb] EP:%02X req(0x%08X) buf(0x%08X) status(%d) length(%d) actual(%d)\n",
            ep->address, (uintptr_t) req, (uintptr_t)req->buf, req->status, req->length, req->actual);

    if (dir == TUSB_DIR_IN) {
        /* inform the tud_task the transfer is completed */
        dcd_event_t event = { .rhport = 0, .event_id = DCD_EVENT_XFER_COMPLETE };
        event.xfer_complete.ep_addr = ep->address;
        event.xfer_complete.len     = req->actual;
        event.xfer_complete.result  = req->status;
        dcd_event_handler(&event, true);
        req->complete = 0;
    }
    else {
        /* copy the received data from completed request buffer to TinyUSB buffer */
        TU_LOG2("[epXferCmplCb] memcpy: from (0x%08X) to (0x%08X) (%d)bytes \n",
            (uintptr_t)req->dma, (uintptr_t)req->buf, req->actual);
        memcpy((void *)req->buf, (void *)req->dma, req->actual);

        /* inform the tud_task the transfer is completed */
        dcd_event_t event = { .rhport = 0, .event_id = DCD_EVENT_XFER_COMPLETE };
        event.xfer_complete.ep_addr = ep->address;
        event.xfer_complete.len     = req->actual;
        event.xfer_complete.result  = req->status;
        dcd_event_handler(&event, true);
        req->complete = 0;
    }
    dwc_usb3_free_request(usb_handle.dwc_usb3_dev, ep, req);
}

void inEp0XferCmplCb(usb_ep_t *ep, usb_request_t *req) {

    TU_LOG2("[inEpXferCmplCb] EP:%02X req(0x%08X) buf(0x%08X) status(%d) length(%d) actual(%d)\n",
            0x80, (uintptr_t) req, (uintptr_t)req->buf, req->status, req->length, req->actual);
    /* inform the tud_task the transfer is completed */
    dcd_event_t event = { .rhport = 0, .event_id = DCD_EVENT_XFER_COMPLETE };
    event.xfer_complete.ep_addr = 0x80;
    event.xfer_complete.len     = req->actual;
    event.xfer_complete.result  = req->status;
    dcd_event_handler(&event, true);
    req->complete = 0;
}

void outEp0XferCmplCb(usb_ep_t *ep, usb_request_t *req) {

    TU_LOG2("[outEp0XferCmplCb] EP:%02X req(0x%08X) buf(0x%08X) status(%d) length(%d) actual(%d)\n",
            ep->address, (uintptr_t) req, (uintptr_t)req->buf, req->status, req->length, req->actual);

    TU_LOG2("[outEp0XferCmplCb] memcpy: from (0x%08X) to (0x%08X) redIdx(%d) (%d)bytes \n",
        (uintptr_t)req->dma, (uintptr_t)req->buf, 0x80, req->actual);
    /* copy the received data from completed request buffer to TinyUSB buffer */
    memcpy((void *)req->buf, (void *)req->dma, req->actual);
    /* inform the tud_task the transfer is completed */
    dcd_event_t event = { .rhport = 0, .event_id = DCD_EVENT_XFER_COMPLETE };
    event.xfer_complete.ep_addr = 0;
    event.xfer_complete.len     = req->actual;
    event.xfer_complete.result  = req->status;
    dcd_event_handler(&event, true);
    req->complete = 0;
}

/* Submit a transfer, When complete dcd_event_xfer_complete() is invoked to notify the stack */
bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
    uint8_t epnum = tu_edpt_number(ep_addr);
    uint8_t dir   = tu_edpt_dir(ep_addr);
    dwc_usb3_pcd_t *pcd = &usb_handle.dwc_usb3_dev->pcd;
    dwc_usb3_pcd_ep_t  *pcd_ep = dwc_usb3_pcd_get_ep_by_addr(pcd, ep_addr);
    TU_LOG2("[dcd_edpt_xfer] ep:%02x \n", ep_addr);

    if (epnum == 0){
        if (dir == TUSB_DIR_IN)
        {
            char *local_buf = (char *)pcd->ep0_status_buf;
            memcpy(local_buf, buffer, total_bytes);
            pcd_ep->dwc_ep.is_in = 1;
            if(total_bytes == 0){
                pcd->ep0state = EP0_IN_WAIT_NRDY;   // the host via setup_in_status_phase()
                                                   // the response can be seen on sniffer
                dcd_event_t event = { .rhport = 0, .event_id = DCD_EVENT_XFER_COMPLETE };
                event.xfer_complete.ep_addr = ep_addr;
                event.xfer_complete.len     = total_bytes;
                event.xfer_complete.result  = 0;
                dcd_event_handler(&event, true);
            }
            else {
                pcd->ep0state = EP0_IN_DATA_PHASE;
                pcd->ep0_req->dwc_req.length = total_bytes;
                pcd->ep0_status_pending = 1;
                pcd->ep0_req->dwc_req.buf[0] = (char *)pcd->ep0_status_buf;
                pcd->ep0_req->dwc_req.bufdma[0] = pcd->ep0_status_buf_dma;
                pcd->ep0_req->dwc_req.actual = 0;
                pcd->ep0_req->usb_req.complete = inEp0XferCmplCb;
                dwc_usb3_pcd_ep0_start_transfer(pcd, pcd->ep0_req);
            }
        }
        else
        {
            pcd_ep->dwc_ep.is_in = 0;
            if(total_bytes == 0){
                       // this two lines trigger a response to
                pcd->ep0state = EP0_OUT_WAIT_NRDY;   // the host via setup_in_status_phase()
                                                   // the response can be seen on sniffer
                dcd_event_t event = { .rhport = 0, .event_id = DCD_EVENT_XFER_COMPLETE };
                event.xfer_complete.ep_addr = ep_addr;
                event.xfer_complete.len     = total_bytes;
                event.xfer_complete.result  = 0;
                dcd_event_handler(&event, true);
            }
            else
            {
                pcd->ep0state = EP0_OUT_DATA_PHASE;
                pcd->ep0_req->dwc_req.length = total_bytes;
                pcd->ep0_status_pending = 1;
                pcd->ep0->dwc_ep.send_zlp = 0;
                pcd->ep0_req->dwc_req.buf[0] = (char *)pcd->ep0_status_buf_dma;
                pcd->ep0_req->dwc_req.bufdma[0] = pcd->ep0_status_buf_dma;
                pcd->ep0_req->dwc_req.actual = 0;
                pcd->ep0_req->usb_req.buf = buffer;
                pcd->ep0_req->usb_req.dma = pcd->ep0_status_buf_dma;
                pcd->ep0_req->usb_req.complete = outEp0XferCmplCb;
                dwc_usb3_pcd_ep0_start_transfer(pcd, pcd->ep0_req);
            }
        }
    }
    else
    {
        usb_request_t * usb_req = dwc_usb3_alloc_request(usb_handle.dwc_usb3_dev, &pcd_ep->usb_ep);
        usb_req->length = total_bytes;
        usb_req->actual = 0;
        usb_req->complete = epXferCmplCb;
        if (dir == TUSB_DIR_IN)
        {
            char *local_buf = (char *)ep_in_buf[epnum-1];
            memcpy(local_buf, buffer, total_bytes);
            usb_req->buf = (char *)ep_in_buf[epnum-1];
            usb_req->dma = (dwc_dma_t) ep_in_buf[epnum-1];
            dwc_usb3_ep_queue(usb_handle.dwc_usb3_dev, &pcd_ep->usb_ep, usb_req);
        }
        else
        {
            usb_req->buf = (char *) buffer;
            usb_req->dma = (dwc_dma_t) ep_out_buf[epnum-1];
            dwc_usb3_ep_queue(usb_handle.dwc_usb3_dev, &pcd_ep->usb_ep, usb_req);
        }
    }

    return true;
}

/* Stall endpoint */
void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr)
{
    TU_LOG2("[dcd_edpt_stall] ep_addr=%d\n", ep_addr);
    dwc_usb3_pcd_t *pcd = &usb_handle.dwc_usb3_dev->pcd;
    dwc_usb3_pcd_ep_t *ep = dwc_usb3_pcd_get_ep_by_addr(pcd, ep_addr);
    dwc_usb3_pcd_ep_set_halt(pcd, ep, 1);
}

/* clear stall, data toggle is also reset to DATA0 */
void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr)
{
    TU_LOG2("[dcd_edpt_clear_stall] ep_addr=%d\n", ep_addr);
    dwc_usb3_pcd_t *pcd = &usb_handle.dwc_usb3_dev->pcd;
    dwc_usb3_pcd_ep_t *ep = dwc_usb3_pcd_get_ep_by_addr(pcd, ep_addr);
    dwc_usb3_pcd_ep_set_halt(pcd, ep, 0);
}


#ifdef CFG_TUSB_DEBUG 
int CFG_TUSB_DEBUG_PRINTF(const char *format, ...)
{
	va_list va; 
	va_start(va,format); 
	printf_("[TUSB MSG]");
	vprintf_(format,va);
	va_end(va);
	return 0 ; 
}
#endif 