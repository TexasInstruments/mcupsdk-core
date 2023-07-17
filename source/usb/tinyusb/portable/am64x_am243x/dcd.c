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
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

#include "usb_wrapper.h"
#include "device/dcd.h"
#include <kernel/dpl/DebugP.h> 
#include <kernel/nortos/dpl/common/printf.h>

void outEpXferCmplCb(CUSBD_Ep *ep, CUSBD_Req * req);

/*
--------------------------------------------------------------------+
 MACRO TYPEDEF CONSTANT ENUM DECLARATION
--------------------------------------------------------------------+
*/
CUSBD_Ep* getEpFromAddr(CUSBD_Dev *dev, uint8_t epaddr)
{
  LIST_ListHead *list; /* used in for_each loop */

  /* find the right EP matching the epaddr */
  /* for EP0, dev->ep0 is bi-directional */
  if ((dev->ep0->address&0x0F) == (epaddr&0x0F))
    return dev->ep0;

  /* for other EPs */
  for (list = dev->epList.next; list != &dev->epList; list = list->next) {
      CUSBD_Ep *ep = (CUSBD_Ep *) list;
      if (ep->address == epaddr) {
          return ep;
      }
  }

  return NULL;
}

CUSBD_Dev* getDevFromAddr(usb_handle_t *pD, uint8_t devaddr)
{
  return NULL;
}

/*------------------------------------------------------------------*/
/* Device API
 *------------------------------------------------------------------*/

/* Initialize controller to device mode */
void dcd_init (uint8_t rhport)
{
  TU_LOG2("[dcd_init]\n");

  usbDeviceInit(NULL); /* USB SS initialization */

  return;
}

/* Enable device interrupt */
void dcd_int_enable (uint8_t rhport)
{
  /* Enable USB interrupt */
  HwiP_enableInt(usb_handle.hwiParamsUsb.intNum);

  (void) rhport;
}

/* Disable device interrupt */
void dcd_int_disable (uint8_t rhport)
{
  /* Disable USB interrupt */
  HwiP_disableInt(usb_handle.hwiParamsUsb.intNum);

  (void) rhport;
}

/* Receive Set Address request, mcu port must also include status IN response */
void dcd_set_address (uint8_t rhport, uint8_t dev_addr)
{
  CUSBD_PrivateData * dev;
  CH9_UsbState state;

  dev = usb_handle.pD;
  state = dev->device.state;

  TU_LOG2("[dcd_set_address] dev_addr=%d\n", dev_addr);

  /* check if device address is within correct range */
  if (dev_addr > 0x007FU) {
      TU_LOG2("Invalid device address %d\n", dev_addr);
      return;
  }

  /* check if device is in correct state */
  if (state == CH9_USB_STATE_CONFIGURED) {
      TU_LOG2("Trying to set address when configured %c\n", ' ');
  return;
  }

  /* set device address in controller */
  CPS_UncachedWrite32(&dev->reg->USBR_CMD, ((dev_addr << 1) | 0x00000001U));

  if (dev_addr > 0U) {
      dev->device.state = CH9_USB_STATE_ADDRESS;
  } else {
      dev->device.state = CH9_USB_STATE_DEFAULT;
  }

  /* Response with zlp status */
  dcd_edpt_xfer(rhport, 0x80, NULL, 0);

  return;
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
  TU_LOG2("[dcd_connect]\n");
  usb_handle.drv->start(usb_handle.pD);
}

/* Disconnect by disabling internal pull-up resistor on D+/D- */
void dcd_disconnect(uint8_t rhport)
{
  (void) rhport;
  TU_LOG2("[dcd_disconnect]\n");
  usb_handle.drv->stop(usb_handle.pD);
}

/*
--------------------------------------------------------------------+
 Endpoint API
--------------------------------------------------------------------+
*/
/* Configure endpoint's registers according to descriptor */
bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const * ep_desc)
{
  CUSBD_Dev * dev;
  CUSBD_Ep * ep;
  int32_t res = 0;

  TU_LOG2("[dcd_edpt_open] ep_addr=%d\n", ep_desc->bEndpointAddress);

  /* get device handle dev */
  usb_handle.drv->getDevInstance(usb_handle.pD, &dev);

  /* Get EP according to ep_addr */
  ep = (CUSBD_Ep *) getEpFromAddr(dev, ep_desc->bEndpointAddress);

  if (ep!= NULL)
  {
    /* enable this EP accroding to the EP descriptor */
    res = ep->ops->epEnable(usb_handle.pD, ep, (uint8_t *) ep_desc);
    if (res!=0)
    {
      TU_LOG2("[dcd_edpt_open] epEnable error(%d) ep_addr=%d\n", res, ep_desc->bEndpointAddress);
      return false;
    } else
    {
      TU_LOG2("[dcd_edpt_open] epEnable EP%02X sucessfully\n", ep_desc->bEndpointAddress);
    }

    (void) rhport;

    return true;
  } else
  {
    TU_LOG2("[dcd_edpt_open] EP open error|||\n");
    return false;
  }

  return true;
}

void dcd_edpt_close_all (uint8_t rhport)
{
  (void) rhport;
  /* TODO implement dcd_edpt_close_all() */
}

void ctrlInCmplCb(CUSBD_Ep *ep, CUSBD_Req * req)
{
  if (req->status != 0)
  {
    TU_LOG2("[ctrlInCmplCb][WARNING] status(%d) != 0 !!!!\n", (int)(req->status));
    return;
  } else
  {
    TU_LOG2("[ctrlInCmplCb] Transfer complete on ep:%02X, %d(%d) bytes\n", ep->address, req->actual, req->length);
    /* inform the tud_task the transfer is completed */
    dcd_event_t event = { .rhport = 0, .event_id = DCD_EVENT_XFER_COMPLETE };
    event.xfer_complete.ep_addr = ep->address;
    event.xfer_complete.len     = req->actual;
    event.xfer_complete.result  = req->status;
    dcd_event_handler(&event, true);

    return;
  }
}

void inEpXferCmplCb(CUSBD_Ep *ep, CUSBD_Req * req) {

    TU_LOG2("[inEpXferCmplCb] EP:%02X req(0x%08X) buf(0x%08X) status(%d) length(%d) actual(%d)\n",
            ep->address, (uintptr_t) req, (uintptr_t)req->buf, req->status, req->length, req->actual);

    /* inform the tud_task the transfer is completed */
    dcd_event_t event = { .rhport = 0, .event_id = DCD_EVENT_XFER_COMPLETE };
    event.xfer_complete.ep_addr = ep->address;
    event.xfer_complete.len     = req->actual;
    event.xfer_complete.result  = req->status;
    dcd_event_handler(&event, true);
}

void outEp0XferCmplCb(CUSBD_Ep *ep, CUSBD_Req * req) {

    TU_LOG2("[outEp0XferCmplCb] EP:%02X req(0x%08X) buf(0x%08X) status(%d) length(%d) actual(%d)\n",
            ep->address, (uintptr_t) req, (uintptr_t)req->buf, req->status, req->length, req->actual);

    /* copy the received data from completed request buffer to TinyUSB buffer */
    memcpy(req->buf, (uint8_t *)req->dma, req->actual);
    TU_LOG2("[outEp0XferCmplCb] memcpy: from (0x%08X) to (0x%08X) redIdx(%d) (%d)bytes \n",
        (uintptr_t)req->dma, (uintptr_t)req->buf, 0x80, req->actual);

    /* inform the tud_task the transfer is completed */
    dcd_event_t event = { .rhport = 0, .event_id = DCD_EVENT_XFER_COMPLETE };
    event.xfer_complete.ep_addr = (ep->address)&0x0F;
    event.xfer_complete.len     = req->actual;
    event.xfer_complete.result  = req->status;
    dcd_event_handler(&event, true);
}

void outEpXferCmplCb(CUSBD_Ep *ep, CUSBD_Req * req) {

    int reqIdx = usbGetDataXferRequestIndexOut(req);

    TU_LOG2("[outEpXferCmplCb] EP:%02X req(0x%08X) buf(0x%08X) reqIdx(%d) status(%d) length(%d) actual(%d)\n",
            ep->address, (uintptr_t) req, (uintptr_t)req->buf, reqIdx, req->status, req->length, req->actual);
    if (reqIdx == -1) {
        TU_LOG2("[outEpXferCmplCb] Illegal reqIdx(%d)\n", reqIdx);
        return;
    } else {
        /* copy the received data from completed request buffer to TinyUSB expected data buffer */
        memcpy(req->buf, (uint8_t *)req->dma, req->actual);
        TU_LOG2("[outEpXferCmplCb] memcpy: from (0x%08X) to (0x%08X) redIdx(%d) (%d)bytes \n",
            (uintptr_t)req->buf, (uintptr_t)req->dma, reqIdx, req->actual);

        /* inform the tud_task the transfer is completed */
        dcd_event_t event = { .rhport = 0, .event_id = DCD_EVENT_XFER_COMPLETE };
        event.xfer_complete.ep_addr = ep->address;
        event.xfer_complete.len     = req->actual;
        event.xfer_complete.result  = req->status;
        dcd_event_handler(&event, true);
    }

    return;
}

/* Submit a transfer, When complete dcd_event_xfer_complete() is invoked to notify the stack */
bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  CUSBD_Dev * dev;
  CUSBD_Ep * ep;
  int32_t reqIdx = 0;
  int32_t res = 0;

  usb_handle.drv->getDevInstance(usb_handle.pD, &dev);
  /* Get EP according to ep_addr */
  ep = (CUSBD_Ep *) getEpFromAddr(dev, ep_addr);
  if (ep == NULL)
    return false;

  TU_LOG2("[dcd_edpt_xfer] ep:%02X (%08X)\n", ep_addr, (uint32_t)ep);

  /* if it is EP0 OUT */
  if (ep_addr==0)
  {
    /* if it is ZLP then then return */
    if (total_bytes==0)
    {
        /* inform the tud_task the transfer is completed */
        dcd_event_t event = { .rhport = 0, .event_id = DCD_EVENT_XFER_COMPLETE };
        event.xfer_complete.ep_addr = 0;
        event.xfer_complete.len     = 0;
        event.xfer_complete.result  = 0;
        dcd_event_handler(&event, true);
    } else
    {
      /* EP0 OUT data stage */
      /* clear the request */
      memset(&ep0DataXferRequest, 0, sizeof(CUSBD_Req));
      ep0DataXferRequest.length = total_bytes;
      /* save the TinyUSB buffer pointer in the request */
      ep0DataXferRequest.buf = buffer;
      ep0DataXferRequest.dma =  (uintptr_t)buffEp0;
      ep0DataXferRequest.complete = outEp0XferCmplCb;
	  ep0DataXferRequest.deferStatusStage = 1;
      usb_handle.pD->ep0NextState = CH9_EP0_DATA_PHASE;
      usb_handle.pD->ep0DataDirFlag = 0;
      TU_LOG2("[dcd_edpt_xfer] request for ep:%02X, %d bytes, CompCb=%08X\n", ep_addr,
              ep0DataXferRequest.length, (uint32_t)ep0DataXferRequest.complete);
      /* put the ep0 OUT request into the queue */
      res = ep->ops->reqQueue(usb_handle.pD, ep, &ep0DataXferRequest);
      if (res!=0)
      {
        TU_LOG2("[dcd_edpt_xfer] error(%x) when queue request for ep:%02X, %d bytes, CompCb=%08X\n", res, ep_addr,
                ep0DataXferRequest.length, (uint32_t)ep0DataXferRequest.complete);
      } else
      {
        TU_LOG2("[dcd_edpt_xfer] queued request for ep:%02X, %d bytes, CompCb=%08X\n", ep_addr,
                ep0DataXferRequest.length, (uint32_t)ep0DataXferRequest.complete);
      }
    }
    return true;
  }

  /* for EP0 IN */
  if ((ep_addr&0x0F)==0)
  {
    /* fill in the EP0 request */
    ep0Req.length = total_bytes;
    /* copy data to ep0Buff */
    memcpy(ep0Buff, buffer, total_bytes);
    ep0Req.buf = ep0Buff;
    ep0Req.dma = (uintptr_t)(ep0Req.buf);
    ep0Req.complete = &ctrlInCmplCb;

    /* Add the request to the EP request queue */
    if (ep0Req.length>0)
      usb_handle.pD->ep0NextState = CH9_EP0_DATA_PHASE;
    else
      usb_handle.pD->ep0NextState = CH9_EP0_STATUS_PHASE;
    if ((ep_addr&0xF0)>0)
      usb_handle.pD->ep0DataDirFlag = 1;
    else
      usb_handle.pD->ep0DataDirFlag = 0;

    TU_LOG2("[dcd_edpt_xfer] request for ep:%02X, %d bytes, CompCb=%08X\n", ep_addr, ep0Req.length, (uint32_t)ep0Req.complete);
    res = ep->ops->reqQueue(usb_handle.pD, ep, &ep0Req);
    if (res!=0)
    {
      TU_LOG2("[dcd_edpt_xfer] error(%x) when queue request for ep:%02X, %d bytes, CompCb=%08X\n", res, ep_addr,
            ep0DataXferRequest.length, (uint32_t)ep0DataXferRequest.complete);
    } else
    {
      TU_LOG2("[dcd_edpt_xfer] queued request for ep:%02X, %d bytes, CompCb=%08X\n", ep_addr,
            ep0DataXferRequest.length, (uint32_t)ep0DataXferRequest.complete);
    }

  } else
  {
    /* for EP IN request */
    if ((ep_addr&0xF0)>0)
    {
      /* use dataXferBufferIn for EP IN */
      reqIdx = (ep->address&0x0F) - 1;
      if ((reqIdx<0)||(reqIdx>=DATA_XFER_BUFFER_COUNT))
      {
        TU_LOG2("[dcd_edpt_xfer] Error: incorrect EP address(%02X)\n", ep_addr);
        return false;
      }

      /* use DataXferRequestIn[reqIdx] for EP IN */
      memset(&DataXferRequestsIn[reqIdx], 0, sizeof(CUSBD_Req));
      DataXferRequestsIn[reqIdx].length = total_bytes;
      /* copy data to dataXferBufferIn[reqIdx] */
      memcpy(dataXferBufferIn[reqIdx], buffer, total_bytes);
      DataXferRequestsIn[reqIdx].buf = dataXferBufferIn[reqIdx];
      DataXferRequestsIn[reqIdx].dma = (uintptr_t)(dataXferBufferIn[reqIdx]);
      DataXferRequestsIn[reqIdx].complete = inEpXferCmplCb;

      /* Add the request to the EP IN request queue */
      TU_LOG2("[dcd_edpt_xfer] request(%d) for ep:%02X, %d bytes\n", reqIdx, ep_addr, total_bytes);
      res = ep->ops->reqQueue(usb_handle.pD, ep, &DataXferRequestsIn[reqIdx]);
      if (res!=0)
      {
        TU_LOG2("[dcd_edpt_xfer] error(%x) when queue request for ep:%02X, %d bytes, CompCb=%08X\n", res, ep_addr,
                DataXferRequestsIn[reqIdx].length, (uint32_t)DataXferRequestsIn[reqIdx].complete);
      } else
      {
        TU_LOG2("[dcd_edpt_xfer] queued request for ep:%02X, %d bytes, CompCb=%08X\n", ep_addr,
                DataXferRequestsIn[reqIdx].length, (uint32_t)DataXferRequestsIn[reqIdx].complete);
      }
    } else
    {
      /* for EP Out request */
      reqIdx = (ep->address&0x0F) - 1;
      if ((reqIdx<0)||(reqIdx>=DATA_XFER_BUFFER_COUNT))
      {
        TU_LOG2("[dcd_edpt_xfer] Error: incorrect EP address(%02X)\n", ep_addr);
        return false;
      }

      TU_LOG2("[dcd_edpt_xfer] waiting request(%d) for ep:%02X, %d bytes to buffer=%08X\n", reqIdx, ep_addr,
              total_bytes, (uint32_t)buffer);

      /* use DataXferRequestOut[reqIdx] for EP Out */
      memset(&DataXferRequestsOut[reqIdx], 0, sizeof(CUSBD_Req));

      /* save the TinyUSB buffer pointer in the request */
      DataXferRequestsOut[reqIdx].buf = buffer;
      DataXferRequestsOut[reqIdx].dma = (uintptr_t)(dataXferBufferOut[reqIdx]);
      DataXferRequestsOut[reqIdx].complete = outEpXferCmplCb;
      DataXferRequestsOut[reqIdx].length = total_bytes;
      res = ep->ops->reqQueue(usb_handle.pD, ep, &DataXferRequestsOut[reqIdx]);
      if (res!=0)
      {
        TU_LOG2("[dcd_edpt_xfer] error(%x) when queue request for ep:%02X, %d bytes, CompCb=%08X\n", res, ep_addr,
                total_bytes, (uint32_t)DataXferRequestsOut[reqIdx].complete);
      } else
      {
        TU_LOG2("[dcd_edpt_xfer] queue request for ep:%02X, %d bytes, CompCb=%08X\n", ep_addr,
                total_bytes, (uint32_t)DataXferRequestsOut[reqIdx].complete);
      }
    }
  }

  (void) rhport;
  return true;
}

/* Stall endpoint */
void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr)
{
  CUSBD_Dev * dev;
  CUSBD_Ep * ep;

  (void) rhport;

  TU_LOG2("[dcd_edpt_stall] ep_addr=%d\n", ep_addr);

  usb_handle.drv->getDevInstance(usb_handle.pD, &dev);
  /* Get EP according to ep_addr */
  ep = (CUSBD_Ep *) getEpFromAddr(dev, ep_addr);
  usb_handle.drv->epSetHalt(usb_handle.pD, ep, 1);
}

/* clear stall, data toggle is also reset to DATA0 */
void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr)
{
  CUSBD_Dev * dev;
  CUSBD_Ep * ep;

  (void) rhport;

  TU_LOG2("[dcd_edpt_clear_stall] ep_addr=%d\n", ep_addr);

  usb_handle.drv->getDevInstance(usb_handle.pD, &dev);
  /* Get EP according to ep_addr */
  ep = (CUSBD_Ep *) getEpFromAddr(dev, ep_addr);
  usb_handle.drv->epSetHalt(usb_handle.pD, ep, 0);
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
