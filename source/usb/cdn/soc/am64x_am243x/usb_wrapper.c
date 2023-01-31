/*
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
 *
 */
/**
 *  \file     am64x/usb_wrapper.c
 *
 *  \brief    This file contains APIs for manipulating the SOC specific wrapper.
 *            As the name suggests this file implements the USB wrapper as
 *            impemented by AM64x.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* #include "tusb_option.h" */
#include "usb_wrapper.h"
#include "device/dcd.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define USBSSP_RING_BOUNDARY            (16384U)
#define PRIVATE_DATA_MEM                (USBSSP_RING_BOUNDARY)

/* ========================================================================== */
/*                 Define CDN related globals here                            */
/* ========================================================================== */

usb_handle_t     usb_handle;      /* global so it can be accessed in ISRs */

/* EP0 IN data buffer */
uint8_t ep0Buff[1024] __attribute__((aligned(8), section(".bss.nocache")));

/** ring boundary */
uint8_t alignedBufCusbd[USBSSP_RING_BOUNDARY] __attribute__((aligned(1024), section(".bss.nocache")));

/* data buffers for OUT EPs */
uint8_t buffEpOut[DATA_XFER_BUFFER_COUNT][1024] __attribute__((aligned(8), section(".bss.nocache")));
/* data buffer pointers for OUT Eps */
uint8_t *dataXferBufferOut[DATA_XFER_BUFFER_COUNT] __attribute__ ((aligned(8), section(".bss.nocache")));
/* requests for OUT EPs */
CUSBD_Req DataXferRequestsOut[DATA_XFER_BUFFER_COUNT]  __attribute__((aligned(8), section(".bss.nocache")));

/* data buffers for IN EPs */
uint8_t buffEpIn[DATA_XFER_BUFFER_COUNT][1024] __attribute__((aligned(8), section(".bss.nocache")));
/* data buffer pointers for IN Eps */
uint8_t *dataXferBufferIn[DATA_XFER_BUFFER_COUNT] __attribute__ ((aligned(8), section(".bss.nocache")));
/* requests for IN EPs */
CUSBD_Req DataXferRequestsIn[DATA_XFER_BUFFER_COUNT]  __attribute__((aligned(8), section(".bss.nocache")));

/* TRB buffers for USB DMA */
CUSBDMA_DmaTrb buff[32][8] __attribute__ ((aligned(8), section(".bss.nocache")));
/* endpoint resource tables */
CUSBDMA_MemResources epMemRes[32]  __attribute__((aligned(8), section(".bss.nocache")));

/* receiving buffer for setup packet (EP0 OUT) */
CH9_UsbSetup setupReqPacket  __attribute__((aligned(8), section(".bss.nocache")));
/* request for EP0 OUT */
CUSBD_Req ep0Req  __attribute__((aligned(8), section(".bss.nocache")));

/* sending buffer for EP0 IN */
uint8_t buffEp0[64] __attribute__((aligned(8), section(".bss.nocache")));
/* request for EP0 IN */
CUSBD_Req ep0DataXferRequest  __attribute__((aligned(8), section(".bss.nocache")));

/*
 There are total 16KB USB FIFO for non-EP0 EPs in the AM64x USB core IP
 To allocate the USB FIFO among non-EP0 EPs, the .bufferingValue for each EP
 is used to tell the USB core how much buffer in KB will be used for each EP IN
 for EP OUT, the .bufferingValue is used to tell the USB core the combined
 USB FIFO for all EP OUT
 ep1in: 2KB
 ep2in: 2KB
 ep3in: 2KB
 ep4in: 2KB
 ep1out + ep2out + ep3out + ep4out: 4KB
*/
CUSBD_Config     usbCfgCusbd = {
    .regBase = CSL_USB0_VBP2APB_WRAP_CONTROLLER_VBP_CORE_ADDR_MAP_BASE+0x20000, /* address where USB core is mapped */
    .epIN =
    {
        {.bufferingValue = 2}, /* ep1in */
        {.bufferingValue = 2}, /* ep2in */
        {.bufferingValue = 2}, /* ep3in */
        {.bufferingValue = 2}, /* ep4in */
        {.bufferingValue = 0}, /* ep5in */
        {.bufferingValue = 0}, /* ep6in */
        {.bufferingValue = 0}, /* ep7in */
        {.bufferingValue = 0}, /* ep8in */
        {.bufferingValue = 0}, /* ep9in */
        {.bufferingValue = 0}, /* ep10in */
        {.bufferingValue = 0}, /* ep11in */
        {.bufferingValue = 0}, /* ep12in */
        {.bufferingValue = 0}, /* ep13in */
        {.bufferingValue = 0}, /* ep14in */
        {.bufferingValue = 0} /* ep15in */
    },
    .epOUT =
    {
        {.bufferingValue = 4}, /* ep1out */
        {.bufferingValue = 4}, /* ep2out */
        {.bufferingValue = 4}, /* ep3out */
        {.bufferingValue = 4}, /* ep4out */
        {.bufferingValue = 0}, /* ep5out */
        {.bufferingValue = 0}, /* ep6out */
        {.bufferingValue = 0}, /* ep7out */
        {.bufferingValue = 0}, /* ep8out */
        {.bufferingValue = 0}, /* ep9out */
        {.bufferingValue = 0}, /* ep10out */
        {.bufferingValue = 0}, /* ep11out */
        {.bufferingValue = 0}, /* ep12out */
        {.bufferingValue = 0}, /* ep13out */
        {.bufferingValue = 0}, /* ep14out */
        {.bufferingValue = 0} /* ep15out */
    },
    .dmultEnabled = 0, /* set to 1 if scatter/gather DMA available */
    .dmaInterfaceWidth = CUSBD_DMA_64_WIDTH,
    .preciseBurstLength = CUSBD_PRECISE_BURST_0,
    .forcedUsbMode = 2, /* force to USB2/HS */
    .setupPacket = &setupReqPacket,
    .setupPacketDma = (uintptr_t)&setupReqPacket
};

CUSBD_SysReq sysReqCusbd;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void outEpXferCmplCb(CUSBD_Ep *ep, CUSBD_Req * req);

int32_t usbClockSSCfg(uint32_t portNumber);
void usb_irq6_isr( void *arg);

static void connect(CUSBD_PrivateData *pD);
static void disconnect(CUSBD_PrivateData *pD);
static void resume(CUSBD_PrivateData *pD);
static uint32_t setup(CUSBD_PrivateData *pD, CH9_UsbSetup *ctrl);
static void suspend(CUSBD_PrivateData *pD);

static CUSBD_Callbacks callback =
{
    disconnect, connect, setup, suspend, resume
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void usbDisplayRequestinfo(CUSBD_Req * req)
{
    TU_LOG2("-------Request INFO-------------- %c\n", ' ');
    TU_LOG2("buf: %08X\n", (uintptr_t) req->buf);
    TU_LOG2("length: %d\n", req->length);
    TU_LOG2("dma: %08X\n", req->dma);
    TU_LOG2("sg:  %c\n", ' ');
    TU_LOG2("streamId: %04X\n", req->streamId);
    TU_LOG2("noInterrupt: %d\n", req->noInterrupt);
    TU_LOG2("zero: %d\n", req->zero);
    TU_LOG2("shortNotOk: %d\n", req->shortNotOk);
    TU_LOG2("context: %08X\n", (uintptr_t) req->context);
    TU_LOG2("status: %08X\n", req->status);
    TU_LOG2("actual: %d\n", req->actual);
}

void usbDisplayEndpointInfo(CUSBD_Ep * ep)
{
    TU_LOG2("-------Endpoint INFO------------- %c\n", ' ');
    TU_LOG2("address: %02X\n", ep->address);
    TU_LOG2("epList: %08X\n", (uintptr_t) & ep->epList);
    TU_LOG2("name: %s\n", ep->name);
    TU_LOG2("ops: %c\n", ' ');
    TU_LOG2("maxPacket: %d\n", ep->maxPacket);
    TU_LOG2("maxStreams: %d\n", ep->maxStreams);
    TU_LOG2("mult: %d\n", ep->mult);
    TU_LOG2("maxburst: %d\n", ep->maxburst);
    TU_LOG2("desc: %08X\n", (uintptr_t) ep->desc);
    TU_LOG2("compDesc: %08X\n", (uintptr_t) ep->compDesc);
}

void usbDisplayDeviceInfo(CUSBD_Dev * dev)
{
    TU_LOG2("-------Device INFO--------------- %c\n", ' ');
    TU_LOG2("epList: prev = %08X, next = %08X\n", (uintptr_t) dev->epList.prev, (uintptr_t) dev->epList.next);
    TU_LOG2("ep0: %08X\n", (uintptr_t) dev->ep0);
    TU_LOG2("speed: %d\n", dev->speed);
    TU_LOG2("maxSpeed: %d\n", dev->maxSpeed);
    TU_LOG2("state: %d\n", dev->state);
    TU_LOG2("sgSupported: %d\n", dev->sgSupported);
    TU_LOG2("name: %s\n", dev->name);
}

static void connect(CUSBD_PrivateData *pD)
{
    CUSBD_Dev *dev;

    usb_handle.drv->getDevInstance(pD, &dev);
    TU_LOG2("Application: connect at %d speed\n",  dev->speed);
    usbDisplayDeviceInfo(dev);
}

static void disconnect(CUSBD_PrivateData *pD)
{
    CUSBD_Dev *dev;
    tusb_speed_t speed;
    usb_handle.drv->getDevInstance(pD, &dev);
    dcd_event_t event = { .rhport = 0, .event_id = DCD_EVENT_BUS_RESET };

    /* pass the acutal speed to TinyUSB driver */
    switch (dev->speed)
    {
        case CH9_USB_SPEED_LOW:
        speed = TUSB_SPEED_LOW;
        break;
        case CH9_USB_SPEED_FULL:
        speed = TUSB_SPEED_FULL;
        break;
        case CH9_USB_SPEED_HIGH:
        speed = TUSB_SPEED_HIGH;
        break;
        default:
        speed = TUSB_SPEED_HIGH;
        break;
    }

    event.bus_reset.speed = speed;
    TU_LOG2("Application: disconnect at %d speed\n",  dev->speed);

    /* send bus reset event to TinyUSB stack */
    dcd_event_handler(&event, true);
}

static void resume(CUSBD_PrivateData *pD){
    TU_LOG2("Application: resume %c\n", ' ');
}

static void reqComplete(CUSBD_Ep *ep, CUSBD_Req * req) {
    TU_LOG2("Transfer complete on ep:%02X %08X req, len=%d(%d)\n", ep->address, (uintptr_t) req, req->length, req->actual);
}

/* setup callback for TinyUSB */
static uint32_t setup(CUSBD_PrivateData *pD, CH9_UsbSetup *ctrl)
{
    dcd_event_t event = { .rhport = 0, .event_id = DCD_EVENT_SETUP_RECEIVED };

    TU_LOG2("setup: %02x, %02x, %04x, %04x, %04x\n",
            ctrl->bmRequestType,
            ctrl->bRequest,
            ctrl->wValue,
            ctrl->wIndex,
            ctrl->wLength);

    memcpy(&event.setup_received, ctrl, 8);

    dcd_event_handler(&event, true);

    return 0;
}

static void suspend(CUSBD_PrivateData *pD)
{
    TU_LOG2("Application: suspend %c\n", ' ');
}

void usbPinmuxConfig(usb_init_param_t *usbInitParamsPtr)
{
    /* Set MAIN_PLL1_HSDIV4_CLKOUT to 24MHz (1920/80 = 24) */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, 0);
    *(volatile uint32_t*)(CSL_PLL0_CFG_BASE+CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL4) = (*(volatile uint32_t*)(CSL_PLL0_CFG_BASE+CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL4) & 0xFFFFFFC0) | (0x50-1);
}

/* turn on the USB2 PHY clock and do its settings
   the clock freq needs to be updated while the USBSS is in reset */
void usbPhyOn(usb_init_param_t *usbInitParamsPtr)
{
    uint32_t usbCmnRegs = CSL_USB0_MMR_MMRVBP_USBSS_CMN_BASE;

    /* assert reset to the USB controller */
    usbss_reset_assert(&usb_handle);

    /* setting VBUS_SEL for external divider - AM64x EVM has a voltage divider
     * for VBUS. If not, we can disable this setting by setting the USB config
     */
    if(usbInitParamsPtr->vbusSel == 1)
    {
        HW_WR_FIELD32(usbCmnRegs + CSL_USB3P0SS_CMN_STATIC_CONFIG,
                  CSL_USB3P0SS_CMN_STATIC_CONFIG_VBUS_SEL, usbInitParamsPtr->vbusSel);
    }

    /* setting PHY ref clock */
    HW_WR_FIELD32(usbCmnRegs + CSL_USB3P0SS_CMN_STATIC_CONFIG,
              CSL_USB3P0SS_CMN_STATIC_CONFIG_PLL_REF_SEL, usbInitParamsPtr->pllRefSel);

    /* release reset */
    usbss_reset_release(&usb_handle);
}

/*
 * Setting up the USB wrapper for USB mode  (host or device mode)
 * on a controller(portNumber)
 * Also need to set the USB2.0 or USB3.0 mode depending on config
 *
 */
#define MODESTRAP_SEL_HOST_MODE     (1)
#define MODESTRAP_SEL_DEVICE_MODE   (2)
#define MODESTRAP_USES_MODE_S_SEL   (1)

static int32_t usbConfig(usb_init_param_t *usbInitParamsPtr)
{
    uint32_t usbBaseRegs = 0;
    uint32_t usbCmnRegs = 0;
    uint32_t modeStrap = 0;

    usbBaseRegs = CSL_USB0_VBP2APB_WRAP_CONTROLLER_VBP_CORE_ADDR_MAP_BASE;
    usbCmnRegs = CSL_USB0_MMR_MMRVBP_USBSS_CMN_BASE;

    /* assert reset to the USB controller */
    usbss_reset_assert(&usb_handle);

    /* set the USB controller to host or device mode */
    if(usbInitParamsPtr->isHostMode == 1)
    {
       modeStrap =  MODESTRAP_SEL_HOST_MODE;
    }
    else
    {
       modeStrap =  MODESTRAP_SEL_DEVICE_MODE;
    }

    HW_WR_FIELD32(usbCmnRegs + CSL_USB3P0SS_CMN_USB3P0SS_W1,
                  CSL_USB3P0SS_CMN_USB3P0SS_W1_MODESTRAP, modeStrap);

    HW_WR_FIELD32(usbCmnRegs + CSL_USB3P0SS_CMN_USB3P0SS_W1,
                  CSL_USB3P0SS_CMN_USB3P0SS_W1_MODESTRAP_SEL,
                  MODESTRAP_USES_MODE_S_SEL);

    /* set to USB2.0 only if USB3.0 is not enabled. i.e SERDES is not used */
    HW_WR_FIELD32(usbCmnRegs + CSL_USB3P0SS_CMN_USB3P0SS_W1,
                  CSL_USB3P0SS_CMN_USB3P0SS_W1_USB2_ONLY_MODE, usbInitParamsPtr->usb2Enable);

    /* release reset */
    usbss_reset_release(&usb_handle);

    /* wait for Controller Not Ready to go 0 */
    while(HW_RD_FIELD32(usbBaseRegs + CSL_USB3P0SS_CTRL_DRD_OTGSTS,
        CSL_USB3P0SS_CTRL_DRD_OTGSTS_OTG_NRDY));

    /* for USB host, needs to wait for XHC_READY */
    if(usbInitParamsPtr->isHostMode == 1)
    {
        /* wait for Controller XHC_READY to go 1 */
        while(!HW_RD_FIELD32(usbBaseRegs + CSL_USB3P0SS_CTRL_DRD_OTGSTS,
            CSL_USB3P0SS_CTRL_DRD_OTGSTS_XHC_READY));
    } else {
        /* wait for Controller DEV_READY to go 1 */
        while(!HW_RD_FIELD32(usbBaseRegs + CSL_USB3P0SS_CTRL_DRD_OTGSTS,
            CSL_USB3P0SS_CTRL_DRD_OTGSTS_DEV_READY));
    }

    return 0;
}

/* setting up USB clocks for USB instance #portNumber
 * which is either 0 or 1
 *
 * EVM has HFOSC0 connected to 25MHz crystal on EVM
 * and this matches with the allowed frequency USB PHY clock. So we will set the
 * USB PHY to use this HFOSC0
 *
 */
void usbClockCfg(usb_init_param_t *usbInitParamsPtr)
{
    uint32_t mmrUsbClkSel;

    mmrUsbClkSel  = CSL_MAIN_CTRL_MMR_CFG0_USB0_CLKSEL;

    /* mux for usb2 ref clock */
    /* #define CSL_FINS(reg, PER_REG_FIELD, val) */
    CSL_FINS(*(uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + mmrUsbClkSel),
                           MAIN_CTRL_MMR_CFG0_USB0_CLKSEL_REFCLK_SEL,
                           usbInitParamsPtr->clkSrcSel); /* HF_OSC0 used */
}

void usbSetupDeviceMode(usb_init_param_t *usbInitParamsPtr)
{
    usbConfig(usbInitParamsPtr);
}

/*
 ---------------------------------------------------------------------------
 Interrupt Service Routines:
 ---------------------------------------------------------------------------
*/

void usb_irq6_isr( void *arg)
{
    CUSBD_Isr( usb_handle.pD ); /* Cadence isr */
}

void usbDeviceIntSetup(usb_init_param_t *usbInitParamsPtr)
{
    /* Initialize the interrupt controller (VIM is already hooked up). */
    HwiP_Params_init(&usb_handle.hwiParamsUsb);
    usb_handle.hwiParamsUsb.intNum = CSLR_R5FSS0_CORE0_INTR_USB0_IRQ_6;
    usb_handle.hwiParamsUsb.callback = usb_irq6_isr;
    usb_handle.hwiParamsUsb.isPulse = 0;
    usb_handle.hwiParamsUsb.isFIQ = 0;
    HwiP_construct(&usb_handle.hwiObjUsb, &usb_handle.hwiParamsUsb);

    /* Enable interrupts for R5 */
    HwiP_enableInt(CSLR_R5FSS0_CORE0_INTR_USB0_IRQ_6);
}

int usbGetDataXferRequestIndexOut(CUSBD_Req * req)
{
    int reqIdx = 0;

    for(reqIdx = 0; reqIdx < DATA_XFER_BUFFER_COUNT; reqIdx++)
    {
        if(&DataXferRequestsOut[reqIdx] == req)
        {
            return reqIdx;
        }
    }

    return -1;
}

/* clears request */
static void clearReq(CUSBD_Req * req)
{
    memset(req, 0, sizeof (CUSBD_Req));
}

/* 
 ---------------------------------------------------------------------------
 USB device initilization:
 ---------------------------------------------------------------------------
*/
void usbDeviceInit(usb_init_param_t *usbInitParamsPtr)
{
    CUSBD_Dev *dev; /* auxiliary pointer used for testing purposes */
    uint32_t res; /* keeps result of operation on */
    int i;

    /* Set pull down control */
    HW_WR_FIELD32(&(usb_handle.p_usbregs_ctlr->DRD.OVERRIDE),
                CSL_USB3P0SS_CTRL_DRD_OVERRIDE_BC_PULLDOWNCTRL, 0x1);

    /* Disable super speed for USB */
    HW_WR_FIELD32(&(usb_handle.p_usbregs_ctlr->DEV.USB_CONF),
                CSL_USB3P0SS_CTRL_DEV_USB_CONF_USB3DIS, 0x1);

    for(i = 0U; i < DATA_XFER_BUFFER_COUNT; i++)
    {
        dataXferBufferOut[i] = buffEpOut[i];
        dataXferBufferIn[i] = buffEpIn[i];
    }


    /* Initialize addresses of resources for all endpoints */
    memset(&buff[0][0], 0, sizeof(CUSBDMA_DmaTrb)*32*8);
    for(i = 0; i < 32; i++)
    {
        epMemRes[i].memPageIndex = 0;
        epMemRes[i].trbAddr = (CUSBDMA_DmaTrb*) buff[i];
        epMemRes[i].trbBufferSize = 8;
        epMemRes[i].trbDmaAddr = (uintptr_t) buff[i];
    }

    /* Set up configuration object of device */
    usbCfgCusbd.epMemRes = &epMemRes;
    usbCfgCusbd.didRegPtr = (uintptr_t)&(usb_handle.p_usbregs_ctlr->DRD.CDNS_DID);
    usbCfgCusbd.ridRegPtr = (uintptr_t)&(usb_handle.p_usbregs_ctlr->DRD.CDNS_RID);
    usbCfgCusbd.dmultEnabled = 0; /* set to 1 if scatter/gather DMA available */
    usbCfgCusbd.dmaInterfaceWidth = CUSBD_DMA_64_WIDTH;
    usbCfgCusbd.preciseBurstLength = CUSBD_PRECISE_BURST_0;
    usbCfgCusbd.forcedUsbMode = 2; /* force to USB2/HS */
    usbCfgCusbd.setupPacket = &setupReqPacket;
    usbCfgCusbd.setupPacketDma = (uintptr_t)&setupReqPacket;

    /* Get driver handle */
    usb_handle.drv = CUSBD_GetInstance();

    /* Query the memory requirement for the driver */
    res = usb_handle.drv->probe(&usbCfgCusbd, &sysReqCusbd);
    if(res != 0)
    {
        goto error;
    }
    else
    {
      TU_LOG2("Device memory requirement: %ld bytes/%ld bytes (prv/trb), allocating\n",
         (long)sysReqCusbd.privDataSize, (long)sysReqCusbd.trbMemSize);
    }

    /* Allocate memory for private data */
    if(PRIVATE_DATA_MEM >= sysReqCusbd.privDataSize)
    {
        usb_handle.pD = (CUSBD_PrivateData*)alignedBufCusbd;
        memset(usb_handle.pD, 0, sysReqCusbd.privDataSize);
    }
    else
    {
        goto error;
    }

    usbCfgCusbd.extendedTBCMode = 0;

    TU_LOG2("Initializing driver\n");

    /* Initialize the driver */
    res = usb_handle.drv->init(usb_handle.pD, &usbCfgCusbd, &callback);
    if(res != 0)
    {
        goto error;
    }

    /* checking device and endpoints parameters correctness */
    /* get CUSBD device instance exposed as Linux gadget device */
    usb_handle.drv->getDevInstance(usb_handle.pD, &dev);
    usbDisplayDeviceInfo(dev);

    /* Start the driver */
    res = usb_handle.drv->start(usb_handle.pD);
    if(res != 0)
    {
        goto error;
    }

    TU_LOG2("Wait for device has been configured ...\n");

    TU_LOG2("... device configured !!!\n");

    TU_LOG2("Initializing driver: Completed\n");
    return;

error:
    TU_LOG2("Initializing driver: Error Occurred\n");
    return;
}
