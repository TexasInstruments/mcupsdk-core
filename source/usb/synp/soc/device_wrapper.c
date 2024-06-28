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

#include <stdint.h>
#include "hw_types.h"
#include "device_wrapper.h"
#include "device_usb_reg_offset.h"

/* 5 milli sec timeout for per pll locking and usb reg reset while loop */ 
#define CONFIG_TIMEOUT   (5U)

usb_handle_t usb_handle __attribute__((section(".usbdfuCxtRam")));      /* global so it can be accessed in ISRs */

/**
 *  @b  Description 
 *  @n 
 *      Static pointer to topRCM MMR space 
 */ 
static CSL_top_rcmRegs * ptrTopRCMRegs = (CSL_top_rcmRegs *)DEVICE_TOPRCM_BASE;

void usbdIntrConfig();
void usbCoreIntrHandler();

/*
 *  ======== borad_init ========
 */
void USB_init()
{
    DebugP_log("USB Init");
    usb_handle.cfg_base = USB_DWC_3;
    usb_handle.dwc_usb3_dev = NULL;

    if (usb_phy_power_sequence() == USB_PHY_OK )
    {
        usbdIntrConfig();
    }
    else
    {
        DebugP_assert(FALSE);
    }
}

void USB_dwcTask()
{
    dwc_usb3_task(usb_handle.dwc_usb3_dev);
}

void usbdIntrConfig()
{
	HwiP_Params usb_hwi_handle = usb_handle.hwiParamsUsb;

    /* Initialize the interrupt controller (VIM is already hooked up). */
    HwiP_Params_init(&usb_handle.hwiParamsUsb);
    usb_hwi_handle.intNum = USB20_MAIN0_INT;
    usb_hwi_handle.callback = usbCoreIntrHandler;
    HwiP_construct(&usb_handle.hwiObjUsb, &usb_hwi_handle);

    /* Enable interrupts for R5 */
    HwiP_enableInt(USB20_MAIN0_INT);
}

/* main entry point for DWC core interrupt handler with USB Wrapper setup
* Matching interrupt call-back function API */
void usbCoreIntrHandler()
{
    /*Clear USB Interrrupts*/
    USB_clearInterrupt();
    /* Call device interrupt handler */
    dwc_usb3_irq(usb_handle.dwc_usb3_dev, 0xdd);
}

/* configure PER PLL 960Mhz clock and route it to USB UTMI PHy */ 
static inline UsbPhy_ret_t per_pll_usb_clk_cfg(void)
{
	/* Do USB PLL settings here */ 
	uint8_t status ; 
	UsbPhy_ret_t ret = USB_PHY_OK ; 
	uint32_t startTime, curTime;

    CycleCounterP_reset();
	startTime = CycleCounterP_getCount32() ; 
	curTime = startTime ;  

	/* Set M2DIV_M2 = 1  & M2DIV_N = 0x10 */ 
	CSL_REG32_WR(&ptrTopRCMRegs->PLL_PER_M2NDIV, 0x00010009U) ;

	/* set MN2DIV = 0x00000180 */ 
	CSL_REG32_WR(&ptrTopRCMRegs->PLL_PER_MN2DIV, 0x00000180U) ;
	CSL_REG32_WR(&ptrTopRCMRegs->PLL_PER_FRACDIV, 0x04000000U) ;
	CSL_REG32_WR(&ptrTopRCMRegs->PLL_PER_CLKCTRL, 0x200B4800U) ;
	/* TENABLE = 1 */ 
	CSL_REG32_WR(&ptrTopRCMRegs->PLL_PER_TENABLE,1U); 

	CSL_FINSR(ptrTopRCMRegs->PLL_PER_CLKCTRL,0U,0U,1U); 

	CSL_REG32_WR(&ptrTopRCMRegs->PLL_PER_TENABLE,0U); 
	CSL_REG32_WR(&ptrTopRCMRegs->PLL_PER_TENABLEDIV,1U); 
	CSL_REG32_WR(&ptrTopRCMRegs->PLL_PER_TENABLEDIV,0U); 
	
	/* Wait till pll is locked 10th bit */
	/* Add 5ms timeout value to recover from this
	 * However this is a safe loop */ 
	status =  (uint8_t)CSL_FEXTR(ptrTopRCMRegs->PLL_PER_STATUS, 10U, 10U);
	while((status != 0x1U) && ( curTime - startTime <= CONFIG_TIMEOUT))
	{
		status =  (uint8_t)CSL_FEXTR(ptrTopRCMRegs->PLL_PER_STATUS, 10U, 10U);
		curTime = CycleCounterP_getCount32() ; 
	}
	if( curTime - startTime > CONFIG_TIMEOUT)
	{
		ret = USB_PHY_ERR ; 
	}
	else
	{
		ret = USB_PHY_OK ; 
	}

	return ret ; 
}

/* Power on sequence for USB */
UsbPhy_ret_t usb_phy_power_sequence(void){

    CycleCounterP_reset();
	uint32_t startTime = CycleCounterP_getCount32() ; 
	uint32_t curTime = startTime ;  
	UsbPhy_ret_t status = USB_PHY_OK;
	/* Enable clock and route to USB PHY */ 
	if( per_pll_usb_clk_cfg() == USB_PHY_ERR ) 
	{
		status = USB_PHY_ERR ; 
	}
	
    /* Configuring ocp2scp to sync ocp and scp clock */ 
    HW_WR_FIELD32_RAW(USB_OCP2SCP_REG + USB_OCP2SCP_REG_TIMING, 0x00000001,0,1);
    HW_WR_REG32(USB_OCP2SCP_REG + USB_OCP2SCP_REG_TIMING , 0x385);

    /* IDDIG. reset value = 1 , SESSEND . reset value = x1 
	 * VBUSVALID. set 1, BVALID . set 1 , AVALID . set 1 
	 */ 
    HW_WR_FIELD32_RAW(MSS_CTRL + MSS_CTRL_CONTROL_USBOTGHS_CONTROL, 0x0000000F,0,0xF);

    /* Turn on PHY/OTGSS (no impact in Presilicon, since there is no PHY involved) */ 
	/* set this field to 0 to enable phy */ 
    HW_WR_FIELD32_RAW(MSS_CTRL + MSS_CTRL_CTRL_USB_CTRL, 0x00000001,0,0); // CM_PWRDN . 0x0

    /* wait until CNR flag in USBSTS is '0' before writing any xHC registers -- reset successful */ 
	/* TODO: Add timeout value here for safety 
	 * Mostly this is safe but won't hurt to put a timeout here */ 
    while(((HW_RD_REG32(USB_DWC_3 + USB_DWC_3_USBSTS) & 0x800) == 1) &&
			(curTime - startTime) <= CONFIG_TIMEOUT) 
	{
		curTime = CycleCounterP_getCount32() ;
	}
	if(curTime - startTime > CONFIG_TIMEOUT)
	{
		status = USB_PHY_ERR ; 
	}

	if (status == USB_PHY_OK)
	{
		/* ID and VBUS status for controller are taken from UTMI inputs */ 
		HW_WR_REG32(USB_OTGSS_C2 + USB_OTGSS_C2_UTMI_OTG_STATUS , 0x18);

		/* bring up PHY */ 
		HW_WR_REG32(USB_DWC_3 + USB_DWC_3_GUSB2PHYCFG, 0x00002500);
	}

	return status;
}

/* Configure USB events and interrupts */
void USB_configureInterrupt(uint32_t intr){

    HW_WR_FIELD32_RAW(USB_OTGSS_C2 + USB_OTGSS_C2_IRQENABLE_SET_MAIN_0 + 0x10*intr, 0x00000001,0,1);
}

/* Configure USB events and interrupts */
void USB_disableInterrupt(uint32_t intr){

    HW_WR_FIELD32_RAW(USB_OTGSS_C2 + USB_OTGSS_C2_IRQENABLE_CLR_MAIN_0 + 0x10*intr, 0x00000001,0,1);
}
/* helper function to Clear main0 interrupt */
void USB_clearInterrupt(void)
{
    /* Rest of the interrupts are not used thus clearing only Main0 interrupt */
    HW_WR_REG32(USB_OTGSS_C2 + USB_OTGSS_C2_IRQSTATUS_MAIN_0, 0x1);
}

