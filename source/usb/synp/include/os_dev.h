/* ==========================================================================
 * $File: //dwh/usb_iip/dev/software/DWC_usb3/driver/no_os/no_os_dev.h $
 * $Revision: #1 $
 * $Date: 2012/08/09 $
 * $Change: 2046772 $
 *
 * Synopsys SS USB3 Linux Software Driver and documentation (hereinafter,
 * "Software") is an Unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you.
 *
 * The Software IS NOT an item of Licensed Software or Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Product
 * with Synopsys or any supplement thereto. You are permitted to use and
 * redistribute this Software in source and binary forms, with or without
 * modification, provided that redistributions of source code must retain this
 * notice. You may not view, use, disclose, copy or distribute this file or
 * any information contained herein except pursuant to this license grant from
 * Synopsys. If you do not agree with this notice, including the disclaimer
 * below, then you are not authorized to use the Software.
 *
 * THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 * ========================================================================== */

#ifndef _DWC_NO_OS_DEV_H_
#define _DWC_NO_OS_DEV_H_

#include <kernel/nortos/dpl/common/printf.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @file
 */

/** Wrapper routine for _handshake() */
#define handshake(_dev_, _ptr_, _mask_, _done_)	\
	_handshake(_dev_, _ptr_, _mask_, _done_)

/** Takes a usb req pointer and returns the associated pcd req pointer */
#define dwc_usb3_get_pcd_req(usbreq)			\
	((dwc_usb3_pcd_req_t *)((char *)(usbreq) -	\
		offsetof(struct dwc_usb3_pcd_req, usb_req)))

/** Takes a usb ep pointer and returns the associated pcd ep pointer */
#define dwc_usb3_get_pcd_ep(usbep)			\
	((dwc_usb3_pcd_ep_t *)((char *)(usbep) -	\
		offsetof(struct dwc_usb3_pcd_ep, usb_ep)))

/** @{ */
/**
 * Register read/write.
 */
static inline u32 dwc_rd32(struct dwc_usb3_device *dev, volatile u32 __iomem *adr)
{
	return *adr;
}

static inline void dwc_wr32(struct dwc_usb3_device *dev, volatile u32 __iomem *adr, u32 val)
{
	*adr = val;
}
static inline void chipDelay32(int delay)
{
}
/** @} */

/** @{ */
/**
 * Non-sleeping delays.
 */

//#define dwc_udelay(dev, us)	do {} while (0)
//#define dwc_mdelay(dev, ms)	do {} while (0)
//
#define dwc_udelay(dev, us)	chipDelay32 (us)
#define dwc_mdelay(dev, ms)	chipDelay32 (ms*1000)

/** @} */

/**
 * Sleeping delay.
 */
#define dwc_msleep(dev, ms)	chipDelay32 (ms*1000)

/*
 * Debugging support - vanishes in non-debug builds.
 */

//extern char* printBuffer;
//extern unsigned int bufferIdx;

/** Prefix string for print macros */
#define USB3_DWC	"dwc_usb3: "

/** @{ */
/** Print a debug message */
#ifdef DEBUG
# define dwc_debug(dev, x...)	printf(KERN_DEBUG USB3_DWC x);printf("\n\r");
//# define dwc_debug(dev, x...)	bufferIdx += sprintf(&printBuffer[bufferIdx], KERN_DEBUG USB3_DWC x )
#else
# define dwc_debug(dev, x...)	do {} while ((bool)(0))
#endif /* DEBUG */

# define dwc_debug0(dev, fmt)			dwc_debug(dev, fmt)
# define dwc_debug1(dev, fmt, a)		dwc_debug(dev, fmt, a)
# define dwc_debug2(dev, fmt, a, b)		dwc_debug(dev, fmt, a, b)
# define dwc_debug3(dev, fmt, a, b, c)		dwc_debug(dev, fmt, a, b, c)
# define dwc_debug4(dev, fmt, a, b, c, d)	dwc_debug(dev, fmt, a, b, c, d)
# define dwc_debug5(dev, fmt, a, b, c, d, e) \
			dwc_debug(dev, fmt, a, b, c, d, e)
# define dwc_debug6(dev, fmt, a, b, c, d, e, f) \
			dwc_debug(dev, fmt, a, b, c, d, e, f)
# define dwc_debug7(dev, fmt, a, b, c, d, e, f, g) \
			dwc_debug(dev, fmt, a, b, c, d, e, f, g)
# define dwc_debug8(dev, fmt, a, b, c, d, e, f, g, h) \
			dwc_debug(dev, fmt, a, b, c, d, e, f, g, h)
# define dwc_debug9(dev, fmt, a, b, c, d, e, f, g, h, i) \
			dwc_debug(dev, fmt, a, b, c, d, e, f, g, h, i)
# define dwc_debug10(dev, fmt, a, b, c, d, e, f, g, h, i, j) \
			dwc_debug(dev, fmt, a, b, c, d, e, f, g, h, i, j)
/** @} */

/** @{ */
/** Print an isochronous debug message */
#if defined(DEBUG) || defined(ISOC_DEBUG)
# define dwc_isocdbg(dev, x...)		printf(KERN_DEBUG USB3_DWC x )
#else
# define dwc_isocdbg(dev, x...)		do {} while ((bool)(0))
#endif

# define dwc_isocdbg0(dev, fmt)			dwc_isocdbg(dev, fmt)
# define dwc_isocdbg1(dev, fmt, a)		dwc_isocdbg(dev, fmt, a)
# define dwc_isocdbg2(dev, fmt, a, b)		dwc_isocdbg(dev, fmt, a, b)
# define dwc_isocdbg3(dev, fmt, a, b, c)	dwc_isocdbg(dev, fmt, a, b, c)
# define dwc_isocdbg4(dev, fmt, a, b, c, d) \
			dwc_isocdbg(dev, fmt, a, b, c, d)
# define dwc_isocdbg5(dev, fmt, a, b, c, d, e) \
			dwc_isocdbg(dev, fmt, a, b, c, d, e)
# define dwc_isocdbg6(dev, fmt, a, b, c, d, e, f) \
			dwc_isocdbg(dev, fmt, a, b, c, d, e, f)
/** @} */

/** @{ */
/** Print a error message */
#ifdef DEBUG
# define dwc_error(dev, x...)   printf(KERN_ERR USB3_DWC x);printf("\n\r");
//# define dwc_debug(dev, x...) bufferIdx += sprintf(&printBuffer[bufferIdx], KERN_DEBUG USB3_DWC x )
#else
# define dwc_error(dev, x...)   do {} while ((bool)(0))
#endif /* DEBUG */

#define dwc_error0(dev, fmt)			dwc_error(dev, fmt)
#define dwc_error1(dev, fmt, a)			dwc_error(dev, fmt, a)
#define dwc_error2(dev, fmt, a, b)		dwc_error(dev, fmt, a, b)
#define dwc_error3(dev, fmt, a, b, c)		dwc_error(dev, fmt, a, b, c)
#define dwc_error4(dev, fmt, a, b, c, d)	dwc_error(dev, fmt, a, b, c, d)
/** @} */

/** @{ */
/** Print a Warning message */
//#define dwc_warn(dev, x...)	printf(KERN_WARNING USB3_DWC x );printf("\r")

/** @{ */
/** Print a error message */
#ifdef DEBUG
# define dwc_warn(dev, x...)   printf(KERN_WARNING USB3_DWC x);printf("\n\r");
//# define dwc_debug(dev, x...) bufferIdx += sprintf(&printBuffer[bufferIdx], KERN_DEBUG USB3_DWC x )
#else
# define dwc_warn(dev, x...)   do {} while ((bool)(0))
#endif /* DEBUG */


#define dwc_warn0(dev, fmt)			dwc_warn(dev, fmt)
#define dwc_warn1(dev, fmt, a)			dwc_warn(dev, fmt, a)
#define dwc_warn2(dev, fmt, a, b)		dwc_warn(dev, fmt, a, b)
#define dwc_warn3(dev, fmt, a, b, c)		dwc_warn(dev, fmt, a, b, c)
#define dwc_warn4(dev, fmt, a, b, c, d)		dwc_warn(dev, fmt, a, b, c, d)
/** @} */

/** @{ */
/** Print an Informational message (normal but significant) */
//#define dwc_info(dev, x...)	printf(KERN_INFO USB3_DWC x ); printf("\r")
/** @{ */
/** Print a error message */
#ifdef DEBUG
#define dwc_info(dev, x...)   printf(KERN_INFO USB3_DWC x);printf("\n\r");
//# define dwc_debug(dev, x...) bufferIdx += sprintf(&printBuffer[bufferIdx], KERN_DEBUG USB3_DWC x )
#else
#define dwc_info(dev, x...)   do {} while ((bool)(0))
#endif /* DEBUG */

#define dwc_info0(dev, fmt)			dwc_info(dev, fmt)
#define dwc_info1(dev, fmt, a)			dwc_info(dev, fmt, a)
#define dwc_info2(dev, fmt, a, b)		dwc_info(dev, fmt, a, b)
#define dwc_info3(dev, fmt, a, b, c)		dwc_info(dev, fmt, a, b, c)
#define dwc_info4(dev, fmt, a, b, c, d)		dwc_info(dev, fmt, a, b, c, d)
/** @} */

/** @{ */
/** Basic message printing */
//#define dwc_print(dev, x...)	printf(USB3_DWC x );printf("\r")
#ifdef DEBUG
#define dwc_print(dev, x...)   printf(KERN_INFO USB3_DWC x);printf("\n\r");
//# define dwc_debug(dev, x...) bufferIdx += sprintf(&printBuffer[bufferIdx], KERN_DEBUG USB3_DWC x )
#else
#define dwc_print(dev, x...)   do {} while ((bool)(0))
#endif /* DEBUG */

#define dwc_print0(dev, fmt)			dwc_print(dev, fmt)
#define dwc_print1(dev, fmt, a)			dwc_print(dev, fmt, a)
#define dwc_print2(dev, fmt, a, b)		dwc_print(dev, fmt, a, b)
#define dwc_print3(dev, fmt, a, b, c)		dwc_print(dev, fmt, a, b, c)
#define dwc_print4(dev, fmt, a, b, c, d)	dwc_print(dev, fmt, a, b, c, d)
#define dwc_print5(dev, fmt, a, b, c, d, e) \
			dwc_print(dev, fmt, a, b, c, d, e)
/** @} */

extern int dwc_usb3_gadget_init(struct dwc_usb3_device *usb3_dev);
extern void dwc_usb3_gadget_remove(struct dwc_usb3_device *usb3_dev);
extern int dwc_usb3_no_os_setup(dwc_usb3_pcd_t *pcd, usb_device_request_t *ctrl);
extern int dwc_usb3_function_setup(dwc_usb3_pcd_t *pcd, usb_device_request_t *ctrl);

#ifdef __cplusplus
}
#endif

#endif /* _DWC_NO_OS_DEV_H_ */
