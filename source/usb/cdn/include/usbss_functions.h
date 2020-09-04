#ifndef _USBSS_FUNCTIONS_H_
#define _USBSS_FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif


#include <drivers/hw_include/cslr.h>
#include <hw_usb.h>

#include "cusbd_if.h"
#include "cusbd_obj_if.h"
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>

#define USBSS_REG_MISMATCH   (0x00000001)
#define USBSS_CSLR_MMR_TEST  (0x00010000)

/*
 ASSERT MODULE MARKERS:
 -- Convention:
 --    Error codes defined in individual testcases range from:    0x0000_0000 to 0x0000_0FFF
 --    Error codes defined in usbss_functions.h range from:       0x0000_1000 to 0x0000_FFFF
 --    Error codes defined in usb_common.h range from:            0x0001_0000 to 0x00FF_FFFF
*/
#define USB_ASSERT_ERRCODE_MODE_STRAP_POLLCNT_EXCEEDED              0x1001
#define USB_ASSERT_ERRCODE_CMD_CMPLT_UNSUCCESSFUL                   0x1002
#define USB_ASSERT_ERRCODE_USBSS_READY_POLLCNT_EXCEEDED             0x1003

#define base2core(base) ((uintptr_t)((uint64_t)(base)))
#define mkptr64(base,offset)      ((volatile uint64_t *)(base2core((base)+(offset))))
#define mkptr32(base,offset)      ((volatile uint32_t *)(base2core((base)+(offset))))
#define mkptr16(base,offset)      ((volatile uint16_t *)(base2core((base)+(offset))))
#define mkptr8(base,offset)       ((volatile uint8_t *)(base2core((base)+(offset))))
#define mkptr(base,offset)        mkptr32(base,offset)

/*
 *  \brief Data structure for storing variables that are frequently passed to the various USB functions.
 */
typedef struct usb_handle_s {
    uint32_t                  cfg_base;           /* Base of the vbusp cfg interface */
    uint32_t                  pdctl;              /* PSC power domain id assigned to this USB instance */
    uint32_t                  mdctl;              /* PSC module domain id assigned to this USB instance */
    uint32_t                  dinum;              /* Selected USB instance.  Set by usb_instance_select(). */

    /* CSL cfg reg region pointers: */
    CSL_usb3p0ss_cmnRegs      *p_usbregs_tiwrap;      /* TI_Wrap config registers  (selected_inst, core if, rsel0) */
    CSL_usb3p0ss_ctrlRegs     *p_usbregs_ctlr;        /* CDNS_CTRL CFG registers   (selected_inst, core if, rsel3) */
    CSL_usb3p0ss_phy2Regs     *p_usbregs_phy2;        /* USB_phy2 config registers (selected_inst, phy2 if, rsel0) */

    /* global CDN USB device structure */
    CUSBD_OBJ * drv; /* driver pointer */
    CUSBD_PrivateData * pD; /* private data pointer */

    HwiP_Params hwiParamsUsb;
    HwiP_Object hwiObjUsb;    /* *** SoC Enable Interrupts ***  */
} usb_handle_t;

/*
 * \brief Causes the PSC to release mod_g_rst_n for the instance selected by the previous call to the usb_instance_select() function.
 */
uint32_t usb_instance_select ( usb_handle_t *h, uint32_t instance );

void usbss_reset_assert( usb_handle_t *h );

/*
 * \brief Causes the TI Wrapper to release USB3P1SS_W1.pwrup_rst_n for the instance selected by the previous call to the usb_instance_select() function.
 *
 * \b Note:
 * - This is done after programming the PHY3 registers and prior to accessing the main controller registers.
 */
void usbss_reset_release( usb_handle_t *h );

uint8_t check_usbss_ready ( usb_handle_t *h );

/*
 * \brief Configures the IP to operate in host mode.
 *
 * \b Note:
 * - You need to poll for success of this operation by subsequently calling the function check_usbss_host_mode() and checking for a return value of zero.
 */
void set_usbss_host_mode( usb_handle_t *h );

/*
 * \brief Configures the IP to operate in device mode.
 *
 * \b Note:
 * - You need to poll for success of this operation by subsequently calling the function check_usbss_device_mode() and checking for a return value of zero.
 */
void set_usbss_device_mode( usb_handle_t *h );

/*
 * \brief Polls for successful mode switch to host mode.
 *
 * \b Note:
 * - Call this function after calling set_usbss_host_mode().  Verify the return value is zero to ensure successfully switched the mode.
 */
uint8_t check_usbss_host_mode ( usb_handle_t *h );

/*
 * \brief Polls for successful mode switch to device mode.
 *
 * \b Note:
 * - Call this function after calling set_usbss_device_mode().  Verify the return value is zero to ensure successfully switched the mode.
 */
uint8_t check_usbss_device_mode ( usb_handle_t *h );

void usb_local_disable_module ( uint32_t cfg_base );


#ifdef __cplusplus
}
#endif

#endif
