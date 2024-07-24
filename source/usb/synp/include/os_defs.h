#ifndef _DWC_NO_OS_DEFS_H_
#define _DWC_NO_OS_DEFS_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 *
 * This file contains OS-specific includes and definitions.
 *
 */

#undef linux
#undef __linux
#undef __linux__

#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <string.h>     // for memcpy
#include <stdbool.h>
#include <stdio.h>

/* HACK fixes */
#define NEAR	
#ifndef TRUE
#define TRUE 	1
#endif
#ifndef FALSE
#define FALSE 	0
#endif


/** @{ */
/** Data types needed by the PCD */

typedef unsigned long long      u64;
//typedef unsigned long long      u_int64_t;
typedef unsigned int            u32;
//typedef unsigned int            u_int32_t;
typedef unsigned short          u16;
//typedef unsigned short          u_int16_t;
typedef unsigned char           u8;
//typedef unsigned char           u_int8_t;
typedef unsigned char           UINT8;

typedef long long               s64;
typedef int                     s32;
typedef short                   s16;
typedef char                    s8;

typedef unsigned long   u_long;
typedef unsigned int    u_int;
typedef unsigned short  u_short;
typedef unsigned char   u_char;
/** @} */

/** @{ */
/** Data type for DMA addresses */
typedef unsigned long           dwc_dma_t;
#define DWC_DMA_ADDR_INVALID    (~(dwc_dma_t)0)
/** @} */

/** Compiler 'packed' attribute */
//#define UPACKED               __attribute__((__packed__))
#define UPACKED

/** Compiler 'aligned(16)' attribute ---  doesn't work with cl470. Use pragma DATA_ALIGN instead */
#define UALIGNED16    __attribute__((__aligned__(16)))

/** I/O memory attribute for pointers. Needed for Linux "sparse" tool. */
#define __iomem         /* */

#define KERN_DEBUG      "debug"          /* debug messages */
#define KERN_INFO       "info"          /* informational messages */
#define KERN_WARNING    "war"          /* warning messages */
#define KERN_ERR        "err"          /* error messages */

// from /usr/include/asm-x86_64/errno.h
#define EIO             5       /* I/O error */
#define EAGAIN          11      /* Try again */
#define ENOMEM          12      /* Out of memory */
#define EBUSY           16      /* Device or resource busy */
#define ENODEV          19      /* No such device */
#define EINVAL          22      /* Invalid argument */
#define ENOSPC          28      /* No space left on device */
#define EPIPE           32      /* Broken pipe */
#define EDOM            33      /* Math argument out of domain of func */
#define ENODATA         61      /* No data available */
#define ENOSR           63      /* Out of streams resources */
#define ECOMM           70      /* Communication error on send */
#define EPROTO          71      /* Protocol error */
#define EOVERFLOW       75      /* Value too large for defined data type */
#define ERESTART        85      /* Interrupted system call should be restarted */
#define EOPNOTSUPP      95      /* Operation not supported on transport endpoint */
#define ECONNABORTED    103     /* Software caused connection abort */
#define ECONNRESET      104     /* Connection reset by peer */
#define ESHUTDOWN       108     /* Cannot send after transport endpoint shutdown */
#define ETIMEDOUT       110     /* Connection timed out */
#define EINPROGRESS     115     /* Operation now in progress */

/** Write memory barrier macro */
#define wmb()           do {} while ((bool)(0))

#define interrupt_disable()     0
#define interrupt_enable()      do {} while ((bool)(0))

#define dwc_init_spinlock(d, p)                 do {} while ((bool)(0))
#define dwc_acquire_spinlock(d, p)              do {} while ((bool)(0))
#define dwc_release_spinlock(d, p)              do {} while ((bool)(0))
#define dwc_acquire_spinlock_irq(d, p, f)       do { (f) = interrupt_disable(); } while ((bool)(0))
#define dwc_release_spinlock_irq(d, p, f)       do { if (f > 0U) { interrupt_enable(); } } while ((bool)(0))

struct task_struct {
        int dummy;
};

struct tasklet_struct {
        int dummy;
};


#include "usb.h"
#include "dwc_list.h"

/** @name Error Codes */
/** @{ */
#define DWC_E_INVALID           EINVAL
#define DWC_E_NO_MEMORY         ENOMEM
#define DWC_E_NO_DEVICE         ENODEV
#define DWC_E_NOT_SUPPORTED     EOPNOTSUPP
#define DWC_E_TIMEOUT           ETIMEDOUT
#define DWC_E_BUSY              EBUSY
#define DWC_E_AGAIN             EAGAIN
#define DWC_E_RESTART           ERESTART
#define DWC_E_ABORT             ECONNABORTED
#define DWC_E_SHUTDOWN          ESHUTDOWN
#define DWC_E_NO_DATA           ENODATA
#define DWC_E_DISCONNECT        ECONNRESET
#define DWC_E_UNKNOWN           EINVAL
#define DWC_E_NO_STREAM_RES     ENOSR
#define DWC_E_COMMUNICATION     ECOMM
#define DWC_E_OVERFLOW          EOVERFLOW
#define DWC_E_PROTOCOL          EPROTO
#define DWC_E_IN_PROGRESS       EINPROGRESS
#define DWC_E_PIPE              EPIPE
#define DWC_E_IO                EIO
#define DWC_E_NO_SPACE          ENOSPC
#define DWC_E_DOMAIN            EDOM
/** @} */

/**
 * The number of DMA Descriptors (TRBs) to allocate for each endpoint type.
 * NOTE: The driver currently supports more than 1 TRB for Isoc EPs only.
 * So the values for Bulk and Intr must be 1.
 */
#define DWC_NUM_BULK_TRBS       1
#define DWC_NUM_INTR_TRBS       1
#define DWC_NUM_ISOC_TRBS       32

/**
 * These parameters may be specified when loading the module. They define how
 * the DWC_usb3 controller should be configured. The parameter values are passed
 * to the CIL initialization routine dwc_usb3_pcd_common_init().
 */
typedef struct dwc_usb3_core_params {
        int burst;
        int newcore;
        int phy;
        int wakeup;
        int pwrctl;
        int lpmctl;
        int phyctl;
        int usb2mode;
        int hibernate;
        int hiberdisc;
        int clkgatingen;
        int ssdisquirk;
        int nobos;
        int loop;
        int nump;
        int newcsr;
        int rxfsz;
        int txfsz[16];
        int txfsz_cnt;
        int baseline_besl;
        int deep_besl;
        int besl;
} dwc_usb3_core_params_t;

// linux/usb/gadget.h

/**
 * Platform-specific USB endpoint
 */
typedef struct usb_ept {
        const void      *desc;
        const void      *comp_desc;
        unsigned        maxpacket:16;
        u8              address;
} usb_ep_t;

/**
 * Platform-specific USB request
 */
typedef struct usb_request {
        void            *buf;
        unsigned        length;
        dwc_dma_t       dma;

        unsigned        stream_id:16;
        unsigned        zero:1;

        void            (*complete)(usb_ep_t *ep, struct usb_request *req);

        int             status;
        unsigned        actual;
} usb_request_t;

/**
 */
static inline void dwc_usb3_task_schedule(struct tasklet_struct *tasklet)
{
#ifdef LINUXTEST
        tasklet_schedule(tasklet);
#endif
}

/* Make the following structure type definitions "packed" if using a Microsoft
 * compiler. The UPACKED attribute (defined above) embedded in the structure
 * type definitions does the same thing for GCC. Other compilers may need
 * something different.
 */
#ifdef _MSC_VER
#include <pshpack1.h>
#endif


typedef struct usb_dfu_functional_descriptor_s {
        uByte           bLength;            /* size of descriptor = 0x9 */
        uByte           bDescriptorType;    /* DFU functional descriptor type. 0x21 */
        uByte           bmAttributes;       /* DFU attributes */
        uWord           wDetachTimeOut;     /* max time (ms) wait for reset after DFU_DETACH */
        uWord           wTransferSize;      /* maximum number of bytes that device can accept
                                               per control-write transaction */
        uWord           bcdDFUVersion;
} usb_dfu_functional_descriptor_t;

/**
 */
typedef struct fs_msc_config_desc_st {
        usb_config_descriptor_t                 config_desc;
        usb_interface_descriptor_t              intf_desc;
        usb_endpoint_descriptor_t               bulk_in_ep_desc;
        usb_endpoint_descriptor_t               bulk_out_ep_desc;
} UPACKED fs_msc_config_desc_t;


/**
 */
typedef struct hs_msc_config_desc_st {
        usb_config_descriptor_t                 config_desc;
        usb_interface_descriptor_t              intf_desc;
        usb_endpoint_descriptor_t               bulk_in_ep_desc;    // only one IN bulk EP in the Launchpad example
        usb_endpoint_descriptor_t               bulk_out_ep_desc;   // removed this
} UPACKED hs_msc_config_desc_t;

typedef struct hs_dfu_config_desc_st {
        usb_config_descriptor_t                 config_desc;
        usb_interface_descriptor_t              intf_desc;
        usb_interface_descriptor_t              intf1_desc;
        usb_dfu_functional_descriptor_t         dfu_func_desc;
} UPACKED dfu_config_desc_t;
/**
 */
typedef struct ss_config_desc_st {
        usb_config_descriptor_t                 config_desc;
        usb_interface_descriptor_t              intf_desc;
        usb_endpoint_descriptor_t               bulk_in_ep_desc;
        ss_endpoint_companion_descriptor_t      bulk_in_ss_ep_comp_desc;
        usb_endpoint_descriptor_t               bulk_out_ep_desc;
        ss_endpoint_companion_descriptor_t      bulk_out_ss_ep_comp_desc;
} UPACKED ss_config_desc_t;

/* Stop packing structure type definitions */
#ifdef _MSC_VER
#include <poppack.h>
#endif

/* These structures are defined in no_os_ep0.c */

#ifdef USB_BOOT_DFU
extern const dfu_config_desc_t dfu_config_desc;
#else
extern fs_msc_config_desc_t fs_msc_config_desc;
extern hs_msc_config_desc_t hs_msc_config_desc;
#endif

extern ss_config_desc_t ss_config_desc;

/**
 * Function driver API routines
 */

struct dwc_usb3_device; /* so the simulation code can include just this file */

extern const dwc_usb3_core_params_t usb3ss_module_params;

#ifndef LINUXTEST
extern struct dwc_usb3_device *dwc_usb3_driver_init(u32 base_addr_dwc);
extern void dwc_usb3_driver_remove(void);
extern void dwc_usb3_common_irq(int irq, void *dev);
#endif

extern usb_ep_t *dwc_usb3_ep_enable(struct dwc_usb3_device *usb3_dev, const void *epdesc,
                                    const void *epcomp);
extern int dwc_usb3_ep_disable(struct dwc_usb3_device *usb3_dev, usb_ep_t *usb_ep);
extern int dwc_usb3_close_all_ep(struct dwc_usb3_device *usb3_dev);
extern usb_request_t *dwc_usb3_alloc_request(struct dwc_usb3_device *usb3_dev, usb_ep_t *usb_ep);
extern void dwc_usb3_free_request(struct dwc_usb3_device *usb3_dev, usb_ep_t *usb_ep,
                                  usb_request_t *usb_req);
extern int dwc_usb3_ep_queue(struct dwc_usb3_device *usb3_dev, usb_ep_t *usb_ep,
                             usb_request_t *usb_req);
extern int dwc_usb3_ep_dequeue(struct dwc_usb3_device *usb3_dev, usb_ep_t *usb_ep,
                               usb_request_t *usb_req);
extern int dwc_usb3_wait_pme(struct dwc_usb3_device *usb3_dev);
extern int dwc_usb3_handle_pme_intr(struct dwc_usb3_device *usb3_dev);

extern int dwc_usb3_function_init(struct dwc_usb3_device *usb3_dev);
extern void dwc_usb3_function_remove(struct dwc_usb3_device *usb3_dev);
extern int dwc_usb3_function_connect(struct dwc_usb3_device *usb3_dev, int speed);
extern int dwc_usb3_function_disconnect(struct dwc_usb3_device *usb3_dev);

#ifdef __cplusplus
}
#endif

#endif /* _DWC_NO_OS_DEFS_H_ */
