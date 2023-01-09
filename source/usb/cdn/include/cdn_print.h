/******************************************************************************
 * Copyright (C) 2011-2018 Cadence Design Systems, Inc.
 * All rights reserved worldwide
 *
 * The material contained herein is the proprietary and confidential
 * information of Cadence or its licensors, and is supplied subject to, and may
 * be used only by Cadence's customer in accordance with a previously executed
 * license and maintenance agreement between Cadence and that customer.
 *
 ******************************************************************************
 * cdn_print.h
 * Print debug logs
 *
 * This is the example implementation of DbgPrint used in debug build
 ******************************************************************************
 */

 /**
 *  \defgroup USB_MODULE APIs for USB
 *  \ingroup DRV_MODULE
 *
 *  This module has APIs for USB device driver porting layer.
 *  See this page, \ref USB_DEVICE_DRIVER, for using USB using tinyUSB APIs
 *  @{
 */
#ifdef __cplusplus
extern "C"
{
#endif

#include "stdarg.h"
#include "stdio.h"
#include "stdlib.h"
#include "stddef.h"
#include "stdint.h"
#include <usb/cdn/core_driver/common/include/cdn_log.h>

/** \defgroup CUSBD_MODULE_IDS cadence USB device driver module Id 
 *  \ingroup USB_MODULE 
 *
 *  ModuleIds for CUSBD modules 
 *  @{
 */
/** Module Id for CUSBD */ 
#define USBSSP_DBG_CUSBD            0x00000010U 

/** Module Id for CUSBD_ISR */ 
#define USBSSP_DBG_CUSBD_ISR        0x00000001U

/** Module Id for DMA_BASIC_MSG */ 
#define DBG_DMA_BASIC_MSG           0x00000100U

/** Module Id for DMA_VERBOSE_MSG */ 
#define DBG_DMA_VERBOSE_MSG         0x00000200U

/** Module Id for DMA_ERR_MSG */ 
#define DBG_DMA_ERR_MSG             0x00000400U

/** Module Id for DMA_CHANNAL_USEGE_MSG */ 
#define DBG_DMA_CHANNEL_USEGE_MSG   0x00001000U
/**@} */

/**
 * \brief 
 *  
 *  Printf function hook for cadence usb device driver */ 
inline void DbgPrint(uint32_t module_id, uint32_t log_lvl, const char *str, ...);

/** 
 * \brief 
 * Initialize CUSBD Debug module with default parameters 
 */ 
void CUSBD_DbgMsgInit(void);

/** \brief Sets debug verbosity level 
 *
 *  valid arguments 
 *
 *  1. DBG_CRIT - critical         
 * 	2. DBG_WARN - warning
 *  3. DBG_FYI - fyi
 *  4. DBG_HIVERB - highly verbose
 */
void DbgMsgSetLvl(uint32_t log_lvl);

/** 
 * \brief enable logs for given module 
 *  
 *  @param module_id module id of CUSBD module who's 
 *		  logs will be enabled 
 */ 
void DbgMsgEnableModule(uint32_t module_id); 

/** \brief disable logs for given module 
 *
 *  @param module_id module id of CUSBD module who's 
 *		  logs will be disbaled 
 */ 
void DbgMsgDisableModule(uint32_t module_id); 


#ifdef DEBUG 

#define DEFAULT_CDN_DBG_MODULE          ( USBSSP_DBG_CUSBD | DBG_DMA_BASIC_MSG )
#define DEFAULT_CDN_LOG_LVL              (DBG_FYI)


#else

#define DEFAULT_CDN_DBG_MODULE           (0)
#define DEFAULT_CDN_LOG_LVL              (0)

#endif

/**@} */

#ifdef __cplusplus
}
#endif


