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
 * cdn_print.c
 * Print debug logs
 *
 * This is the example implementation of DbgPrint used in debug build
 ******************************************************************************
 */

#include "stdarg.h"
#include "stdio.h"
#include "stdlib.h"
#include "stddef.h"
#include "stdint.h"
#include "string.h"

#include "cdn_log.h"
#include "cdn_print.h"
#include <kernel/nortos/dpl/common/printf.h>

uint32_t g_dbg_enable_log  = 0x00000000; /* DBG_GEN_MSG; */
uint32_t g_dbg_log_lvl = 0; /* DBG_FYI; */  /* DBG_HIVERB; */
uint32_t g_dbg_log_cnt = 0;
uint32_t g_dbg_state = 0;

#ifdef DEBUG
/* DbgPrint is required for DEBUG build */
inline void DbgPrint(uint32_t module_id, uint32_t log_lvl,const char *fmt, ...)
{
	if( (module_id & g_dbg_enable_log) &&(log_lvl <= g_dbg_log_lvl) )
	{
		va_list va; 
		va_start(va,fmt); 
		vprintf_(fmt,va);
		va_end(va);
	}
    return;
}
#endif

inline void CUSBD_DbgMsgInit(void)
{
	DbgMsgEnableModule(DEFAULT_CDN_DBG_MODULE);
	DbgMsgSetLvl(DEFAULT_CDN_LOG_LVL); 
}

inline void DbgMsgSetLvl(uint32_t log_lvl)
{
	g_dbg_log_lvl = log_lvl ; 
	return ; 
}

void DbgMsgEnableModule(uint32_t module_id)
{
	g_dbg_enable_log |= module_id ; 
}

void DbgMsgDisableModule(uint32_t module_id)
{
	g_dbg_enable_log &= ~(module_id); 
}

