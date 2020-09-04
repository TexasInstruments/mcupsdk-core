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

/* #ifdef DEBUG */
/* parasoft-begin-suppress MISRA2012-RULE-17_1_a-2 "Example implementation of DbgPrint" */
/* parasoft-begin-suppress MISRA2012-RULE-17_1_b-2 "Example implementation of DbgPrint" */

uint32_t g_dbg_enable_log  = 0; /* DBG_GEN_MSG; */
uint32_t g_dbg_log_lvl = DBG_CRIT; /* DBG_FYI; */  /* DBG_HIVERB; */
uint32_t g_dbg_log_cnt = 0;
uint32_t g_dbg_state = 0;

/* DbgPrint is required for DEBUG build */
void DbgPrint(const char *fmt, ...)
{
    return;
}

/* parasoft-end-suppress MISRA2012-RULE-17_1_a-2 "Example implementation of DbgPrint" */
/* parasoft-end-suppress MISRA2012-RULE-17_1_b-2 "Example implementation of DbgPrint" */
/* #endif */
