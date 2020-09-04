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
#ifdef __cplusplus
extern "C"
{
#endif

#include "stdarg.h"
#include "stdio.h"
#include "stdlib.h"
#include "stddef.h"
#include "stdint.h"

/* #ifdef DEBUG */

/* DbgPrint is required for DEBUG build */
void DbgPrint(const char *fmt, ...);

void vDbgMsg(unsigned int message, unsigned int type, const char *str, ...);

/* #endif */

#ifdef __cplusplus
}
#endif


